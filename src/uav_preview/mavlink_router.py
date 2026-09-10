from __future__ import annotations

from dataclasses import asdict, dataclass
import select
import socket
from threading import Event, Lock, Thread
from time import monotonic
from typing import Any, Callable

from .config import MavlinkRouterConfig


_MANUAL_CONTROL = 69
_RC_CHANNELS_OVERRIDE = 70
_APPROVED_SETPOINT = 84  # SET_POSITION_TARGET_LOCAL_NED
_FLIGHT_SETPOINT_MESSAGES = {
    82,   # SET_ATTITUDE_TARGET
    84,   # SET_POSITION_TARGET_LOCAL_NED
    86,   # SET_POSITION_TARGET_GLOBAL_INT
    139,  # SET_ACTUATOR_CONTROL_TARGET
}
_MIRRORED_SETPOINT_TTL_SECONDS = 5.0


@dataclass(frozen=True, slots=True)
class _MavlinkFrame:
    message_id: int
    system_id: int
    component_id: int
    payload: bytes


def _parse_mavlink_datagram(packet: bytes) -> list[_MavlinkFrame] | None:
    """Parse concatenated MAVLink 1/2 frames without accepting arbitrary UDP."""

    frames: list[_MavlinkFrame] = []
    offset = 0
    while offset < len(packet):
        magic = packet[offset]
        if magic == 0xFD:  # MAVLink 2
            if len(packet) - offset < 12:
                return None
            payload_length = packet[offset + 1]
            signed = bool(packet[offset + 2] & 0x01)
            frame_length = 10 + payload_length + 2 + (13 if signed else 0)
            if offset + frame_length > len(packet):
                return None
            message_id = int.from_bytes(packet[offset + 7 : offset + 10], "little")
            frames.append(
                _MavlinkFrame(
                    message_id=message_id,
                    system_id=packet[offset + 5],
                    component_id=packet[offset + 6],
                    payload=packet[offset + 10 : offset + 10 + payload_length],
                )
            )
        elif magic == 0xFE:  # MAVLink 1
            if len(packet) - offset < 8:
                return None
            payload_length = packet[offset + 1]
            frame_length = 6 + payload_length + 2
            if offset + frame_length > len(packet):
                return None
            frames.append(
                _MavlinkFrame(
                    message_id=packet[offset + 5],
                    system_id=packet[offset + 3],
                    component_id=packet[offset + 4],
                    payload=packet[offset + 6 : offset + 6 + payload_length],
                )
            )
        else:
            return None
        offset += frame_length
    return frames or None


@dataclass(slots=True)
class MavlinkRouterState:
    enabled: bool = False
    ready: bool = False
    rc_priority_enforced: bool = False
    vehicle_endpoint: str = ""
    vehicle_bind: str = ""
    local_ingress: str = ""
    qgc_endpoint: str = ""
    telemetry_endpoint: str = ""
    vehicle_datagrams_received: int = 0
    local_datagrams_forwarded: int = 0
    qgc_datagrams_forwarded: int = 0
    telemetry_datagrams_forwarded: int = 0
    local_setpoints_mirrored_to_qgc: int = 0
    dropped_vehicle_datagrams: int = 0
    dropped_local_datagrams: int = 0
    blocked_manual_control: int = 0
    blocked_rc_override: int = 0
    blocked_unapproved_setpoint: int = 0
    blocked_reflected_vehicle: int = 0
    blocked_reflected_setpoint: int = 0
    blocked_malformed_uplink: int = 0
    transient_socket_errors: int = 0
    last_blocked_reason: str = ""
    last_blocked_monotonic: float | None = None
    last_vehicle_packet_monotonic: float | None = None
    last_local_packet_monotonic: float | None = None
    error: str = ""

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        now = monotonic()
        result["vehicle_age_seconds"] = (
            None
            if self.last_vehicle_packet_monotonic is None
            else max(0.0, now - self.last_vehicle_packet_monotonic)
        )
        result["local_uplink_age_seconds"] = (
            None
            if self.last_local_packet_monotonic is None
            else max(0.0, now - self.last_local_packet_monotonic)
        )
        result["last_blocked_age_seconds"] = (
            None
            if self.last_blocked_monotonic is None
            else max(0.0, now - self.last_blocked_monotonic)
        )
        result.pop("last_vehicle_packet_monotonic", None)
        result.pop("last_local_packet_monotonic", None)
        result.pop("last_blocked_monotonic", None)
        return result


class LocalMavlinkRouter:
    """Route one MiniHomer UDP session to QGC and the local web application.

    Only datagrams from the configured vehicle IP are accepted on the external
    socket. Local uplink is bound to loopback so another LAN host cannot inject
    vehicle commands. It never creates or edits MAVLink messages; when RC
    priority is enabled it drops computer-side manual/override/continuous
    setpoint commands while preserving QGC high-level flight actions.
    """

    def __init__(
        self,
        config: MavlinkRouterConfig,
        socket_factory: Callable[[int, int], Any] = socket.socket,
        selector: Callable[..., Any] = select.select,
    ) -> None:
        self.config = config
        self._socket_factory = socket_factory
        self._selector = selector
        self._state = MavlinkRouterState(
            enabled=config.enabled,
            rc_priority_enforced=config.enforce_rc_priority,
            vehicle_endpoint=f"{config.vehicle_host}:{config.vehicle_port}",
            vehicle_bind=f"{config.vehicle_bind_host}:{config.vehicle_bind_port}",
            local_ingress=f"{config.local_host}:{config.local_ingress_port}",
            qgc_endpoint=f"{config.qgc_host}:{config.qgc_port}",
            telemetry_endpoint=f"{config.telemetry_host}:{config.telemetry_port}",
        )
        self._lock = Lock()
        self._stop = Event()
        self._ready = Event()
        self._thread: Thread | None = None
        self._mirrored_setpoints: dict[
            tuple[tuple[int, int, int, bytes], ...], float
        ] = {}

    def start(self) -> None:
        if not self.config.enabled or self._thread is not None:
            return
        self._thread = Thread(target=self._run, name="local-mavlink-router", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5.0)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._state.to_dict()

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._state.ready = False
            self._state.error = message
        self._ready.set()

    def _mark_healthy(self) -> None:
        with self._lock:
            self._state.ready = True
            self._state.error = ""

    def _block_reason(self, packet: bytes) -> tuple[str, str] | None:
        frames = _parse_mavlink_datagram(packet)
        if frames is None:
            return "blocked_malformed_uplink", "已拦截无法验证的本地上行数据"

        # QGC may reflect telemetry received from this router back into its
        # local ingress. Sending an aircraft-originated frame back to the
        # aircraft creates an accelerating feedback loop, so this check is
        # independent of the manual-control policy.
        if any(frame.system_id == self.config.vehicle_system_id for frame in frames):
            return (
                "blocked_reflected_vehicle",
                f"已拦截QGC回送的飞控遥测 src={self.config.vehicle_system_id}",
            )

        if not self.config.enforce_rc_priority:
            return None

        approved_identity = (
            self.config.approved_setpoint_system,
            self.config.approved_setpoint_component,
        )
        for frame in frames:
            if frame.message_id == _MANUAL_CONTROL:
                return "blocked_manual_control", "已拦截QGC虚拟手柄 MANUAL_CONTROL"
            if frame.message_id == _RC_CHANNELS_OVERRIDE:
                return "blocked_rc_override", "已拦截电脑侧 RC_CHANNELS_OVERRIDE"
            if frame.message_id in _FLIGHT_SETPOINT_MESSAGES:
                identity = (frame.system_id, frame.component_id)
                if frame.message_id != _APPROVED_SETPOINT or identity != approved_identity:
                    return (
                        "blocked_unapproved_setpoint",
                        f"已拦截未授权设定值 msg={frame.message_id} src={identity[0]}/{identity[1]}",
                    )
        return None

    def _record_block(self, counter: str, reason: str) -> None:
        with self._lock:
            setattr(self._state, counter, getattr(self._state, counter) + 1)
            self._state.last_blocked_reason = reason
            self._state.last_blocked_monotonic = monotonic()

    def _is_approved_setpoint(self, packet: bytes) -> bool:
        frames = _parse_mavlink_datagram(packet)
        if not frames:
            return False
        approved_identity = (
            self.config.approved_setpoint_system,
            self.config.approved_setpoint_component,
        )
        return any(
            frame.message_id == _APPROVED_SETPOINT
            and (frame.system_id, frame.component_id) == approved_identity
            for frame in frames
        )

    @staticmethod
    def _packet_fingerprint(
        packet: bytes,
    ) -> tuple[tuple[int, int, int, bytes], ...] | None:
        """Ignore MAVLink sequence/CRC so QGC repacking cannot evade loop detection."""

        frames = _parse_mavlink_datagram(packet)
        if not frames:
            return None
        return tuple(
            (frame.message_id, frame.system_id, frame.component_id, frame.payload)
            for frame in frames
        )

    def _remember_mirrored_setpoint(self, packet: bytes) -> None:
        fingerprint = self._packet_fingerprint(packet)
        if fingerprint is None:
            return
        now = monotonic()
        self._mirrored_setpoints = {
            key: expires_at
            for key, expires_at in self._mirrored_setpoints.items()
            if expires_at > now
        }
        self._mirrored_setpoints[fingerprint] = (
            now + _MIRRORED_SETPOINT_TTL_SECONDS
        )

    def _is_reflected_setpoint(self, packet: bytes) -> bool:
        fingerprint = self._packet_fingerprint(packet)
        if fingerprint is None:
            return False
        now = monotonic()
        expires_at = self._mirrored_setpoints.get(fingerprint)
        if expires_at is None:
            return False
        if expires_at <= now:
            self._mirrored_setpoints.pop(fingerprint, None)
            return False
        return True

    def _run(self) -> None:
        vehicle_sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        local_sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            vehicle_sock.bind(
                (self.config.vehicle_bind_host, self.config.vehicle_bind_port)
            )
            local_sock.bind((self.config.local_host, self.config.local_ingress_port))
            vehicle_sock.setblocking(False)
            local_sock.setblocking(False)
        except OSError as exc:
            vehicle_sock.close()
            local_sock.close()
            self._set_error(
                f"MAVLink路由器无法占用端口：{exc}。请先让QGC退出直接监听8080。"
            )
            return

        vehicle_endpoint = (self.config.vehicle_host, self.config.vehicle_port)
        qgc_endpoint = (self.config.qgc_host, self.config.qgc_port)
        telemetry_endpoint = (
            self.config.telemetry_host,
            self.config.telemetry_port,
        )
        with self._lock:
            self._state.ready = True
            self._state.error = ""
        self._ready.set()

        try:
            while not self._stop.is_set():
                try:
                    readable, _, _ = self._selector(
                        [vehicle_sock, local_sock], [], [], 0.5
                    )
                except OSError as exc:
                    self._set_error(f"MAVLink路由监听失败：{exc}")
                    continue

                for ready_sock in readable:
                    try:
                        packet, address = ready_sock.recvfrom(65535)
                    except (BlockingIOError, TimeoutError):
                        continue
                    except OSError as exc:
                        if getattr(exc, "winerror", None) == 10054:
                            # Windows reports an ICMP UDP port-unreachable on a
                            # later recvfrom. QGC restarts can cause this briefly;
                            # it must not permanently mark a healthy radio link
                            # as offline.
                            with self._lock:
                                self._state.transient_socket_errors += 1
                            continue
                        self._set_error(f"MAVLink路由接收失败：{exc}")
                        continue

                    if ready_sock is vehicle_sock:
                        if address[0] != self.config.vehicle_host:
                            with self._lock:
                                self._state.dropped_vehicle_datagrams += 1
                            continue
                        try:
                            local_sock.sendto(packet, qgc_endpoint)
                            local_sock.sendto(packet, telemetry_endpoint)
                        except OSError as exc:
                            self._set_error(f"MAVLink本地分流失败：{exc}")
                            continue
                        with self._lock:
                            self._state.vehicle_datagrams_received += 1
                            self._state.qgc_datagrams_forwarded += 1
                            self._state.telemetry_datagrams_forwarded += 1
                            self._state.last_vehicle_packet_monotonic = monotonic()
                        self._mark_healthy()
                    else:
                        if address[0] != self.config.local_host:
                            with self._lock:
                                self._state.dropped_local_datagrams += 1
                            continue
                        if self._is_reflected_setpoint(packet):
                            self._record_block(
                                "blocked_reflected_setpoint",
                                "已拦截QGC回送的应用速度设定值",
                            )
                            continue
                        blocked = self._block_reason(packet)
                        if blocked is not None:
                            self._record_block(*blocked)
                            continue
                        try:
                            # Sending through the vehicle-facing socket preserves
                            # source port 8080, matching the proven QGC session.
                            vehicle_sock.sendto(packet, vehicle_endpoint)
                            approved_setpoint = self._is_approved_setpoint(packet)
                            if approved_setpoint:
                                # Mirror only the application's approved velocity
                                # setpoint into QGC so MAVLink Inspector can verify
                                # keyboard axes during propeller-off ground tests.
                                self._remember_mirrored_setpoint(packet)
                                local_sock.sendto(packet, qgc_endpoint)
                        except OSError as exc:
                            self._set_error(f"MAVLink本地上行失败：{exc}")
                            continue
                        with self._lock:
                            self._state.local_datagrams_forwarded += 1
                            if approved_setpoint:
                                self._state.local_setpoints_mirrored_to_qgc += 1
                            self._state.last_local_packet_monotonic = monotonic()
                        self._mark_healthy()
        finally:
            vehicle_sock.close()
            local_sock.close()
