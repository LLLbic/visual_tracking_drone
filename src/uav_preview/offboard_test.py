from __future__ import annotations

from dataclasses import asdict, dataclass
import socket
from threading import Event, Lock, Thread
from time import monotonic
from typing import Any, Callable

from .config import GroundOffboardTestConfig
from .types import TelemetrySnapshot


# SET_POSITION_TARGET_LOCAL_NED ignore bits. Velocity X/Y/Z are deliberately
# the only active fields, and all three are always encoded as 0.0 m/s.
ZERO_VELOCITY_TYPE_MASK = (
    (1 << 0)  # position x
    | (1 << 1)  # position y
    | (1 << 2)  # position z
    | (1 << 6)  # acceleration x
    | (1 << 7)  # acceleration y
    | (1 << 8)  # acceleration z
    | (1 << 10)  # yaw
    | (1 << 11)  # yaw rate
)


@dataclass(slots=True)
class GroundOffboardTestState:
    available: bool = False
    enabled: bool = False
    frequency_hz: float = 5.0
    uplink_endpoint: str = ""
    packets_sent: int = 0
    session_packets_sent: int = 0
    enabled_since_monotonic: float | None = None
    last_send_monotonic: float | None = None
    ready_for_offboard_arm: bool = False
    last_stop_reason: str = "默认关闭"
    error: str = ""
    message_type: str = "SET_POSITION_TARGET_LOCAL_NED"
    type_mask: int = ZERO_VELOCITY_TYPE_MASK
    velocity_ned_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["last_send_age_seconds"] = (
            None
            if self.last_send_monotonic is None
            else max(0.0, monotonic() - self.last_send_monotonic)
        )
        result["enabled_for_seconds"] = (
            None
            if self.enabled_since_monotonic is None
            else max(0.0, monotonic() - self.enabled_since_monotonic)
        )
        result.pop("last_send_monotonic", None)
        result.pop("enabled_since_monotonic", None)
        return result


class GroundOffboardSetpointSender:
    """Manual 5 Hz Offboard proof-of-life and guarded ARM preparation.

    The class has no COMMAND_LONG, SET_MODE, arm/disarm, RC override or manual
    control path. Its sole outbound message is a zero-velocity local-NED
    setpoint. After a stable pre-stream it may remain active through a guarded
    ground ARM in OFFBOARD, but it never arms or changes mode by itself.
    """

    def __init__(
        self,
        config: GroundOffboardTestConfig,
        telemetry_snapshot: Callable[[], TelemetrySnapshot],
        estop_latched: Callable[[], bool],
        stale_after_seconds: float,
        socket_factory: Callable[[int, int], Any] = socket.socket,
    ) -> None:
        self.config = config
        self._telemetry_snapshot = telemetry_snapshot
        self._estop_latched = estop_latched
        self._stale_after_seconds = stale_after_seconds
        self._socket_factory = socket_factory
        self._state = GroundOffboardTestState(
            available=config.available,
            frequency_hz=config.frequency_hz,
            uplink_endpoint=f"{config.uplink_host}:{config.uplink_port}",
        )
        self._lock = Lock()
        self._stop = Event()
        self._wake = Event()
        self._ready = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if not self.config.available or self._thread is not None:
            return
        self._thread = Thread(target=self._run, name="ground-offboard-test", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5.0)

    def stop(self) -> None:
        self.disable("应用已停止")
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._state.to_dict()

    def enable(self) -> dict[str, Any]:
        if not self.config.available:
            raise ValueError("地面 Offboard 预备流未在配置中开放")
        if self._thread is None:
            self.start()
        if not self._ready.is_set():
            raise ValueError("地面 Offboard 预备流发送线程尚未就绪")
        with self._lock:
            if self._state.error:
                raise ValueError(self._state.error)
        reason = self._unsafe_reason(self._telemetry_snapshot())
        if reason:
            raise ValueError(reason)
        with self._lock:
            self._state.enabled = True
            self._state.enabled_since_monotonic = monotonic()
            self._state.session_packets_sent = 0
            self._state.ready_for_offboard_arm = False
            self._state.last_stop_reason = ""
            self._state.error = ""
        self._wake.set()
        return self.snapshot()

    def disable(self, reason: str = "已由用户关闭") -> dict[str, Any]:
        with self._lock:
            self._state.enabled = False
            self._state.enabled_since_monotonic = None
            self._state.session_packets_sent = 0
            self._state.ready_for_offboard_arm = False
            self._state.last_stop_reason = reason
        self._wake.set()
        return self.snapshot()

    def _unsafe_reason(self, telemetry: TelemetrySnapshot) -> str:
        if self._estop_latched():
            return "本地急停已锁存，拒绝发送"
        age = telemetry.age_seconds()
        if (
            not telemetry.connected
            or age is None
            or age > self._stale_after_seconds
        ):
            return "飞控遥测离线或已过期，拒绝发送"
        if telemetry.landed_state != "ON_GROUND":
            return f"仅允许飞控明确报告ON_GROUND；当前为{telemetry.landed_state}"
        if telemetry.armed is False:
            return ""
        if telemetry.armed is True:
            mode = (telemetry.flight_mode or "UNKNOWN").upper().replace(" ", "_")
            with self._lock:
                ready = self._state.ready_for_offboard_arm
            if telemetry.rc_channel_6_pwm is None:
                return "已解锁后未收到实体Kill Switch通道值"
            if telemetry.rc_channel_6_pwm >= 1800:
                return "实体Kill Switch已触发"
            if ready and mode == "OFFBOARD":
                return ""
            return "检测到已解锁，但零速度预备尚未就绪或飞控不在OFFBOARD"
        return "飞控解锁状态未知，拒绝发送"

    def _auto_stop(self, reason: str) -> None:
        with self._lock:
            if self._state.enabled:
                self._state.enabled = False
                self._state.last_stop_reason = reason

    def _run(self) -> None:
        try:
            from pymavlink.dialects.v20 import common as mavlink2
        except Exception as exc:
            with self._lock:
                self._state.error = f"pymavlink 未安装或加载失败：{exc}"
            self._ready.set()
            return

        try:
            sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as exc:
            with self._lock:
                self._state.error = f"无法创建地面测试 UDP 套接字：{exc}"
            self._ready.set()
            return

        mav = mavlink2.MAVLink(
            None,
            srcSystem=self.config.source_system,
            srcComponent=self.config.source_component,
        )
        endpoint = (self.config.uplink_host, self.config.uplink_port)
        period = 1.0 / self.config.frequency_hz
        self._ready.set()

        try:
            while not self._stop.is_set():
                with self._lock:
                    enabled = self._state.enabled
                if not enabled:
                    self._wake.wait(timeout=0.5)
                    self._wake.clear()
                    continue

                started = monotonic()
                telemetry = self._telemetry_snapshot()
                reason = self._unsafe_reason(telemetry)
                if reason:
                    self._auto_stop(f"自动停止：{reason}")
                    continue

                target_system = telemetry.system_id or self.config.target_system
                target_component = telemetry.component_id or self.config.target_component
                message = mav.set_position_target_local_ned_encode(
                    int(started * 1000.0) & 0xFFFFFFFF,
                    target_system,
                    target_component,
                    mavlink2.MAV_FRAME_LOCAL_NED,
                    ZERO_VELOCITY_TYPE_MASK,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
                packet = message.pack(mav)

                # Keep the enabled check and send under one lock: once disable()
                # returns, no later packet can be emitted by a stale loop cycle.
                with self._lock:
                    if not self._state.enabled:
                        continue
                    try:
                        sock.sendto(packet, endpoint)
                    except OSError as exc:
                        self._state.enabled = False
                        self._state.error = f"零速度设定值发送失败：{exc}"
                        self._state.last_stop_reason = "自动停止：UDP 发送失败"
                        continue
                    self._state.packets_sent += 1
                    self._state.session_packets_sent += 1
                    self._state.last_send_monotonic = monotonic()
                    enabled_for = (
                        0.0
                        if self._state.enabled_since_monotonic is None
                        else self._state.last_send_monotonic
                        - self._state.enabled_since_monotonic
                    )
                    self._state.ready_for_offboard_arm = bool(
                        self._state.session_packets_sent >= 5
                        and enabled_for >= self.config.arm_ready_after_seconds
                    )

                remaining = max(0.0, period - (monotonic() - started))
                self._wake.wait(timeout=remaining)
                self._wake.clear()
        finally:
            sock.close()
