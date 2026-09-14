from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import isfinite
import socket
from threading import Event, Lock, Thread
from time import monotonic
from typing import Any, Callable

from .config import PositionStreamConfig
from .types import TelemetrySnapshot


_LOCAL_POSITION_NED = 32
_GLOBAL_POSITION_INT = 33
_EXTENDED_SYS_STATE = 245
_ESTIMATOR_STATUS = 230


@dataclass(slots=True)
class PositionStreamState:
    enabled: bool = False
    status: str = "DISABLED"
    requested_local_hz: float = 0.0
    requested_global_hz: float = 0.0
    requested_extended_state_hz: float = 0.0
    requested_estimator_hz: float = 0.0
    require_global_position: bool = True
    observed_local_hz: float | None = None
    observed_global_hz: float | None = None
    observed_extended_state_hz: float | None = None
    observed_estimator_hz: float | None = None
    observed_flow_hz: float | None = None
    observed_range_hz: float | None = None
    local_effective: bool = False
    global_effective: bool = False
    extended_state_effective: bool = False
    estimator_effective: bool = False
    stream_attempts: dict[str, int] = field(default_factory=dict)
    missing_required: list[str] = field(default_factory=list)
    attempts: int = 0
    requests_sent: int = 0
    last_request_monotonic: float | None = None
    last_reason: str = ""
    error: str = ""

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["last_request_age_seconds"] = (
            None
            if self.last_request_monotonic is None
            else max(0.0, monotonic() - self.last_request_monotonic)
        )
        result.pop("last_request_monotonic", None)
        return result


class PositionStreamRateRequester:
    """Request faster position telemetry without writing any PX4 parameter.

    MAV_CMD_SET_MESSAGE_INTERVAL only changes the active MAVLink stream.  This
    worker sends requests only while PX4 explicitly reports DISARMED and
    ON_GROUND, and it never sends a flight-mode, arm, actuator, or setpoint
    message.
    """

    def __init__(
        self,
        config: PositionStreamConfig,
        telemetry_snapshot: Callable[[], TelemetrySnapshot],
        router_link_ready: Callable[[], bool],
        socket_factory: Callable[[int, int], Any] = socket.socket,
        flow_message_type: str = "OPTICAL_FLOW_RAD",
    ) -> None:
        self.config = config
        self._telemetry_snapshot = telemetry_snapshot
        self._router_link_ready = router_link_ready
        self._socket_factory = socket_factory
        if flow_message_type not in {"OPTICAL_FLOW_RAD","OPTICAL_FLOW"}:
            raise ValueError("unsupported flow telemetry message type")
        self._flow_message_type = flow_message_type
        self._state = PositionStreamState(
            enabled=config.enabled,
            status="WAITING" if config.enabled else "DISABLED",
            requested_local_hz=config.local_position_hz,
            requested_global_hz=config.global_position_hz,
            requested_extended_state_hz=config.extended_state_hz,
            requested_estimator_hz=config.estimator_hz,
            require_global_position=config.require_global_position,
            last_reason=("等待真实飞控地面遥测" if config.enabled else "功能已关闭"),
        )
        self._lock = Lock()
        self._stop = Event()
        self._thread: Thread | None = None
        self._stream_attempts: dict[int, int] = {}
        self._last_requests: dict[int, float] = {}

    def start(self) -> None:
        if not self.config.enabled or self._thread is not None:
            return
        self._thread = Thread(
            target=self._run,
            name="position-stream-rate",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._state.to_dict()

    @staticmethod
    def _age(last_monotonic: float | None, now: float) -> float | None:
        if last_monotonic is None or not isfinite(last_monotonic) or last_monotonic > now:
            return None
        return now - last_monotonic

    def _rate_effective(
        self,
        rate_hz: float | None,
        last_monotonic: float | None,
        now: float,
        minimum_hz: float | None = None,
    ) -> bool:
        age = self._age(last_monotonic, now)
        return bool(
            rate_hz is not None
            and isfinite(rate_hz)
            and rate_hz >= (
                self.config.minimum_effective_hz
                if minimum_hz is None
                else minimum_hz
            )
            and age is not None
            and age <= 1.0
        )

    def _run(self) -> None:
        while not self._stop.wait(0.2):
            self._step(monotonic())

    def _step(self, now: float) -> None:
        started = monotonic()
        if not self.config.enabled or self._stop.is_set():
            return
        telemetry = self._telemetry_snapshot()
        local_effective = self._rate_effective(
            telemetry.local_position_hz,
            telemetry.last_local_position_monotonic,
            now,
        )
        global_effective = self._rate_effective(
            telemetry.global_position_hz,
            telemetry.last_global_position_monotonic,
            now,
        )
        extended_effective = self._rate_effective(
            telemetry.extended_state_hz,
            telemetry.last_extended_state_monotonic,
            now,
            self.config.minimum_extended_state_hz,
        )
        estimator_effective = self._rate_effective(
            telemetry.estimator_hz, telemetry.last_estimator_monotonic, now,
        )
        # Independent retry budgets: missing GPS must not consume retries for
        # a local/estimator stream that becomes slow later. No endless retries.
        required = [
            (_LOCAL_POSITION_NED, "LOCAL_POSITION_NED", self.config.local_position_hz, local_effective),
            (_EXTENDED_SYS_STATE, "EXTENDED_SYS_STATE", self.config.extended_state_hz, extended_effective),
            (_ESTIMATOR_STATUS, "ESTIMATOR_STATUS", self.config.estimator_hz, estimator_effective),
        ]
        if self.config.request_flow_range:
            required.extend([
                (106 if self._flow_message_type == "OPTICAL_FLOW_RAD" else 100,
                 self._flow_message_type,self.config.flow_hz,
                 self._rate_effective(telemetry.flow_hz,telemetry.last_flow_monotonic,now)),
                (132,"DISTANCE_SENSOR",self.config.range_hz,
                 self._rate_effective(telemetry.laser_hz,telemetry.last_laser_monotonic,now)),
            ])
        if self.config.require_global_position:
            required.append((_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT/GPS", self.config.global_position_hz, global_effective))
        missing = [entry for entry in required if not entry[3]]
        with self._lock:
            self._state.observed_local_hz = telemetry.local_position_hz
            self._state.observed_global_hz = telemetry.global_position_hz
            self._state.observed_extended_state_hz = telemetry.extended_state_hz
            self._state.observed_estimator_hz = telemetry.estimator_hz
            self._state.observed_flow_hz = telemetry.flow_hz
            self._state.observed_range_hz = telemetry.laser_hz
            self._state.local_effective = local_effective
            self._state.global_effective = global_effective
            self._state.extended_state_effective = extended_effective
            self._state.estimator_effective = estimator_effective
            self._state.missing_required = [entry[1] for entry in missing]
            if not missing:
                self._state.status = "EFFECTIVE"
                self._state.last_reason = "所需遥测频率已达标（不代表定位/光流融合健康检查通过）"
                self._state.error = ""
                return

        if not self._router_link_ready():
            self._set_waiting("等待来自真实飞控的MAVLink链路")
            return
        reason = self._ground_reason(telemetry, now)
        if reason:
            self._set_waiting(reason)
            return

        available = [entry for entry in missing if self._stream_attempts.get(entry[0], 0) < self.config.max_attempts]
        if not available:
            with self._lock:
                self._state.status = "EXHAUSTED"
                self._state.last_reason = "临时请求次数已用完，仍未达标：" + "、".join(entry[1] for entry in missing)
            return

        requests = [entry for entry in available if now - self._last_requests.get(entry[0], float('-inf')) >= self.config.retry_seconds]
        for message_id, name, rate_hz, _ in requests:
            fresh = self._telemetry_snapshot()
            reason = self._ground_reason(fresh, now + max(0.0, monotonic() - started))
            if self._stop.is_set() or not self._router_link_ready() or reason or fresh.system_id != telemetry.system_id:
                self._set_waiting(reason or "链路或目标变化，停止遥测请求")
                return
            # Count failed attempts too, so socket failures cannot cause a flood.
            self._stream_attempts[message_id] = self._stream_attempts.get(message_id, 0) + 1
            self._last_requests[message_id] = now
            with self._lock:
                self._state.stream_attempts[name] = self._stream_attempts[message_id]
                self._state.attempts = max(self._stream_attempts.values())
                self._state.last_request_monotonic = now
            try:
                self._send_interval_request(fresh, message_id, rate_hz)
            except Exception as exc:
                with self._lock:
                    self._state.status = "ERROR"
                    self._state.error = f"{name} 遥测频率请求失败：{exc}"
                    self._state.last_reason = self._state.error
                return
            with self._lock:
                self._state.requests_sent += 1
                self._state.status = "REQUESTED"
                self._state.last_reason = f"已临时请求 {name} {rate_hz:g} Hz，等待实际接收频率验证"
                self._state.error = ""

    def _ground_reason(self, telemetry: TelemetrySnapshot, now: float) -> str:
        if not telemetry.connected or telemetry.system_id is None or telemetry.component_id != 1:
            return "等待飞控1号组件心跳"
        for stamp in (telemetry.last_heartbeat_monotonic, telemetry.last_extended_state_monotonic):
            age = self._age(stamp, now)
            if age is None or age > 1.5:
                return "心跳或落地状态过期，暂停遥测频率请求"
        if telemetry.armed is not False or telemetry.landed_state != "ON_GROUND":
            return "仅在DISARMED + ON_GROUND时请求遥测频率"
        return ""

    def _set_waiting(self, reason: str) -> None:
        with self._lock:
            self._state.status = "WAITING"
            self._state.last_reason = reason

    def _send_interval_request(
        self,
        telemetry: TelemetrySnapshot,
        message_id: int,
        rate_hz: float,
    ) -> None:
        from pymavlink.dialects.v20 import common as mavlink2

        mav = mavlink2.MAVLink(
            None,
            srcSystem=self.config.source_system,
            srcComponent=self.config.source_component,
        )
        interval_us = 1_000_000.0 / rate_hz
        message = mav.command_long_encode(
            telemetry.system_id or self.config.target_system,
            telemetry.component_id or self.config.target_component,
            mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id),
            interval_us,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        packet = message.pack(mav)
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(packet, (self.config.uplink_host, self.config.uplink_port))
        finally:
            sock.close()
