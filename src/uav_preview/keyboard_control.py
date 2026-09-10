from __future__ import annotations

from dataclasses import asdict, dataclass
from math import radians
import socket
from threading import Event, Lock, Thread
from time import monotonic
from typing import Any, Callable

from .config import GroundOffboardTestConfig, KeyboardControlConfig
from .types import TelemetrySnapshot


# Position, acceleration/force and yaw are ignored. Body-frame vx/vy/vz and
# yaw-rate are the only active fields.
KEYBOARD_VELOCITY_TYPE_MASK = (
    (1 << 0)
    | (1 << 1)
    | (1 << 2)
    | (1 << 6)
    | (1 << 7)
    | (1 << 8)
    | (1 << 10)
)


@dataclass(slots=True)
class KeyboardControlState:
    available: bool = False
    enabled: bool = False
    mode: str = "OFF"
    frequency_hz: float = 10.0
    uplink_endpoint: str = ""
    packets_sent: int = 0
    last_send_monotonic: float | None = None
    last_input_monotonic: float | None = None
    last_stop_reason: str = "默认关闭"
    error: str = ""
    forward_m_s: float = 0.0
    right_m_s: float = 0.0
    down_m_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    watchdog_zero_active: bool = False

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        now = monotonic()
        result["last_send_age_seconds"] = (
            None if self.last_send_monotonic is None else max(0.0, now - self.last_send_monotonic)
        )
        result["last_input_age_seconds"] = (
            None if self.last_input_monotonic is None else max(0.0, now - self.last_input_monotonic)
        )
        result.pop("last_send_monotonic", None)
        result.pop("last_input_monotonic", None)
        return result


class KeyboardOffboardSetpointSender:
    """Guarded browser-keyboard to body-NED velocity setpoint sender.

    Ground inspection is allowed while PX4 is disarmed, ON_GROUND and the
    verified physical kill switch (RC channel 6 on this aircraft) is engaged.
    A separately enabled guarded takeoff path allows ARMED + ON_GROUND +
    OFFBOARD, but clamps every command except upward velocity. Full control is
    allowed only for ARMED + IN_AIR + OFFBOARD. Stale browser input first
    becomes zero, then automatically stops the sender.
    """

    def __init__(
        self,
        config: KeyboardControlConfig,
        link: GroundOffboardTestConfig,
        telemetry_snapshot: Callable[[], TelemetrySnapshot],
        estop_latched: Callable[[], bool],
        stale_after_seconds: float,
        socket_factory: Callable[[int, int], Any] = socket.socket,
    ) -> None:
        self.config = config
        self.link = link
        self._telemetry_snapshot = telemetry_snapshot
        self._estop_latched = estop_latched
        self._stale_after_seconds = stale_after_seconds
        self._socket_factory = socket_factory
        self._state = KeyboardControlState(
            available=config.available,
            frequency_hz=config.frequency_hz,
            uplink_endpoint=f"{link.uplink_host}:{link.uplink_port}",
        )
        self._axes = (0.0, 0.0, 0.0, 0.0)
        self._lock = Lock()
        self._stop = Event()
        self._wake = Event()
        self._ready = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if not self.config.available or self._thread is not None:
            return
        self._thread = Thread(target=self._run, name="keyboard-offboard", daemon=True)
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

    @staticmethod
    def _physical_kill_engaged(telemetry: TelemetrySnapshot) -> bool:
        if telemetry.rc_channel_6_pwm is not None:
            return telemetry.rc_channel_6_pwm >= 1800
        return "kill-switch engaged" in (telemetry.last_status_text or "").casefold()

    def _allowed_mode(self, telemetry: TelemetrySnapshot) -> tuple[str | None, str]:
        age = telemetry.age_seconds()
        if not telemetry.connected or age is None or age > self._stale_after_seconds:
            return None, "飞控遥测离线或已过期"
        if self._estop_latched():
            return None, "网页紧急制动已锁存"
        if telemetry.armed is False and telemetry.landed_state == "ON_GROUND":
            if not self._physical_kill_engaged(telemetry):
                return None, "地面发包测试要求实体Kill Switch保持触发"
            return "GROUND_QGC_TEST", ""
        if (
            self.config.allow_ground_takeoff
            and telemetry.armed is True
            and telemetry.landed_state == "ON_GROUND"
            and (telemetry.flight_mode or "").upper() == "OFFBOARD"
        ):
            if self._physical_kill_engaged(telemetry):
                return None, "实体Kill Switch已触发"
            return "GROUND_TAKEOFF", ""
        if (
            telemetry.armed is True
            and telemetry.landed_state == "IN_AIR"
            and (telemetry.flight_mode or "").upper() == "OFFBOARD"
        ):
            if self._physical_kill_engaged(telemetry):
                return None, "实体Kill Switch已触发"
            return "FLIGHT_OFFBOARD", ""
        return None, (
            "实际键盘控制只允许DISARMED+ON_GROUND+实体急停的地面测试，"
            "ARMED+ON_GROUND+OFFBOARD的Shift起飞，"
            "或ARMED+IN_AIR+OFFBOARD的飞行控制"
        )

    def enable(self) -> dict[str, Any]:
        if not self.config.available:
            raise ValueError("键盘Offboard发送器未开放")
        if self._thread is None:
            self.start()
        if not self._ready.is_set():
            raise ValueError("键盘Offboard发送线程尚未就绪")
        telemetry = self._telemetry_snapshot()
        mode, reason = self._allowed_mode(telemetry)
        if reason:
            raise ValueError(reason)
        with self._lock:
            self._axes = (0.0, 0.0, 0.0, 0.0)
            self._state.enabled = True
            self._state.mode = mode or "OFF"
            self._state.last_input_monotonic = monotonic()
            self._state.last_stop_reason = ""
            self._state.error = ""
            self._state.watchdog_zero_active = False
        self._wake.set()
        return self.snapshot()

    def disable(self, reason: str = "已由用户关闭") -> dict[str, Any]:
        with self._lock:
            self._state.enabled = False
            self._state.mode = "OFF"
            self._axes = (0.0, 0.0, 0.0, 0.0)
            self._state.forward_m_s = 0.0
            self._state.right_m_s = 0.0
            self._state.down_m_s = 0.0
            self._state.yaw_rate_deg_s = 0.0
            self._state.last_stop_reason = reason
            self._state.watchdog_zero_active = False
        self._wake.set()
        return self.snapshot()

    def update_axes(self, pitch: float, roll: float, throttle: float, yaw: float) -> dict[str, Any]:
        values = tuple(max(-1.0, min(1.0, float(value))) for value in (pitch, roll, throttle, yaw))
        with self._lock:
            if not self._state.enabled:
                raise ValueError("键盘真实发送尚未开启")
            self._axes = values
            self._state.last_input_monotonic = monotonic()
            self._state.watchdog_zero_active = False
        self._wake.set()
        return self.snapshot()

    def _auto_stop(self, reason: str) -> None:
        self.disable(f"自动停止：{reason}")

    def _run(self) -> None:
        try:
            from pymavlink.dialects.v20 import common as mavlink2
        except Exception as exc:
            with self._lock:
                self._state.error = f"pymavlink加载失败：{exc}"
            self._ready.set()
            return

        try:
            sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as exc:
            with self._lock:
                self._state.error = f"无法创建键盘控制UDP套接字：{exc}"
            self._ready.set()
            return

        mav = mavlink2.MAVLink(
            None,
            srcSystem=self.link.source_system,
            srcComponent=self.link.source_component,
        )
        endpoint = (self.link.uplink_host, self.link.uplink_port)
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
                mode, reason = self._allowed_mode(telemetry)
                if reason:
                    self._auto_stop(reason)
                    continue

                with self._lock:
                    input_age = (
                        float("inf")
                        if self._state.last_input_monotonic is None
                        else started - self._state.last_input_monotonic
                    )
                    if input_age > self.config.auto_stop_seconds:
                        should_stop = True
                        axes = (0.0, 0.0, 0.0, 0.0)
                    elif input_age > self.config.input_timeout_seconds:
                        should_stop = False
                        axes = (0.0, 0.0, 0.0, 0.0)
                        self._state.watchdog_zero_active = True
                    else:
                        should_stop = False
                        axes = self._axes
                        self._state.watchdog_zero_active = False
                if should_stop:
                    self._auto_stop("浏览器输入超过看门狗时限")
                    continue

                pitch, roll, throttle, yaw = axes
                if mode == "GROUND_TAKEOFF":
                    # Before PX4 reports IN_AIR, only Shift/upward velocity is
                    # accepted. Horizontal, descent and yaw input stay zero.
                    pitch = 0.0
                    roll = 0.0
                    throttle = max(0.0, throttle)
                    yaw = 0.0
                forward = pitch * self.config.max_horizontal_speed_m_s
                right = roll * self.config.max_horizontal_speed_m_s
                down = -throttle * self.config.max_vertical_speed_m_s
                yaw_rate_deg = yaw * self.config.max_yaw_rate_deg_s
                message = mav.set_position_target_local_ned_encode(
                    int(started * 1000.0) & 0xFFFFFFFF,
                    telemetry.system_id or self.link.target_system,
                    telemetry.component_id or self.link.target_component,
                    mavlink2.MAV_FRAME_BODY_NED,
                    KEYBOARD_VELOCITY_TYPE_MASK,
                    0.0,
                    0.0,
                    0.0,
                    forward,
                    right,
                    down,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    radians(yaw_rate_deg),
                )
                packet = message.pack(mav)
                with self._lock:
                    if not self._state.enabled:
                        continue
                    try:
                        sock.sendto(packet, endpoint)
                    except OSError as exc:
                        self._state.enabled = False
                        self._state.mode = "OFF"
                        self._state.error = f"键盘设定值发送失败：{exc}"
                        self._state.last_stop_reason = "自动停止：UDP发送失败"
                        continue
                    self._state.mode = mode or "OFF"
                    self._state.forward_m_s = forward
                    self._state.right_m_s = right
                    self._state.down_m_s = down
                    self._state.yaw_rate_deg_s = yaw_rate_deg
                    self._state.packets_sent += 1
                    self._state.last_send_monotonic = monotonic()

                self._wake.wait(timeout=max(0.0, period - (monotonic() - started)))
                self._wake.clear()
        finally:
            sock.close()
