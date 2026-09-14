from __future__ import annotations

from dataclasses import asdict, dataclass
from math import radians, isfinite
import socket
from threading import Event, Lock, Thread
from time import monotonic
from typing import Any, Callable

from .config import GroundOffboardTestConfig, KeyboardControlConfig
from .types import TelemetrySnapshot
from .motion_plan import KeyboardMotionPlan
from .local_takeoff import LOCAL_POSITION_TYPE_MASK


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
    motion_phase: str = "OFF"
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
    awaiting_neutral: bool = False

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

    Real keyboard TX is deliberately an airborne-only function.  The pilot
    must take off with the physical RC, then explicitly select Offboard with
    the physical channel-8 mode switch.  Full control is allowed only for
    ARMED + IN_AIR + OFFBOARD with the physical kill released and channel 8
    still high.  Stale browser input first becomes zero, then automatically
    stops the sender.  The separate GroundOffboardSetpointSender remains the
    zero-only, propeller-off QGC Inspector test path.
    """

    def __init__(
        self,
        config: KeyboardControlConfig,
        link: GroundOffboardTestConfig,
        telemetry_snapshot: Callable[[], TelemetrySnapshot],
        estop_latched: Callable[[], bool],
        stale_after_seconds: float,
        socket_factory: Callable[[int, int], Any] = socket.socket,
        navigation_guard: Callable[[], str] = lambda: "",
    ) -> None:
        self.config = config
        self.link = link
        self._telemetry_snapshot = telemetry_snapshot
        self._estop_latched = estop_latched
        self._stale_after_seconds = stale_after_seconds
        self._socket_factory = socket_factory
        self._navigation_guard = navigation_guard
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
        self._browser_seen: float | None = None
        self._first_packet = Event()
        self._plan = KeyboardMotionPlan()
        self._generation = 0

    def browser_presence(self) -> None:
        with self._lock:
            self._browser_seen = monotonic()

    def enable_from_takeoff(self) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        now = monotonic()
        for stamp in (telemetry.last_heartbeat_monotonic,
                      telemetry.last_extended_state_monotonic,
                      telemetry.last_rc_channels_monotonic,
                      telemetry.last_local_position_monotonic):
            if stamp is None or now - stamp > self._stale_after_seconds:
                raise ValueError("交接所需的飞控状态或位置已过期，继续定点保持")
        with self._lock:
            if self._browser_seen is None or monotonic() - self._browser_seen > 1.0:
                raise ValueError("网页未在前台就绪，继续定点保持")
        self.enable(awaiting_neutral=True)
        if not self._first_packet.wait(0.4):
            self.disable("交接首个零速度包未发送，保持起飞定点控制")
            raise ValueError("键盘零速度首包未就绪，继续定点保持")
        if not self.snapshot()["enabled"]:
            raise ValueError("键盘发送器已停止，取消交接")
        return self.snapshot()

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

    def _physical_offboard_selected(self, telemetry: TelemetrySnapshot) -> bool:
        if not self.config.require_physical_offboard_switch:
            return True
        return bool(
            telemetry.rc_channel_8_pwm is not None
            and telemetry.rc_channel_8_pwm
            >= self.config.physical_offboard_switch_pwm_min
        )

    def _allowed_mode(self, telemetry: TelemetrySnapshot) -> tuple[str | None, str]:
        age = telemetry.age_seconds()
        if not telemetry.connected or age is None or age > self._stale_after_seconds:
            return None, "飞控遥测离线或已过期"
        if self._estop_latched():
            return None, "网页安全处置已锁存"
        if (
            telemetry.armed is True
            and telemetry.landed_state == "IN_AIR"
            and (telemetry.flight_mode or "").upper() == "OFFBOARD"
        ):
            if self._physical_kill_engaged(telemetry):
                return None, "实体Kill Switch已触发"
            if telemetry.rc_channel_8_pwm is None:
                return None, "尚未收到实体CH8模式开关值"
            if not self._physical_offboard_selected(telemetry):
                return None, "实体CH8模式开关已离开Offboard"
            now = monotonic()
            for stamp in (telemetry.last_heartbeat_monotonic, telemetry.last_extended_state_monotonic,
                          telemetry.last_rc_channels_monotonic, telemetry.last_local_position_monotonic,
                          telemetry.last_attitude_monotonic):
                if stamp is None or not isfinite(stamp) or not 0 <= now - stamp <= self._stale_after_seconds:
                    return None, "键盘控制所需独立遥测过期"
            if any(v is None or not isfinite(v) for v in (telemetry.local_x_m, telemetry.local_y_m,
                telemetry.local_z_m, telemetry.vx_m_s, telemetry.vy_m_s, telemetry.vz_m_s, telemetry.yaw_deg)):
                return None, "键盘控制所需位置/速度不是有限数值"
            health_reason = self._navigation_guard()
            if health_reason:
                return None, health_reason
            return "FLIGHT_OFFBOARD", ""
        return None, (
            "真实键盘控制只允许ARMED+IN_AIR+OFFBOARD，且实体Kill已解除、"
            "CH8物理模式开关仍选择Offboard；禁止从地面用Shift起飞"
        )

    def enable(self, *, awaiting_neutral: bool = False) -> dict[str, Any]:
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
            self._first_packet.clear()
            self._generation += 1
            self._plan = KeyboardMotionPlan()
            self._axes = (0.0, 0.0, 0.0, 0.0)
            self._state.awaiting_neutral = awaiting_neutral
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
            self._generation += 1
            self._state.motion_phase = "OFF"
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
        if any(not isfinite(float(value)) for value in (pitch, roll, throttle, yaw)):
            self.disable("拒绝非有限键盘轴输入，已停止TX")
            raise ValueError("键盘轴必须是有限数值")
        values = tuple(max(-1.0, min(1.0, float(value))) for value in (pitch, roll, throttle, yaw))
        with self._lock:
            if not self._state.enabled:
                raise ValueError("键盘真实发送尚未开启")
            if self._state.awaiting_neutral:
                if not self._first_packet.is_set():
                    raise ValueError("交接零速度首包尚未发送")
                if any(values):
                    raise ValueError("交接后必须先松开全部按键")
                self._state.awaiting_neutral = False
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
        next_send = 0.0
        try:
            while not self._stop.is_set():
                with self._lock:
                    enabled = self._state.enabled
                    generation = self._generation
                if not enabled:
                    self._wake.wait(timeout=0.5)
                    self._wake.clear()
                    continue

                started = monotonic()
                if started < next_send:
                    self._wake.wait(timeout=next_send - started)
                    self._wake.clear()
                    continue
                next_send = started + period
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
                forward = pitch * self.config.max_horizontal_speed_m_s
                right = roll * self.config.max_horizontal_speed_m_s
                down = -throttle * self.config.max_vertical_speed_m_s
                yaw_rate_deg = yaw * self.config.max_yaw_rate_deg_s
                # One producer owns both velocity and position-hold packets.
                with self._lock:
                    if not self._state.enabled or generation != self._generation:
                        continue
                    kind, goal = self._plan.step((forward, right, down, yaw_rate_deg), telemetry, started)
                    self._state.motion_phase = self._plan.phase
                if kind == "velocity":
                    forward, right, down, yaw_rate_deg = goal
                else:
                    forward = right = down = yaw_rate_deg = 0.0
                message = mav.set_position_target_local_ned_encode(
                    int(started * 1000.0) & 0xFFFFFFFF,
                    telemetry.system_id or self.link.target_system,
                    telemetry.component_id or self.link.target_component,
                    mavlink2.MAV_FRAME_LOCAL_NED if kind == "position" else mavlink2.MAV_FRAME_BODY_NED,
                    LOCAL_POSITION_TYPE_MASK if kind == "position" else KEYBOARD_VELOCITY_TYPE_MASK,
                    goal[0] if kind == "position" else 0.0,
                    goal[1] if kind == "position" else 0.0,
                    goal[2] if kind == "position" else 0.0,
                    forward,
                    right,
                    down,
                    0.0,
                    0.0,
                    0.0,
                    goal[3] if kind == "position" else 0.0,
                    radians(yaw_rate_deg),
                )
                packet = message.pack(mav)
                with self._lock:
                    if not self._state.enabled or generation != self._generation:
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
                    self._first_packet.set()

                self._wake.wait(timeout=max(0.0, period - (monotonic() - started)))
                self._wake.clear()
        finally:
            sock.close()
