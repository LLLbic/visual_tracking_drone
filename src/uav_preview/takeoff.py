from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from threading import Event, RLock, Thread
from time import monotonic
from typing import Any, Callable

from .config import TakeoffConfig
from .flight_actions import ExplicitFlightActionSender
from .types import TelemetrySnapshot


_MANUAL_MODES = {"MANUAL", "STABILIZED", "ALTCTL", "POSCTL", "POSITION"}
_ACK_ACCEPTED = {0, 5}  # MAV_RESULT_ACCEPTED / IN_PROGRESS
_ACK_REJECTED = {1, 2, 3, 4, 6}


@dataclass(slots=True)
class TakeoffState:
    available: bool = False
    min_height_m: float = 1.0
    max_height_m: float = 3.0
    default_height_m: float = 1.5
    active: bool = False
    phase: str = "IDLE"
    target_height_m: float | None = None
    current_height_m: float | None = None
    height_error_m: float | None = None
    progress_pct: float = 0.0
    ground_amsl_m: float | None = None
    start_local_x_m: float | None = None
    start_local_y_m: float | None = None
    start_local_z_m: float | None = None
    start_relative_altitude_m: float | None = None
    started_monotonic: float | None = None
    phase_started_monotonic: float | None = None
    arm_sent_monotonic: float | None = None
    takeoff_sent_monotonic: float | None = None
    arm_acknowledged: bool = False
    takeoff_acknowledged: bool = False
    arm_ack_result: int | None = None
    takeoff_ack_result: int | None = None
    hover_stable_since_monotonic: float | None = None
    last_action: str = ""
    last_reason: str = "等待用户启动"
    error: str = ""

    def to_dict(self, now: float) -> dict[str, Any]:
        result = asdict(self)
        for field_name in (
            "started_monotonic",
            "phase_started_monotonic",
            "arm_sent_monotonic",
            "takeoff_sent_monotonic",
            "hover_stable_since_monotonic",
        ):
            value = result.pop(field_name)
            result[field_name.replace("_monotonic", "_age_seconds")] = (
                None if value is None else max(0.0, now - value)
            )
        return result


class TakeoffCoordinator:
    """Guarded one-shot ARM -> native PX4 Takeoff monitor.

    This coordinator never streams setpoints, changes PX4 parameters, changes
    flight mode directly or overrides RC.  It sends at most one ARM command and
    one MAV_CMD_NAV_TAKEOFF command per run.  Physical CH6/CH8 activity is
    treated as pilot takeover and is never countermanded.
    """

    def __init__(
        self,
        config: TakeoffConfig,
        telemetry_snapshot: Callable[[], TelemetrySnapshot],
        flight_actions: ExplicitFlightActionSender,
        other_sender_enabled: Callable[[], bool],
        estop_latched: Callable[[], bool],
        router_ready: Callable[[], bool],
        stale_after_seconds: float,
        physical_offboard_switch_pwm_min: int = 1800,
        clock: Callable[[], float] = monotonic,
        navigation_guard: Callable[[], str] = lambda: "",
    ) -> None:
        self.config = config
        self._telemetry_snapshot = telemetry_snapshot
        self._flight_actions = flight_actions
        self._other_sender_enabled = other_sender_enabled
        self._estop_latched = estop_latched
        self._router_ready = router_ready
        self._stale_after_seconds = stale_after_seconds
        self._physical_offboard_switch_pwm_min = physical_offboard_switch_pwm_min
        self._clock = clock
        self._navigation_guard = navigation_guard
        self._state = TakeoffState(
            available=config.available,
            min_height_m=config.min_height_m,
            max_height_m=config.max_height_m,
            default_height_m=config.default_height_m,
        )
        self._lock = RLock()
        self._stop = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = Thread(target=self._run, name="takeoff-coordinator", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._state.to_dict(self._clock())

    def begin(self, target_height_m: float) -> dict[str, Any]:
        now = self._clock()
        telemetry = self._telemetry_snapshot()
        height = float(target_height_m)
        with self._lock:
            if not self.config.available:
                raise ValueError("指定高度起飞功能未开放")
            if self._state.active:
                raise ValueError(f"起飞状态机正在运行：{self._state.phase}")
            if not self.config.min_height_m <= height <= self.config.max_height_m:
                raise ValueError("起飞高度必须在1.0至3.0米之间")
            reason = self._preflight_reason(telemetry, now)
            if reason:
                raise ValueError(reason)
            self._state = TakeoffState(
                available=True,
                min_height_m=self.config.min_height_m,
                max_height_m=self.config.max_height_m,
                default_height_m=self.config.default_height_m,
                active=True,
                phase="PRECHECK",
                target_height_m=height,
                ground_amsl_m=telemetry.global_altitude_amsl_m,
                start_local_x_m=telemetry.local_x_m,
                start_local_y_m=telemetry.local_y_m,
                start_local_z_m=telemetry.local_z_m,
                start_relative_altitude_m=telemetry.relative_altitude_m,
                started_monotonic=now,
                phase_started_monotonic=now,
                last_reason=f"地面条件需连续稳定{self.config.preflight_stable_seconds:.1f}秒",
            )
            return self._state.to_dict(now)

    def abort(self, reason: str = "用户取消指定高度起飞") -> dict[str, Any]:
        with self._lock:
            if not self._state.active:
                raise ValueError("指定高度起飞状态机当前没有运行")
            telemetry = self._telemetry_snapshot()
            self._safe_terminal("ABORTED", reason, telemetry)
            return self._state.to_dict(self._clock())

    def cancel_without_action(self, reason: str) -> dict[str, Any]:
        """Stop monitoring because another safety/pilot action owns control."""

        with self._lock:
            if self._state.active:
                self._state.active = False
                self._state.phase = "PILOT_TAKEOVER"
                self._state.phase_started_monotonic = self._clock()
                self._state.last_reason = reason
                self._state.error = ""
            return self._state.to_dict(self._clock())

    def poll_once(self) -> None:
        """Run one deterministic state-machine step (also used by tests)."""

        now = self._clock()
        with self._lock:
            if not self._state.active:
                return
            telemetry = self._telemetry_snapshot()
            self._update_height(telemetry)

            if self._physical_takeover(telemetry):
                self._state.active = False
                self._state.phase = "PILOT_TAKEOVER"
                self._state.phase_started_monotonic = now
                self._state.last_reason = "检测到实体CH6/CH8动作，已停止电脑起飞流程并交还飞手"
                return
            if self._estop_latched():
                # Runtime owns the single LAND/DISARM decision for a latch
                # transition. Avoid racing it with a duplicate command here.
                self._state.active = False
                self._state.phase = "FAILED"
                self._state.phase_started_monotonic = now
                self._state.last_reason = (
                    "网页安全处置已锁存；起飞流程已停止，由安全处置流程统一执行飞行动作"
                )
                self._state.error = self._state.last_reason
                return
            if not self._telemetry_fresh(telemetry, now):
                # An uplink action cannot be trusted when the return telemetry is
                # stale. PX4/RC failsafes own the vehicle from this point.
                self._state.active = False
                self._state.phase = "FAILED"
                self._state.phase_started_monotonic = now
                self._state.last_reason = "遥测或位置数据过期，停止发送；请立即用实体遥控器接管"
                self._state.error = self._state.last_reason
                return

            if self._state.phase == "PRECHECK":
                self._tick_precheck(telemetry, now)
            elif self._state.phase == "ARMING":
                self._tick_arming(telemetry, now)
            elif self._state.phase in {"TAKEOFF_REQUESTED", "CLIMBING", "HOVER_VERIFY"}:
                self._tick_flight(telemetry, now)

    def _run(self) -> None:
        while not self._stop.wait(0.05):
            try:
                self.poll_once()
            except Exception as exc:  # keep the safety monitor alive
                with self._lock:
                    if self._state.active:
                        self._state.active = False
                        self._state.phase = "FAILED"
                        self._state.phase_started_monotonic = self._clock()
                        self._state.error = f"起飞状态机内部错误：{exc}"
                        self._state.last_reason = self._state.error

    def _tick_precheck(self, telemetry: TelemetrySnapshot, now: float) -> None:
        reason = self._preflight_reason(telemetry, now)
        if reason:
            self._state.active = False
            self._state.phase = "FAILED"
            self._state.phase_started_monotonic = now
            self._state.last_reason = reason
            self._state.error = reason
            return
        assert self._state.phase_started_monotonic is not None
        if now - self._state.phase_started_monotonic < self.config.preflight_stable_seconds:
            return
        # Refresh baselines after the stable interval and before ARM.
        self._state.ground_amsl_m = telemetry.global_altitude_amsl_m
        self._state.start_local_x_m = telemetry.local_x_m
        self._state.start_local_y_m = telemetry.local_y_m
        self._state.start_local_z_m = telemetry.local_z_m
        self._state.start_relative_altitude_m = telemetry.relative_altitude_m
        self._state.arm_sent_monotonic = now
        try:
            self._flight_actions.set_armed(True)
        except ValueError as exc:
            self._state.active = False
            self._state.phase = "FAILED"
            self._state.last_reason = str(exc)
            self._state.error = str(exc)
            return
        self._state.phase = "ARMING"
        self._state.phase_started_monotonic = now
        self._state.last_action = "已发送ARM，等待飞控确认"
        self._state.last_reason = "等待COMMAND_ACK与ARMED心跳"

    def _tick_arming(self, telemetry: TelemetrySnapshot, now: float) -> None:
        assert self._state.arm_sent_monotonic is not None
        ack = self._ack_for(400, self._state.arm_sent_monotonic, telemetry)
        if ack in _ACK_ACCEPTED:
            self._state.arm_acknowledged = True
            self._state.arm_ack_result = ack
        if ack in _ACK_REJECTED:
            self._state.arm_ack_result = ack
            self._safe_terminal("FAILED", f"飞控拒绝ARM（MAV_RESULT={ack}）", telemetry)
            return
        if (
            not self._state.arm_acknowledged
            and now - self._state.arm_sent_monotonic > self.config.arm_ack_timeout_seconds
        ):
            self._safe_terminal("FAILED", "等待ARM确认超时", telemetry)
            return
        if (
            self._state.arm_acknowledged
            and telemetry.armed is True
            and now - self._state.arm_sent_monotonic
            >= self.config.arm_to_takeoff_delay_seconds
        ):
            assert self._state.target_height_m is not None
            assert self._state.ground_amsl_m is not None
            self._state.takeoff_sent_monotonic = now
            try:
                self._flight_actions.takeoff(
                    self._state.target_height_m,
                    self._state.ground_amsl_m,
                )
            except ValueError as exc:
                self._safe_terminal("FAILED", str(exc), telemetry)
                return
            self._state.phase = "TAKEOFF_REQUESTED"
            self._state.phase_started_monotonic = now
            self._state.last_action = (
                f"已发送PX4原生Takeoff {self._state.target_height_m:.1f}米"
            )
            self._state.last_reason = "等待飞控确认并离地"

    def _tick_flight(self, telemetry: TelemetrySnapshot, now: float) -> None:
        assert self._state.takeoff_sent_monotonic is not None
        ack = self._ack_for(22, self._state.takeoff_sent_monotonic, telemetry)
        if ack in _ACK_ACCEPTED:
            self._state.takeoff_acknowledged = True
            self._state.takeoff_ack_result = ack
        if ack in _ACK_REJECTED:
            self._state.takeoff_ack_result = ack
            self._safe_terminal("FAILED", f"飞控拒绝Takeoff（MAV_RESULT={ack}）", telemetry)
            return
        since_command = now - self._state.takeoff_sent_monotonic
        if (
            not self._state.takeoff_acknowledged
            and since_command > self.config.takeoff_ack_timeout_seconds
        ):
            self._safe_terminal("FAILED", "等待Takeoff确认超时", telemetry)
            return

        height = self._state.current_height_m or 0.0
        if self._state.phase == "TAKEOFF_REQUESTED":
            lifted = telemetry.landed_state in {"TAKEOFF", "IN_AIR"} or height >= self.config.liftoff_height_m
            if self._state.takeoff_acknowledged and lifted:
                self._state.phase = "CLIMBING"
                self._state.phase_started_monotonic = now
                self._state.last_reason = "飞控已确认起飞，监测高度、姿态和水平漂移"
            elif since_command > self.config.liftoff_timeout_seconds:
                self._safe_terminal("FAILED", "起飞命令已确认但未在时限内离地", telemetry)
            return

        mode = (telemetry.flight_mode or "UNKNOWN").upper().replace(" ", "_")
        if mode in _MANUAL_MODES:
            self._state.active = False
            self._state.phase = "PILOT_TAKEOVER"
            self._state.phase_started_monotonic = now
            self._state.last_reason = f"飞行模式已变为{mode}，视为实体遥控器接管"
            return
        if telemetry.armed is not True:
            self._state.active = False
            self._state.phase = "FAILED"
            self._state.phase_started_monotonic = now
            self._state.last_reason = "爬升过程中飞控已DISARM"
            self._state.error = self._state.last_reason
            return

        roll = abs(float(telemetry.roll_deg or 0.0))
        pitch = abs(float(telemetry.pitch_deg or 0.0))
        if max(roll, pitch) > self.config.max_tilt_deg:
            self._safe_terminal("FAILED", "机体倾斜超过安全阈值，已请求LAND", telemetry)
            return
        drift = self._horizontal_drift(telemetry)
        if drift is not None and drift > self.config.max_horizontal_drift_m:
            self._safe_terminal("FAILED", "水平漂移超过安全阈值，已请求LAND", telemetry)
            return
        assert self._state.target_height_m is not None
        if height > self._state.target_height_m + 0.5:
            self._safe_terminal("FAILED", "高度超调超过0.5米，已请求LAND", telemetry)
            return
        if since_command > self.config.climb_timeout_seconds:
            self._safe_terminal("FAILED", "达到目标高度超时，已请求LAND", telemetry)
            return

        error = abs(self._state.target_height_m - height)
        vertical_stable = (
            telemetry.vz_m_s is not None
            and abs(float(telemetry.vz_m_s)) <= self.config.vertical_speed_tolerance_m_s
        )
        if error <= self.config.height_tolerance_m and vertical_stable:
            if self._state.phase != "HOVER_VERIFY":
                self._state.phase = "HOVER_VERIFY"
                self._state.phase_started_monotonic = now
                self._state.hover_stable_since_monotonic = now
                self._state.last_reason = "目标高度已到达，验证稳定悬停"
            assert self._state.hover_stable_since_monotonic is not None
            if now - self._state.hover_stable_since_monotonic >= self.config.hover_stable_seconds:
                self._state.active = False
                self._state.phase = "COMPLETE"
                self._state.phase_started_monotonic = now
                self._state.progress_pct = 100.0
                self._state.last_reason = (
                    "指定高度起飞完成；PX4保持当前模式，程序未自动切换Offboard"
                )
            return

        if self._state.phase == "HOVER_VERIFY":
            self._state.phase = "CLIMBING"
            self._state.phase_started_monotonic = now
            self._state.hover_stable_since_monotonic = None
            self._state.last_reason = "高度/垂直速度离开容差，继续监测"

    def _preflight_reason(self, telemetry: TelemetrySnapshot, now: float) -> str:
        reason = self._navigation_guard()
        if reason:
            return reason
        if not self._router_ready():
            return "MAVLink路由尚未就绪"
        if self._estop_latched():
            return "网页安全处置仍处于锁存状态"
        if self._other_sender_enabled():
            return "起飞前必须关闭地面测试流和键盘真实TX"
        if not self._telemetry_fresh(telemetry, now):
            return "遥测、本地位置或全球位置数据离线/过期"
        if telemetry.armed is not False or telemetry.landed_state != "ON_GROUND":
            return "起飞只允许从DISARMED + ON_GROUND开始"
        mode = (telemetry.flight_mode or "UNKNOWN").upper().replace(" ", "_")
        if mode not in _MANUAL_MODES:
            return f"起飞前必须处于Position/Altitude/Manual；当前为{mode}"
        if telemetry.rc_channel_6_pwm is None or telemetry.rc_channel_6_pwm >= 1800:
            return "实体Kill Switch（CH6）必须已解除且有实时数值"
        if (
            telemetry.rc_channel_8_pwm is None
            or telemetry.rc_channel_8_pwm >= self._physical_offboard_switch_pwm_min
        ):
            return "实体CH8必须保持Position/手动辅助位置"
        if telemetry.roll_deg is None or telemetry.pitch_deg is None:
            return "缺少实时姿态数据"
        if max(abs(telemetry.roll_deg), abs(telemetry.pitch_deg)) > self.config.preflight_max_tilt_deg:
            return "机体不水平，超过起飞前姿态阈值"
        globals_ = (
            telemetry.latitude_deg,
            telemetry.longitude_deg,
            telemetry.global_altitude_amsl_m,
        )
        if any(value is None or not math.isfinite(float(value)) for value in globals_):
            return "缺少有效的全球位置/海拔数据"
        if abs(float(telemetry.latitude_deg)) > 90 or abs(float(telemetry.longitude_deg)) > 180:
            return "全球位置数据超出有效范围"
        if (
            abs(float(telemetry.latitude_deg)) < 1e-7
            and abs(float(telemetry.longitude_deg)) < 1e-7
        ):
            return "全球位置仍为0,0，定位尚未有效"
        locals_ = (telemetry.local_x_m, telemetry.local_y_m, telemetry.local_z_m)
        if any(value is None or not math.isfinite(float(value)) for value in locals_):
            return "缺少有效的本地位置数据"
        return ""

    def _telemetry_fresh(self, telemetry: TelemetrySnapshot, now: float) -> bool:
        age = telemetry.age_seconds(now)
        local_age = (
            None
            if telemetry.last_local_position_monotonic is None
            else max(0.0, now - telemetry.last_local_position_monotonic)
        )
        global_age = (
            None
            if telemetry.last_global_position_monotonic is None
            else max(0.0, now - telemetry.last_global_position_monotonic)
        )
        heartbeat_age = (
            None
            if telemetry.last_heartbeat_monotonic is None
            else max(0.0, now - telemetry.last_heartbeat_monotonic)
        )
        attitude_age = (
            None
            if telemetry.last_attitude_monotonic is None
            else max(0.0, now - telemetry.last_attitude_monotonic)
        )
        extended_state_age = (
            None
            if telemetry.last_extended_state_monotonic is None
            else max(0.0, now - telemetry.last_extended_state_monotonic)
        )
        rc_age = (
            None
            if telemetry.last_rc_channels_monotonic is None
            else max(0.0, now - telemetry.last_rc_channels_monotonic)
        )
        return bool(
            telemetry.connected
            and age is not None
            and age <= self._stale_after_seconds
            and local_age is not None
            and local_age <= self.config.position_stale_seconds
            and global_age is not None
            and global_age <= self.config.position_stale_seconds
            and heartbeat_age is not None
            and heartbeat_age <= self._stale_after_seconds
            and attitude_age is not None
            and attitude_age <= self._stale_after_seconds
            and extended_state_age is not None
            and extended_state_age <= self._stale_after_seconds
            and rc_age is not None
            and rc_age <= self._stale_after_seconds
        )

    def _physical_takeover(self, telemetry: TelemetrySnapshot) -> bool:
        return bool(
            telemetry.rc_channel_6_pwm is not None
            and telemetry.rc_channel_6_pwm >= 1800
            or telemetry.rc_channel_8_pwm is not None
            and telemetry.rc_channel_8_pwm >= self._physical_offboard_switch_pwm_min
        )

    def _ack_for(
        self,
        command: int,
        sent_monotonic: float,
        telemetry: TelemetrySnapshot,
    ) -> int | None:
        if (
            telemetry.last_command_ack_command == command
            and telemetry.last_command_ack_monotonic is not None
            and telemetry.last_command_ack_monotonic >= sent_monotonic
        ):
            return telemetry.last_command_ack_result
        return None

    def _update_height(self, telemetry: TelemetrySnapshot) -> None:
        height: float | None = None
        if self._state.start_local_z_m is not None and telemetry.local_z_m is not None:
            height = self._state.start_local_z_m - telemetry.local_z_m
        elif (
            self._state.start_relative_altitude_m is not None
            and telemetry.relative_altitude_m is not None
        ):
            height = telemetry.relative_altitude_m - self._state.start_relative_altitude_m
        if height is None or not math.isfinite(height):
            return
        height = max(0.0, height)
        self._state.current_height_m = height
        if self._state.target_height_m is not None:
            self._state.height_error_m = self._state.target_height_m - height
            self._state.progress_pct = max(
                0.0,
                min(100.0, 100.0 * height / self._state.target_height_m),
            )

    def _horizontal_drift(self, telemetry: TelemetrySnapshot) -> float | None:
        values = (
            self._state.start_local_x_m,
            self._state.start_local_y_m,
            telemetry.local_x_m,
            telemetry.local_y_m,
        )
        if any(value is None for value in values):
            return None
        return math.hypot(
            float(telemetry.local_x_m) - float(self._state.start_local_x_m),
            float(telemetry.local_y_m) - float(self._state.start_local_y_m),
        )

    def _safe_terminal(
        self,
        phase: str,
        reason: str,
        telemetry: TelemetrySnapshot,
    ) -> None:
        action = ""
        action_error = ""
        try:
            if telemetry.landed_state == "ON_GROUND" and (
                telemetry.armed is True or self._state.arm_sent_monotonic is not None
            ):
                self._flight_actions.safety_disarm_on_ground()
                action = "已发送正常DISARM"
            elif telemetry.armed is True and telemetry.landed_state in {"IN_AIR", "TAKEOFF"}:
                self._flight_actions.land()
                action = "已发送LAND"
        except ValueError as exc:
            action_error = str(exc)
        self._state.active = False
        self._state.phase = phase
        self._state.phase_started_monotonic = self._clock()
        self._state.last_action = action or self._state.last_action
        self._state.last_reason = reason + (f"；{action}" if action else "")
        self._state.error = action_error or (reason if phase == "FAILED" else "")
