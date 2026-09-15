from __future__ import annotations

from dataclasses import replace
from threading import RLock
from time import monotonic

from .config import AppConfig
from .controller import PreviewController
from .control_transport import GuardedControlSocket
from .flight_actions import ExplicitFlightActionSender
from .keyboard_control import KeyboardOffboardSetpointSender
from .local_takeoff import LocalOffboardTakeoffCoordinator
from .mavlink_router import LocalMavlinkRouter
from .navigation_health import navigation_block_reason
from .offboard_test import GroundOffboardSetpointSender
from .position_stream import PositionStreamRateRequester
from .safety import CONTROL_TRANSMISSION_COMPILED, SafetyGate
from .telemetry import PassiveMavlinkReceiver
from .takeoff import TakeoffCoordinator
from .video import VideoTrackingEngine


class Runtime:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        # Fail closed after every application restart.  The operator must
        # explicitly release the local latch before any computer-side sender
        # can be enabled; starting the web service never emits a flight action.
        self.gate = SafetyGate(initially_latched=True)
        def control_socket(family, kind):
            return GuardedControlSocket(self.gate.run_if_unlatched, family, kind)
        self._safety_transition_lock = RLock()
        self.mavlink_router = LocalMavlinkRouter(config.mavlink_router)
        self.telemetry = PassiveMavlinkReceiver(config.telemetry)
        def router_link_ready() -> bool:
            state = self.mavlink_router.snapshot()
            age = state["vehicle_age_seconds"]
            return bool(
                state["ready"]
                and age is not None
                and age <= config.telemetry.stale_after_seconds
            )

        self.position_stream = PositionStreamRateRequester(
            config.position_stream,
            self.telemetry.snapshot,
            router_link_ready,
            flow_message_type=config.telemetry.flow_message_type,
        )
        self.controller = PreviewController(config.control, self.gate)
        self.video = VideoTrackingEngine(config, self.telemetry, self.controller)
        self.ground_offboard = GroundOffboardSetpointSender(
            config.ground_offboard_test,
            self.telemetry.snapshot,
            lambda: self.gate.estop_latched,
            config.telemetry.stale_after_seconds,
            socket_factory=control_socket,
        )
        self.keyboard_control = KeyboardOffboardSetpointSender(
            config.keyboard_control,
            config.ground_offboard_test,
            self.telemetry.snapshot,
            lambda: self.gate.estop_latched,
            config.telemetry.stale_after_seconds,
            navigation_guard=lambda: navigation_block_reason(self.telemetry.snapshot()),
            socket_factory=control_socket,
        )
        self.flight_actions = ExplicitFlightActionSender(
            config.ground_offboard_test,
            self.telemetry.snapshot,
            lambda: bool(
                self.ground_offboard.snapshot()["enabled"]
                or self.keyboard_control.snapshot()["enabled"]
                or bool(
                    getattr(self, "local_takeoff", None)
                    and self.local_takeoff.snapshot()["active"]
                )
            ),
            lambda: self.gate.estop_latched,
            lambda: bool(self.mavlink_router.snapshot()["ready"]),
            config.telemetry.stale_after_seconds,
            ground_takeoff_ready=lambda: bool(
                self.ground_offboard.snapshot()["ready_for_offboard_arm"]
            ),
            require_ground_takeoff_prestream=config.keyboard_control.allow_ground_takeoff,
            physical_offboard_switch_pwm_min=(
                config.keyboard_control.physical_offboard_switch_pwm_min
            ),
            unlatched_action_guard=self.gate.run_if_unlatched,
        )
        self.takeoff = TakeoffCoordinator(
            config.takeoff,
            self.telemetry.snapshot,
            self.flight_actions,
            lambda: bool(
                self.ground_offboard.snapshot()["enabled"]
                or self.keyboard_control.snapshot()["enabled"]
                or bool(
                    getattr(self, "local_takeoff", None)
                    and self.local_takeoff.snapshot()["active"]
                )
            ),
            lambda: self.gate.estop_latched,
            lambda: bool(self.mavlink_router.snapshot()["ready"]),
            config.telemetry.stale_after_seconds,
            physical_offboard_switch_pwm_min=(
                config.keyboard_control.physical_offboard_switch_pwm_min
            ),
            navigation_guard=lambda: navigation_block_reason(self.telemetry.snapshot()),
        )
        self.local_takeoff = LocalOffboardTakeoffCoordinator(
            config.local_offboard_takeoff,
            config.ground_offboard_test,
            self.telemetry.snapshot,
            self.flight_actions,
            lambda: bool(
                self.ground_offboard.snapshot()["enabled"]
                or self.keyboard_control.snapshot()["enabled"]
                or self.takeoff.snapshot()["active"]
            ),
            lambda: self.gate.estop_latched,
            router_link_ready,
            config.telemetry.stale_after_seconds,
            physical_offboard_switch_pwm_min=(
                config.keyboard_control.physical_offboard_switch_pwm_min
            ),
            socket_factory=control_socket,
        )
        self._last_emergency_action: dict[str, object] = {
            "latched": True,
            "flight_action": "NONE",
            "action_state": None,
            "action_error": "",
            "message": "应用启动后默认锁存本地控制；未向飞控发送动作",
        }

    def start(self) -> None:
        self.telemetry.start()
        self.mavlink_router.start()
        self.position_stream.start()
        self.ground_offboard.start()
        self.keyboard_control.start()
        self.takeoff.start()
        self.local_takeoff.start()
        self.video.start()

    def stop(self) -> None:
        self.video.stop()
        self.local_takeoff.stop()
        self.takeoff.stop()
        self.keyboard_control.stop()
        self.ground_offboard.stop()
        self.position_stream.stop()
        self.mavlink_router.stop()
        self.telemetry.stop()

    def enable_ground_offboard_test(self) -> dict[str, object]:
        with self._safety_transition_lock:
            if self.takeoff.snapshot()["active"] or self.local_takeoff.snapshot()["active"]:
                raise ValueError("指定高度起飞正在运行，不能开启地面测试流")
            self.keyboard_control.disable("已切换到地面零速度测试")
            return self.ground_offboard.enable()

    def disable_ground_offboard_test(self, reason: str = "已由用户关闭") -> dict[str, object]:
        with self._safety_transition_lock:
            return self.ground_offboard.disable(reason)

    def enable_keyboard_control(self) -> dict[str, object]:
        with self._safety_transition_lock:
            if self.takeoff.snapshot()["active"] or self.local_takeoff.snapshot()["active"]:
                raise ValueError("指定高度起飞正在运行，尚不能开启键盘真实TX")
            # Start the keyboard sender at neutral before stopping the 5 Hz
            # pre-stream, so PX4 never sees an avoidable Offboard heartbeat gap.
            state = self.keyboard_control.enable()
            self.ground_offboard.disable("已无缝切换到键盘真实发送")
            return state

    def disable_keyboard_control(self, reason: str = "已由用户关闭") -> dict[str, object]:
        with self._safety_transition_lock:
            return self.keyboard_control.disable(reason)

    def update_keyboard_control(
        self, pitch: float, roll: float, throttle: float, yaw: float
    ) -> dict[str, object]:
        return self.keyboard_control.update_axes(pitch, roll, throttle, yaw)

    def set_armed(self, requested: bool) -> dict[str, object]:
        with self._safety_transition_lock:
            if self.takeoff.snapshot()["active"] or self.local_takeoff.snapshot()["active"]:
                raise ValueError("指定高度起飞正在运行，请使用起飞流程的取消按钮")
            return self.flight_actions.set_armed(requested)

    def begin_takeoff(self, target_height_m: float) -> dict[str, object]:
        with self._safety_transition_lock:
            reason = navigation_block_reason(self.telemetry.snapshot())
            if reason:
                raise ValueError(reason)
            return self.takeoff.begin(target_height_m)

    def abort_takeoff(self) -> dict[str, object]:
        return self.takeoff.abort()

    def begin_local_takeoff(self, target_height_m: float) -> dict[str, object]:
        with self._safety_transition_lock:
            if self.takeoff.snapshot()["active"]:
                raise ValueError("PX4原生起飞流程正在运行")
            return self.local_takeoff.begin(target_height_m)

    def abort_local_takeoff(self) -> dict[str, object]:
        return self.local_takeoff.abort()

    def report_handoff_input(self, **values) -> dict[str, object]:
        with self._safety_transition_lock:
            return self.local_takeoff.report_keyboard(**values)

    def authorize_handoff(self, run_id: str, client_id: str) -> dict[str, object]:
        with self._safety_transition_lock:
            if self.takeoff.snapshot()["active"] or self.keyboard_control.snapshot()["enabled"] or self.ground_offboard.snapshot()["enabled"]:
                raise ValueError("存在其他发送器/起飞任务，拒绝交接")
            return self.local_takeoff.authorize_keyboard(run_id,client_id)

    def revoke_handoff(self, run_id: str, client_id: str, token: str) -> dict[str, object]:
        with self._safety_transition_lock:
            return self.local_takeoff.revoke_keyboard(run_id,client_id,token)

    def brake(self) -> dict[str, object]:
        with self._safety_transition_lock:
            return self.flight_actions.brake()

    def land(self) -> dict[str, object]:
        with self._safety_transition_lock:
            self.disable_keyboard_control("已请求真实降落")
            self.disable_ground_offboard_test("已请求真实降落")
            self.takeoff.cancel_without_action("用户已直接请求LAND，起飞状态机停止")
            self.local_takeoff.cancel_without_action("用户已直接请求LAND，本地起飞状态机停止")
            state = self.flight_actions.land()
            return state

    def rtl(self) -> dict[str, object]:
        with self._safety_transition_lock:
            self.disable_keyboard_control("已请求真实返航")
            self.disable_ground_offboard_test("已请求真实返航")
            self.takeoff.cancel_without_action("用户已直接请求RTL，起飞状态机停止")
            self.local_takeoff.cancel_without_action("用户已直接请求RTL，本地起飞状态机停止")
            state = self.flight_actions.rtl()
            return state

    def set_emergency_latch(self, enabled: bool) -> dict[str, object]:
        with self._safety_transition_lock:
            if not enabled:
                self._validate_emergency_release()
                self.gate.reset_estop()
                self._last_emergency_action = {
                    "latched": False,
                    "flight_action": "NONE",
                    "action_state": None,
                    "action_error": "",
                    "message": (
                        "网页安全处置锁存已解除；不会自动ARM；"
                        "实体Kill Switch仍须由飞手单独解除"
                    ),
                }
                return dict(self._last_emergency_action)

            # This must be the first mutation.  All unsafe one-shot commands
            # take the same gate lock for their final UDP send, closing the
            # check-then-send race with an emergency transition.
            self.gate.latch_estop()
            self._stop_all_local_control("网页安全处置锁存触发")
            return self._apply_emergency_flight_action()

    def latch_local_control_only(self, reason: str = "本地安全锁存触发") -> dict[str, object]:
        """Latch and stop every local sender without issuing a flight command."""

        with self._safety_transition_lock:
            self.gate.latch_estop()
            self._stop_all_local_control(reason)
            self._last_emergency_action = {
                "latched": True,
                "flight_action": "LATCH_ONLY",
                "action_state": None,
                "action_error": "",
                "message": "本地控制已锁存；未向飞控发送动作",
            }
            return dict(self._last_emergency_action)

    def _stop_all_local_control(self, reason: str) -> None:
        takeoff = getattr(self, "takeoff", None)
        if takeoff is not None:
            takeoff.cancel_without_action(f"{reason}，起飞状态机停止")
        local_takeoff = getattr(self, "local_takeoff", None)
        if local_takeoff is not None:
            local_takeoff.cancel_without_action(f"{reason}，本地起飞状态机停止")
        self.disable_ground_offboard_test(reason)
        self.disable_keyboard_control(reason)

    def _validate_emergency_release(self) -> None:
        if not self.gate.estop_latched:
            raise ValueError("网页安全处置当前未锁存，无需重复解除")

        telemetry = self.telemetry.snapshot()
        now = monotonic()
        age = telemetry.age_seconds(now)
        stale_after = self.config.telemetry.stale_after_seconds
        if not telemetry.connected or age is None or age > stale_after:
            raise ValueError("遥测离线或已过期，拒绝解除安全处置锁存")
        required_fresh = (
            (telemetry.last_heartbeat_monotonic, "解锁状态"),
            (telemetry.last_extended_state_monotonic, "落地状态"),
            (telemetry.last_rc_channels_monotonic, "实体RC通道"),
        )
        for timestamp, label in required_fresh:
            if timestamp is None or max(0.0, now - timestamp) > stale_after:
                raise ValueError(f"{label}遥测缺失或已过期，拒绝解除安全处置锁存")
        if telemetry.armed is not False:
            raise ValueError("只有飞控明确报告DISARMED时才能解除安全处置锁存")
        if telemetry.landed_state != "ON_GROUND":
            raise ValueError(
                f"只有飞控明确报告ON_GROUND时才能解除安全处置锁存；"
                f"当前为{telemetry.landed_state}"
            )
        if telemetry.rc_channel_6_pwm is None or telemetry.rc_channel_6_pwm < 1800:
            raise ValueError("实体Kill Switch（CH6）必须保持触发高位，才能解除网页锁存")
        offboard_min = self.config.keyboard_control.physical_offboard_switch_pwm_min
        if telemetry.rc_channel_8_pwm is None or telemetry.rc_channel_8_pwm >= offboard_min:
            raise ValueError("实体CH8必须处于非Offboard位置，才能解除网页锁存")
        if self.ground_offboard.snapshot()["enabled"]:
            raise ValueError("地面Offboard发送器仍在运行，拒绝解除网页锁存")
        if self.keyboard_control.snapshot()["enabled"]:
            raise ValueError("键盘真实TX仍在运行，拒绝解除网页锁存")
        takeoff = getattr(self, "takeoff", None)
        if takeoff is not None and takeoff.snapshot()["active"]:
            raise ValueError("起飞状态机仍在运行，拒绝解除网页锁存")
        local_takeoff = getattr(self, "local_takeoff", None)
        if local_takeoff is not None and local_takeoff.snapshot()["active"]:
            raise ValueError("本地起飞状态机仍在运行，拒绝解除网页锁存")

    def _apply_emergency_flight_action(self) -> dict[str, object]:
        telemetry = self.telemetry.snapshot()
        flight_action = "LATCH_ONLY"
        action_state: dict[str, object] | None = None
        action_error = ""
        try:
            if telemetry.landed_state == "ON_GROUND":
                action_state = self.flight_actions.safety_disarm_on_ground()
                flight_action = "DISARM"
            elif telemetry.landed_state in {"IN_AIR", "TAKEOFF"} and telemetry.armed is True:
                # Never command an in-air motor stop.  LAND is the predictable
                # controlled-descent action; the physical RC remains the
                # primary escape path if the flight controller rejects it.
                action_state = self.flight_actions.land()
                flight_action = "LAND"
            elif telemetry.landed_state in {"IN_AIR", "TAKEOFF"}:
                action_error = (
                    f"飞控报告{telemetry.landed_state}但未解锁："
                    "已锁住网页ARM，未发送飞行动作"
                )
            else:
                action_error = (
                    f"落地状态为{telemetry.landed_state}：已锁住网页ARM，未发送飞行动作"
                )
        except ValueError as exc:
            action_error = str(exc)

        self._last_emergency_action = {
            "latched": True,
            "flight_action": flight_action,
            "action_state": action_state,
            "action_error": action_error,
            "message": (
                "网页安全处置已锁存；后续网页ARM请求将被拒绝"
                + (f"；已发送{flight_action}" if flight_action not in {"NONE", "LATCH_ONLY"} else "")
                + (f"；{action_error}" if action_error else "")
            ),
        }
        return dict(self._last_emergency_action)

    def state(self) -> dict[str, object]:
        vision, preview = self.video.snapshot()
        telemetry = self.telemetry.snapshot()
        age = telemetry.age_seconds()
        telemetry_fresh = bool(
            telemetry.connected
            and age is not None
            and age <= self.config.telemetry.stale_after_seconds
        )
        target_valid = bool(
            vision.target.locked
            and vision.target.center_error_x is not None
            and vision.target.bbox_height_ratio is not None
        )
        decision = self.gate.evaluate(telemetry.flight_mode, telemetry_fresh, target_valid)
        ground_offboard = self.ground_offboard.snapshot()
        keyboard_control = self.keyboard_control.snapshot()
        router = self.mavlink_router.snapshot()
        position_stream = self.position_stream.snapshot()
        flight_actions = self.flight_actions.snapshot()
        takeoff = self.takeoff.snapshot()
        local_takeoff = self.local_takeoff.snapshot()
        emergency_action = dict(
            getattr(
                self,
                "_last_emergency_action",
                {
                    "latched": self.gate.estop_latched,
                    "flight_action": "NONE",
                    "action_error": "",
                },
            )
        )
        emergency_action["release_eligible"] = False
        emergency_action["release_block_reason"] = ""
        if self.gate.estop_latched:
            try:
                self._validate_emergency_release()
                emergency_action["release_eligible"] = True
            except ValueError as exc:
                emergency_action["release_block_reason"] = str(exc)
        # Re-evaluate the gate from the freshest heartbeat even if the video thread is
        # stalled. PID suggestions remain from the latest frame; actual TX remains zero.
        preview = replace(
            preview,
            eligible_forward_m_s=preview.raw_forward_m_s if decision.eligible else 0.0,
            eligible_yaw_rate_deg_s=preview.raw_yaw_rate_deg_s if decision.eligible else 0.0,
            transmitted_forward_m_s=0.0,
            transmitted_yaw_rate_deg_s=0.0,
            offboard_observed=decision.offboard_observed,
            telemetry_fresh=decision.telemetry_fresh,
            target_valid=target_valid,
            local_estop_latched=decision.local_estop_latched,
            transmission_enabled=False,
            gate_reason=decision.reason,
        )
        navigation_now = monotonic()
        local_navigation = self.local_takeoff.navigation_status(telemetry, navigation_now)
        takeoff_reasons = local_navigation["block_reasons"]
        return {
            "navigation_safety": {
                "block_reason": next(iter(takeoff_reasons), ""),
                "block_reasons": takeoff_reasons,
            },
            "build": {
                "passive_only": False,
                "vision_follow_control_transmission": False,
                "control_transmission_compiled": CONTROL_TRANSMISSION_COMPILED,
                "px4_parameter_writes": False,
                "rc_override": False,
                "automatic_offboard_start": False,
                "automatic_arm_or_takeoff": False,
                "user_confirmed_native_takeoff": bool(takeoff["available"]),
                "user_confirmed_local_offboard_takeoff": bool(local_takeoff["available"]),
                "takeoff_height_limits_m": [
                    self.config.takeoff.min_height_m,
                    self.config.takeoff.max_height_m,
                ],
                "guarded_computer_arm_and_shift_takeoff": False,
                "ground_keyboard_takeoff_forbidden": True,
                "air_emergency_action": "LAND",
                "explicit_arm_disarm_command": bool(flight_actions["available"]),
                "explicit_brake_command": bool(flight_actions["available"]),
                "physical_rc_priority": bool(router["rc_priority_enforced"]),
                "qgc_manual_control_blocked": bool(router["rc_priority_enforced"]),
                "qgc_high_level_flight_commands_allowed": True,
                "telemetry_input": "local_router_127.0.0.1:14550",
                "application_udp_output": bool(
                    self.config.telemetry.forward_qgc
                    or ground_offboard["enabled"]
                    or keyboard_control["enabled"]
                    or takeoff["active"]
                    or local_takeoff["active"]
                ),
                "vehicle_uplink": bool(
                    ground_offboard["enabled"]
                    or keyboard_control["enabled"]
                    or takeoff["active"]
                    or local_takeoff["active"]
                ),
                "ground_offboard_zero_velocity_only": True,
                "keyboard_offboard_velocity_control": bool(
                    self.config.keyboard_control.available
                ),
            },
            "telemetry": telemetry.to_dict(),
            "mavlink_router": router,
            "position_stream": position_stream,
            "ground_offboard_test": ground_offboard,
            "keyboard_control": keyboard_control,
            "flight_actions": flight_actions,
            "takeoff": takeoff,
            "local_takeoff": local_takeoff,
            "emergency_action": emergency_action,
            "vision": vision.to_dict(),
            "models": self.video.model_status(),
            "command_preview": preview.to_dict(),
            "tuning": self.controller.tuning(),
        }
