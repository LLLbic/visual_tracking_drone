from __future__ import annotations

from dataclasses import replace

from .config import AppConfig
from .controller import PreviewController
from .flight_actions import ExplicitFlightActionSender
from .keyboard_control import KeyboardOffboardSetpointSender
from .mavlink_router import LocalMavlinkRouter
from .offboard_test import GroundOffboardSetpointSender
from .safety import CONTROL_TRANSMISSION_COMPILED, SafetyGate
from .telemetry import PassiveMavlinkReceiver
from .video import VideoTrackingEngine


class Runtime:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.gate = SafetyGate()
        self.mavlink_router = LocalMavlinkRouter(config.mavlink_router)
        self.telemetry = PassiveMavlinkReceiver(config.telemetry)
        self.controller = PreviewController(config.control, self.gate)
        self.video = VideoTrackingEngine(config, self.telemetry, self.controller)
        self.ground_offboard = GroundOffboardSetpointSender(
            config.ground_offboard_test,
            self.telemetry.snapshot,
            lambda: self.gate.estop_latched,
            config.telemetry.stale_after_seconds,
        )
        self.keyboard_control = KeyboardOffboardSetpointSender(
            config.keyboard_control,
            config.ground_offboard_test,
            self.telemetry.snapshot,
            lambda: self.gate.estop_latched,
            config.telemetry.stale_after_seconds,
        )
        self.flight_actions = ExplicitFlightActionSender(
            config.ground_offboard_test,
            self.telemetry.snapshot,
            lambda: bool(
                self.ground_offboard.snapshot()["enabled"]
                or self.keyboard_control.snapshot()["enabled"]
            ),
            lambda: self.gate.estop_latched,
            lambda: bool(self.mavlink_router.snapshot()["ready"]),
            config.telemetry.stale_after_seconds,
            ground_takeoff_ready=lambda: bool(
                self.ground_offboard.snapshot()["ready_for_offboard_arm"]
            ),
            require_ground_takeoff_prestream=config.keyboard_control.allow_ground_takeoff,
        )

    def start(self) -> None:
        self.telemetry.start()
        self.mavlink_router.start()
        self.ground_offboard.start()
        self.keyboard_control.start()
        self.video.start()

    def stop(self) -> None:
        self.video.stop()
        self.keyboard_control.stop()
        self.ground_offboard.stop()
        self.mavlink_router.stop()
        self.telemetry.stop()

    def enable_ground_offboard_test(self) -> dict[str, object]:
        self.keyboard_control.disable("已切换到地面零速度测试")
        return self.ground_offboard.enable()

    def disable_ground_offboard_test(self, reason: str = "已由用户关闭") -> dict[str, object]:
        return self.ground_offboard.disable(reason)

    def enable_keyboard_control(self) -> dict[str, object]:
        # Start the keyboard sender at neutral before stopping the 5 Hz
        # pre-stream, so PX4 never sees an avoidable Offboard heartbeat gap.
        state = self.keyboard_control.enable()
        self.ground_offboard.disable("已无缝切换到键盘真实发送")
        return state

    def disable_keyboard_control(self, reason: str = "已由用户关闭") -> dict[str, object]:
        return self.keyboard_control.disable(reason)

    def update_keyboard_control(
        self, pitch: float, roll: float, throttle: float, yaw: float
    ) -> dict[str, object]:
        return self.keyboard_control.update_axes(pitch, roll, throttle, yaw)

    def set_armed(self, requested: bool) -> dict[str, object]:
        return self.flight_actions.set_armed(requested)

    def brake(self) -> dict[str, object]:
        return self.flight_actions.brake()

    def land(self) -> dict[str, object]:
        self.disable_keyboard_control("已请求真实降落")
        self.disable_ground_offboard_test("已请求真实降落")
        return self.flight_actions.land()

    def rtl(self) -> dict[str, object]:
        self.disable_keyboard_control("已请求真实返航")
        self.disable_ground_offboard_test("已请求真实返航")
        return self.flight_actions.rtl()

    def set_emergency_latch(self, enabled: bool) -> dict[str, object]:
        if not enabled:
            self.gate.reset_estop()
            return {
                "latched": False,
                "flight_action": "NONE",
                "message": "网页紧急制动锁存已解除；不会自动ARM",
            }

        # Latch first, so every later web ARM request is rejected even if the
        # immediate PX4 command cannot be delivered.
        self.disable_ground_offboard_test("网页紧急制动锁存触发")
        self.disable_keyboard_control("网页紧急制动锁存触发")
        self.gate.latch_estop()
        telemetry = self.telemetry.snapshot()
        flight_action = "LATCH_ONLY"
        action_state: dict[str, object] | None = None
        action_error = ""
        try:
            if telemetry.landed_state == "ON_GROUND":
                action_state = self.flight_actions.safety_disarm_on_ground()
                flight_action = "DISARM"
            elif telemetry.landed_state == "IN_AIR" and telemetry.armed is True:
                action_state = self.flight_actions.brake()
                flight_action = "BRAKE/PAUSE"
            elif telemetry.landed_state == "IN_AIR":
                action_error = "飞控报告IN_AIR但未解锁：已锁住网页ARM，未发送飞行动作"
            else:
                action_error = (
                    f"落地状态为{telemetry.landed_state}：已锁住网页ARM，未发送飞行动作"
                )
        except ValueError as exc:
            action_error = str(exc)

        return {
            "latched": True,
            "flight_action": flight_action,
            "action_state": action_state,
            "action_error": action_error,
            "message": (
                "网页紧急制动已锁存；后续网页ARM请求将被拒绝"
                + (f"；已发送{flight_action}" if flight_action not in {"NONE", "LATCH_ONLY"} else "")
                + (f"；{action_error}" if action_error else "")
            ),
        }

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
        flight_actions = self.flight_actions.snapshot()
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
        return {
            "build": {
                "passive_only": False,
                "vision_follow_control_transmission": False,
                "control_transmission_compiled": CONTROL_TRANSMISSION_COMPILED,
                "px4_parameter_writes": False,
                "rc_override": False,
                "automatic_offboard_start": False,
                "automatic_arm_or_takeoff": False,
                "guarded_computer_arm_and_shift_takeoff": bool(
                    self.config.keyboard_control.allow_ground_takeoff
                ),
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
                ),
                "vehicle_uplink": bool(
                    ground_offboard["enabled"] or keyboard_control["enabled"]
                ),
                "ground_offboard_zero_velocity_only": True,
                "keyboard_offboard_velocity_control": bool(
                    self.config.keyboard_control.available
                ),
            },
            "telemetry": telemetry.to_dict(),
            "mavlink_router": router,
            "ground_offboard_test": ground_offboard,
            "keyboard_control": keyboard_control,
            "flight_actions": flight_actions,
            "vision": vision.to_dict(),
            "models": self.video.model_status(),
            "command_preview": preview.to_dict(),
            "tuning": self.controller.tuning(),
        }
