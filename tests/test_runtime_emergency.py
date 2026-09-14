from __future__ import annotations

from pathlib import Path
import sys
from threading import RLock
from time import monotonic
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.runtime import Runtime
from uav_preview.safety import SafetyGate
from uav_preview.types import TelemetrySnapshot


class _GroundSender:
    def __init__(self, gate: SafetyGate) -> None:
        self.reason = ""
        self.enabled = False
        self.gate = gate
        self.observed_latched_on_disable: list[bool] = []

    def disable(self, reason: str) -> dict[str, object]:
        self.reason = reason
        self.observed_latched_on_disable.append(self.gate.estop_latched)
        self.enabled = False
        return {"enabled": False}

    def snapshot(self) -> dict[str, object]:
        return {"enabled": self.enabled}


class _Coordinator:
    def __init__(self, gate: SafetyGate) -> None:
        self.active = False
        self.gate = gate
        self.observed_latched_on_cancel: list[bool] = []

    def cancel_without_action(self, _reason: str) -> dict[str, object]:
        self.observed_latched_on_cancel.append(self.gate.estop_latched)
        self.active = False
        return {"active": False}

    def snapshot(self) -> dict[str, object]:
        return {"active": self.active}


class _Telemetry:
    def __init__(self, snapshot: TelemetrySnapshot) -> None:
        self.value = snapshot

    def snapshot(self) -> TelemetrySnapshot:
        return self.value


class _Actions:
    def __init__(self, gate: SafetyGate) -> None:
        self.calls: list[str] = []
        self.gate = gate
        self.observed_latched: list[bool] = []

    def safety_disarm_on_ground(self) -> dict[str, object]:
        self.observed_latched.append(self.gate.estop_latched)
        self.calls.append("DISARM")
        return {"last_action": "DISARM-SAFETY-LATCH"}

    def land(self) -> dict[str, object]:
        self.observed_latched.append(self.gate.estop_latched)
        self.calls.append("LAND")
        return {"last_action": "LAND"}


def _runtime(telemetry: TelemetrySnapshot) -> Runtime:
    runtime = Runtime.__new__(Runtime)
    runtime.gate = SafetyGate()
    runtime._safety_transition_lock = RLock()
    runtime.ground_offboard = _GroundSender(runtime.gate)
    runtime.keyboard_control = _GroundSender(runtime.gate)
    runtime.takeoff = _Coordinator(runtime.gate)
    runtime.local_takeoff = _Coordinator(runtime.gate)
    runtime.telemetry = _Telemetry(telemetry)
    runtime.flight_actions = _Actions(runtime.gate)
    runtime.config = SimpleNamespace(
        telemetry=SimpleNamespace(stale_after_seconds=1.5),
        keyboard_control=SimpleNamespace(physical_offboard_switch_pwm_min=1800),
    )
    return runtime


def _fresh_telemetry(**overrides: object) -> TelemetrySnapshot:
    now = monotonic()
    values: dict[str, object] = {
        "connected": True,
        "armed": False,
        "landed_state": "ON_GROUND",
        "rc_channel_6_pwm": 1999,
        "rc_channel_8_pwm": 1000,
        "last_packet_monotonic": now,
        "last_heartbeat_monotonic": now,
        "last_extended_state_monotonic": now,
        "last_rc_channels_monotonic": now,
    }
    values.update(overrides)
    return TelemetrySnapshot(**values)


class RuntimeEmergencyLatchTests(unittest.TestCase):
    def test_ground_latch_sends_normal_disarm_and_blocks_web_arm(self) -> None:
        runtime = _runtime(_fresh_telemetry())
        state = runtime.set_emergency_latch(True)
        self.assertTrue(runtime.gate.estop_latched)
        self.assertEqual(runtime.flight_actions.calls, ["DISARM"])
        self.assertEqual(state["flight_action"], "DISARM")
        self.assertEqual(runtime.flight_actions.observed_latched, [True])
        self.assertTrue(all(runtime.ground_offboard.observed_latched_on_disable))
        self.assertTrue(all(runtime.keyboard_control.observed_latched_on_disable))
        self.assertTrue(all(runtime.takeoff.observed_latched_on_cancel))
        self.assertTrue(all(runtime.local_takeoff.observed_latched_on_cancel))

    def test_air_latch_sends_land_not_disarm_or_pause(self) -> None:
        runtime = _runtime(_fresh_telemetry(armed=True, landed_state="IN_AIR"))
        state = runtime.set_emergency_latch(True)
        self.assertTrue(runtime.gate.estop_latched)
        self.assertEqual(runtime.flight_actions.calls, ["LAND"])
        self.assertEqual(state["flight_action"], "LAND")

    def test_takeoff_phase_latch_also_sends_controlled_land(self) -> None:
        runtime = _runtime(_fresh_telemetry(armed=True, landed_state="TAKEOFF"))
        state = runtime.set_emergency_latch(True)
        self.assertEqual(runtime.flight_actions.calls, ["LAND"])
        self.assertEqual(state["flight_action"], "LAND")

    def test_release_requires_fresh_ground_disarmed_physical_kill_and_manual_mode(self) -> None:
        runtime = _runtime(_fresh_telemetry())
        runtime.gate.latch_estop()
        state = runtime.set_emergency_latch(False)
        self.assertFalse(runtime.gate.estop_latched)
        self.assertEqual(runtime.flight_actions.calls, [])
        self.assertFalse(state["latched"])

    def test_release_fails_closed_for_each_missing_safety_condition(self) -> None:
        unsafe_cases = (
            ("遥测", {"connected": False}),
            ("落地状态遥测", {"last_extended_state_monotonic": None}),
            ("DISARMED", {"armed": True}),
            ("ON_GROUND", {"landed_state": "IN_AIR"}),
            ("Kill Switch", {"rc_channel_6_pwm": 1000}),
            ("CH8", {"rc_channel_8_pwm": 2000}),
        )
        for expected, overrides in unsafe_cases:
            with self.subTest(expected=expected):
                runtime = _runtime(_fresh_telemetry(**overrides))
                runtime.gate.latch_estop()
                with self.assertRaisesRegex(ValueError, expected):
                    runtime.set_emergency_latch(False)
                self.assertTrue(runtime.gate.estop_latched)

    def test_release_rejects_active_sender(self) -> None:
        runtime = _runtime(_fresh_telemetry())
        runtime.gate.latch_estop()
        runtime.keyboard_control.enabled = True
        with self.assertRaisesRegex(ValueError, "键盘真实TX"):
            runtime.set_emergency_latch(False)
        self.assertTrue(runtime.gate.estop_latched)

    def test_legacy_local_latch_never_sends_a_flight_command(self) -> None:
        runtime = _runtime(_fresh_telemetry(armed=True, landed_state="IN_AIR"))
        state = runtime.latch_local_control_only()
        self.assertTrue(runtime.gate.estop_latched)
        self.assertEqual(runtime.flight_actions.calls, [])
        self.assertEqual(state["flight_action"], "LATCH_ONLY")


if __name__ == "__main__":
    unittest.main()
