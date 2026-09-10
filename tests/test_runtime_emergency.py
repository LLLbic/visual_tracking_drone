from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.runtime import Runtime
from uav_preview.types import TelemetrySnapshot


class _GroundSender:
    def __init__(self) -> None:
        self.reason = ""

    def disable(self, reason: str) -> dict[str, object]:
        self.reason = reason
        return {"enabled": False}


class _Gate:
    def __init__(self) -> None:
        self.estop_latched = False

    def latch_estop(self) -> None:
        self.estop_latched = True

    def reset_estop(self) -> None:
        self.estop_latched = False


class _Telemetry:
    def __init__(self, snapshot: TelemetrySnapshot) -> None:
        self.value = snapshot

    def snapshot(self) -> TelemetrySnapshot:
        return self.value


class _Actions:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def safety_disarm_on_ground(self) -> dict[str, object]:
        self.calls.append("DISARM")
        return {"last_action": "DISARM-SAFETY-LATCH"}

    def brake(self) -> dict[str, object]:
        self.calls.append("BRAKE")
        return {"last_action": "BRAKE/PAUSE"}


def _runtime(telemetry: TelemetrySnapshot) -> Runtime:
    runtime = Runtime.__new__(Runtime)
    runtime.ground_offboard = _GroundSender()
    runtime.keyboard_control = _GroundSender()
    runtime.gate = _Gate()
    runtime.telemetry = _Telemetry(telemetry)
    runtime.flight_actions = _Actions()
    return runtime


class RuntimeEmergencyLatchTests(unittest.TestCase):
    def test_ground_latch_sends_normal_disarm_and_blocks_web_arm(self) -> None:
        runtime = _runtime(TelemetrySnapshot(armed=False, landed_state="ON_GROUND"))
        state = runtime.set_emergency_latch(True)
        self.assertTrue(runtime.gate.estop_latched)
        self.assertEqual(runtime.flight_actions.calls, ["DISARM"])
        self.assertEqual(state["flight_action"], "DISARM")

    def test_air_latch_sends_pause_not_disarm(self) -> None:
        runtime = _runtime(TelemetrySnapshot(armed=True, landed_state="IN_AIR"))
        state = runtime.set_emergency_latch(True)
        self.assertTrue(runtime.gate.estop_latched)
        self.assertEqual(runtime.flight_actions.calls, ["BRAKE"])
        self.assertEqual(state["flight_action"], "BRAKE/PAUSE")

    def test_release_only_clears_latch(self) -> None:
        runtime = _runtime(TelemetrySnapshot(armed=False, landed_state="ON_GROUND"))
        runtime.gate.latch_estop()
        state = runtime.set_emergency_latch(False)
        self.assertFalse(runtime.gate.estop_latched)
        self.assertEqual(runtime.flight_actions.calls, [])
        self.assertFalse(state["latched"])


if __name__ == "__main__":
    unittest.main()
