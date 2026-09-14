from __future__ import annotations

from pathlib import Path
import sys
from threading import Event, Thread
from time import sleep
from time import monotonic
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.config import ControlConfig
from uav_preview.controller import PreviewController
from uav_preview.safety import CONTROL_TRANSMISSION_COMPILED, SafetyGate
from uav_preview.types import TargetSnapshot, TelemetrySnapshot


class SafetyAndControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.gate = SafetyGate()
        self.controller = PreviewController(ControlConfig(yaw_kd=0.0, distance_kd=0.0), self.gate)
        self.target = TargetSnapshot(
            locked=True,
            center_error_x=0.5,
            bbox_height_ratio=0.2,
            track_id=7,
        )

    @staticmethod
    def telemetry(mode: str) -> TelemetrySnapshot:
        return TelemetrySnapshot(
            connected=True,
            flight_mode=mode,
            last_packet_monotonic=monotonic(),
        )

    def test_passive_build_never_transmits(self) -> None:
        preview = self.controller.compute(self.target, self.telemetry("OFFBOARD"), 1.5)
        self.assertTrue(preview.offboard_observed)
        self.assertGreater(preview.raw_forward_m_s, 0.0)
        self.assertGreater(preview.raw_yaw_rate_deg_s, 0.0)
        self.assertGreater(preview.eligible_forward_m_s, 0.0)
        self.assertFalse(CONTROL_TRANSMISSION_COMPILED)
        self.assertFalse(preview.transmission_enabled)
        self.assertEqual(preview.transmitted_forward_m_s, 0.0)
        self.assertEqual(preview.transmitted_yaw_rate_deg_s, 0.0)

    def test_manual_mode_immediately_closes_gate(self) -> None:
        for mode in ("LAND", "AUTO_RTL", "POSCTL", "POSITION"):
            with self.subTest(mode=mode):
                preview = self.controller.compute(self.target, self.telemetry(mode), 1.5)
                self.assertFalse(preview.offboard_observed)
                self.assertEqual(preview.eligible_forward_m_s, 0.0)
                self.assertEqual(preview.eligible_yaw_rate_deg_s, 0.0)

    def test_local_estop_latches_preview_gate(self) -> None:
        self.gate.latch_estop()
        preview = self.controller.compute(self.target, self.telemetry("OFFBOARD"), 1.5)
        self.assertTrue(preview.local_estop_latched)
        self.assertEqual(preview.eligible_forward_m_s, 0.0)
        self.gate.reset_estop()
        preview = self.controller.compute(self.target, self.telemetry("OFFBOARD"), 1.5)
        self.assertFalse(preview.local_estop_latched)

    def test_gate_can_start_fail_closed_without_sending_an_action(self) -> None:
        gate = SafetyGate(initially_latched=True)
        self.assertTrue(gate.estop_latched)

    def test_guard_rejects_action_while_latched(self) -> None:
        gate = SafetyGate(initially_latched=True)
        called = False

        def action() -> None:
            nonlocal called
            called = True

        with self.assertRaisesRegex(ValueError, "锁存"):
            gate.run_if_unlatched(action)
        self.assertFalse(called)

    def test_latch_waits_for_inflight_guarded_action_then_blocks_new_one(self) -> None:
        gate = SafetyGate()
        action_entered = Event()
        release_action = Event()
        latch_completed = Event()

        def action() -> None:
            action_entered.set()
            self.assertTrue(release_action.wait(1.0))

        action_thread = Thread(target=lambda: gate.run_if_unlatched(action))
        action_thread.start()
        self.assertTrue(action_entered.wait(1.0))

        def engage() -> None:
            gate.latch_estop()
            latch_completed.set()

        latch_thread = Thread(target=engage)
        latch_thread.start()
        sleep(0.03)
        self.assertFalse(latch_completed.is_set())
        release_action.set()
        action_thread.join(1.0)
        latch_thread.join(1.0)
        self.assertTrue(latch_completed.is_set())
        with self.assertRaisesRegex(ValueError, "锁存"):
            gate.run_if_unlatched(lambda: None)

    def test_offboard_without_fresh_target_stays_closed(self) -> None:
        lost_target = TargetSnapshot(locked=False, track_id=7, source="lost", lost_frames=1)
        preview = self.controller.compute(lost_target, self.telemetry("OFFBOARD"), 1.5)
        self.assertFalse(preview.target_valid)
        self.assertEqual(preview.eligible_forward_m_s, 0.0)
        self.assertEqual(preview.eligible_yaw_rate_deg_s, 0.0)


if __name__ == "__main__":
    unittest.main()
