from __future__ import annotations

from pathlib import Path
import sys
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

    def test_offboard_without_fresh_target_stays_closed(self) -> None:
        lost_target = TargetSnapshot(locked=False, track_id=7, source="lost", lost_frames=1)
        preview = self.controller.compute(lost_target, self.telemetry("OFFBOARD"), 1.5)
        self.assertFalse(preview.target_valid)
        self.assertEqual(preview.eligible_forward_m_s, 0.0)
        self.assertEqual(preview.eligible_yaw_rate_deg_s, 0.0)


if __name__ == "__main__":
    unittest.main()
