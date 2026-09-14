from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.config import TakeoffConfig
from uav_preview.takeoff import TakeoffCoordinator
from uav_preview.types import TelemetrySnapshot


class _Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class _Actions:
    def __init__(self) -> None:
        self.calls: list[tuple[str, float | None]] = []

    def set_armed(self, requested: bool) -> dict[str, object]:
        self.calls.append(("ARM" if requested else "DISARM", None))
        return {}

    def takeoff(self, height: float, _ground_amsl: float) -> dict[str, object]:
        self.calls.append(("TAKEOFF", height))
        return {}

    def safety_disarm_on_ground(self) -> dict[str, object]:
        self.calls.append(("DISARM", None))
        return {}

    def land(self) -> dict[str, object]:
        self.calls.append(("LAND", None))
        return {}


class TakeoffCoordinatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = _Clock()
        self.telemetry = TelemetrySnapshot(
            connected=True,
            system_id=1,
            component_id=1,
            flight_mode="POSCTL",
            armed=False,
            landed_state="ON_GROUND",
            roll_deg=0.0,
            pitch_deg=0.0,
            vx_m_s=0.0,
            vy_m_s=0.0,
            vz_m_s=0.0,
            rc_channel_6_pwm=1000,
            rc_channel_8_pwm=1000,
            local_x_m=2.0,
            local_y_m=3.0,
            local_z_m=-0.1,
            relative_altitude_m=0.0,
            latitude_deg=30.25,
            longitude_deg=119.75,
            global_altitude_amsl_m=102.4,
            last_packet_monotonic=self.clock(),
            last_local_position_monotonic=self.clock(),
            last_global_position_monotonic=self.clock(),
            last_heartbeat_monotonic=self.clock(),
            last_attitude_monotonic=self.clock(),
            last_extended_state_monotonic=self.clock(),
            last_rc_channels_monotonic=self.clock(),
        )
        self.actions = _Actions()
        self.estop_latched = False
        self.coordinator = TakeoffCoordinator(
            TakeoffConfig(
                available=True,
                preflight_stable_seconds=0.1,
                arm_ack_timeout_seconds=1.0,
                arm_to_takeoff_delay_seconds=0.1,
                takeoff_ack_timeout_seconds=1.0,
                liftoff_timeout_seconds=1.0,
                climb_timeout_seconds=3.0,
                hover_stable_seconds=0.1,
            ),
            lambda: self.telemetry,
            self.actions,  # type: ignore[arg-type]
            lambda: self.estop_latched,
            lambda: False,
            lambda: True,
            1.5,
            clock=self.clock,
        )

    def _freshen(self) -> None:
        self.telemetry.last_packet_monotonic = self.clock()
        self.telemetry.last_local_position_monotonic = self.clock()
        self.telemetry.last_global_position_monotonic = self.clock()
        self.telemetry.last_heartbeat_monotonic = self.clock()
        self.telemetry.last_attitude_monotonic = self.clock()
        self.telemetry.last_extended_state_monotonic = self.clock()
        self.telemetry.last_rc_channels_monotonic = self.clock()

    def _reach_arming(self) -> None:
        self.coordinator.begin(1.5)
        self.clock.advance(0.11)
        self._freshen()
        self.coordinator.poll_once()
        self.assertEqual(self.coordinator.snapshot()["phase"], "ARMING")

    def test_complete_arm_takeoff_and_hover_sequence(self) -> None:
        self._reach_arming()
        self.telemetry.armed = True
        self.telemetry.last_command_ack_command = 400
        self.telemetry.last_command_ack_result = 0
        self.telemetry.last_command_ack_monotonic = self.clock()
        self.clock.advance(0.11)
        self._freshen()
        self.coordinator.poll_once()
        self.assertEqual(self.coordinator.snapshot()["phase"], "TAKEOFF_REQUESTED")

        self.telemetry.last_command_ack_command = 22
        self.telemetry.last_command_ack_result = 0
        self.telemetry.last_command_ack_monotonic = self.clock()
        self.telemetry.flight_mode = "TAKEOFF"
        self.telemetry.landed_state = "TAKEOFF"
        self.telemetry.local_z_m = -0.3
        self.clock.advance(0.05)
        self._freshen()
        self.coordinator.poll_once()
        self.assertEqual(self.coordinator.snapshot()["phase"], "CLIMBING")

        self.telemetry.landed_state = "IN_AIR"
        self.telemetry.local_z_m = -1.6
        self.telemetry.vz_m_s = 0.0
        self.clock.advance(0.05)
        self._freshen()
        self.coordinator.poll_once()
        self.assertEqual(self.coordinator.snapshot()["phase"], "HOVER_VERIFY")
        self.clock.advance(0.11)
        self._freshen()
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertEqual(state["phase"], "COMPLETE")
        self.assertFalse(state["active"])
        self.assertEqual(self.actions.calls, [("ARM", None), ("TAKEOFF", 1.5)])

    def test_arm_rejection_sends_normal_ground_disarm(self) -> None:
        self._reach_arming()
        self.telemetry.last_command_ack_command = 400
        self.telemetry.last_command_ack_result = 2
        self.telemetry.last_command_ack_monotonic = self.clock()
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertEqual(state["phase"], "FAILED")
        self.assertEqual(self.actions.calls[-1], ("DISARM", None))

    def test_physical_ch8_activity_yields_without_counter_command(self) -> None:
        self._reach_arming()
        self.telemetry.rc_channel_8_pwm = 2000
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertEqual(state["phase"], "PILOT_TAKEOVER")
        self.assertEqual(self.actions.calls, [("ARM", None)])

    def test_rejects_missing_position_and_out_of_range_height(self) -> None:
        self.telemetry.local_z_m = None
        with self.assertRaisesRegex(ValueError, "位置"):
            self.coordinator.begin(1.5)
        self.telemetry.local_z_m = -0.1
        with self.assertRaisesRegex(ValueError, "1.0至3.0"):
            self.coordinator.begin(0.9)
        with self.assertRaisesRegex(ValueError, "1.0至3.0"):
            self.coordinator.begin(3.1)

    def test_latch_stops_state_machine_without_duplicate_flight_action(self) -> None:
        self.coordinator.begin(1.5)
        self.estop_latched = True
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertFalse(state["active"])
        self.assertEqual(state["phase"], "FAILED")
        self.assertEqual(self.actions.calls, [])


if __name__ == "__main__":
    unittest.main()
