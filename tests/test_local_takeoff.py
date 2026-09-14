from __future__ import annotations

import math
import unittest

from pymavlink.dialects.v20 import common as mavlink2

from uav_preview.config import GroundOffboardTestConfig, LocalOffboardTakeoffConfig
from uav_preview.local_takeoff import (
    LOCAL_POSITION_TYPE_MASK,
    LOCAL_TRAJECTORY_TYPE_MASK,
    LocalOffboardTakeoffCoordinator,
)
from uav_preview.types import TelemetrySnapshot
from test_smooth_handoff import healthy, fresh


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeSocket:
    def __init__(self) -> None:
        self.sent: list[tuple[bytes, tuple[str, int]]] = []

    def sendto(self, packet: bytes, endpoint: tuple[str, int]) -> None:
        self.sent.append((packet, endpoint))

    def close(self) -> None:
        pass


class FakeActions:
    def __init__(self) -> None:
        self.arm_calls = 0
        self.disarm_calls = 0
        self.land_calls = 0

    def arm_for_local_offboard_takeoff(self, prestream_ready: bool) -> dict[str, object]:
        if not prestream_ready:
            raise ValueError("not ready")
        self.arm_calls += 1
        return {}

    def safety_disarm_on_ground(self) -> dict[str, object]:
        self.disarm_calls += 1
        return {}

    def land(self) -> dict[str, object]:
        self.land_calls += 1
        return {}


class LocalOffboardTakeoffTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        # Complete synthetic safety evidence, never obtained from live hardware.
        self.telemetry = healthy(self.clock.now)
        self.telemetry.system_id=self.telemetry.component_id=1
        self.telemetry.flight_mode='POSCTL'
        self.telemetry.armed=False
        self.telemetry.landed_state='ON_GROUND'
        self.telemetry.yaw_deg=30.
        self.telemetry.local_z_m=.25
        self.telemetry.rc_channel_8_pwm=1000
        self._touch()
        self.actions = FakeActions()
        self.sock = FakeSocket()
        self.estop_latched = False
        self.coordinator = LocalOffboardTakeoffCoordinator(
            LocalOffboardTakeoffConfig(
                available=True,
                # Existing handoff regressions opt in; takeoff-only fixtures opt out.
                keyboard_handoff_enabled=getattr(self, "keyboard_handoff_enabled", True),
                navigation_profile=getattr(self, "navigation_profile", "detailed"),
                prestream_seconds=1.0,
                offboard_wait_timeout_seconds=5.0,
                arm_ack_timeout_seconds=1.0,
                liftoff_timeout_seconds=3.0,
                climb_timeout_seconds=10.0,
                climb_rate_m_s=0.5,
                hover_stable_seconds=1.0,
            ),
            GroundOffboardTestConfig(available=True),
            lambda: self.telemetry,
            self.actions,  # type: ignore[arg-type]
            lambda: False,
            lambda: self.estop_latched,
            lambda: True,
            stale_after_seconds=1.5,
            socket_factory=lambda *_: self.sock,
            clock=self.clock,
        )

    def _touch(self) -> None:
        now = self.clock.now
        fresh(self.telemetry,now)
        self.telemetry.last_packet_monotonic = now
        self.telemetry.last_heartbeat_monotonic = now
        self.telemetry.last_attitude_monotonic = now
        self.telemetry.last_extended_state_monotonic = now
        self.telemetry.last_rc_channels_monotonic = now
        self.telemetry.last_local_position_monotonic = now
        self.telemetry.last_reset_evidence_monotonic = now

    def _advance(self, seconds: float) -> None:
        self.clock.advance(seconds)
        self._touch()

    def _elapse(self, seconds: float) -> None:
        # Simulate every control tick. A large clock jump is a stall, not flight.
        remaining=seconds
        while remaining>1e-8:
            step=min(.1,remaining)
            self._advance(step)
            self.coordinator.poll_once()
            remaining-=step

    @staticmethod
    def _decode(packet: bytes):
        parser = mavlink2.MAVLink(None)
        message = None
        for byte in packet:
            parsed = parser.parse_char(bytes((byte,)))
            if parsed is not None:
                message = parsed
        return message

    def _reach_arming(self) -> None:
        self.coordinator.begin(1.5)
        self.coordinator.poll_once()
        self._elapse(1.1)
        self.coordinator.poll_once()
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_8_pwm = 2000
        self._touch()
        self.coordinator.poll_once()

    def test_uses_current_local_pose_without_global_position(self) -> None:
        state = self.coordinator.begin(1.5)
        self.assertEqual(state["phase"], "PRESTREAM")
        self.assertEqual(state["start_local_z_m"], 0.25)
        self.assertIsNone(self.telemetry.latitude_deg)

        self.coordinator.poll_once()
        packet, endpoint = self.sock.sent[-1]
        message = self._decode(packet)
        self.assertEqual(endpoint, ("127.0.0.1", 14560))
        self.assertEqual(message.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertEqual(message.coordinate_frame, mavlink2.MAV_FRAME_LOCAL_NED)
        self.assertEqual(message.type_mask, LOCAL_TRAJECTORY_TYPE_MASK)
        self.assertAlmostEqual(message.x, 4.0)
        self.assertAlmostEqual(message.y, -2.0)
        self.assertAlmostEqual(message.z, 0.25)
        self.assertAlmostEqual(message.yaw, math.radians(30.0), places=5)
        self.assertEqual(self.actions.arm_calls, 0)

    def test_waits_for_physical_offboard_then_arms_and_ramps(self) -> None:
        self._reach_arming()
        self.assertEqual(self.actions.arm_calls, 1)
        self.assertEqual(self.coordinator.snapshot()["phase"], "ARMING")

        self.telemetry.armed = True
        self.telemetry.last_command_ack_command = 400
        self.telemetry.last_command_ack_result = 0
        self.telemetry.last_command_ack_monotonic = self.clock.now
        self._touch()
        self.coordinator.poll_once()
        self.assertEqual(self.coordinator.snapshot()["phase"], "CLIMBING")

        self._elapse(1.0)
        self.telemetry.landed_state = "TAKEOFF"
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertAlmostEqual(state["commanded_height_m"], 0.5)
        message = self._decode(self.sock.sent[-1][0])
        self.assertAlmostEqual(message.z, -0.25, places=5)

    def test_reaching_height_keeps_hold_stream_active(self) -> None:
        self._reach_arming()
        self.telemetry.armed = True
        self.telemetry.last_command_ack_command = 400
        self.telemetry.last_command_ack_result = 0
        self.telemetry.last_command_ack_monotonic = self.clock.now
        self._touch()
        self.coordinator.poll_once()

        self.telemetry.landed_state = "TAKEOFF"
        self._elapse(3.1)
        self.telemetry.local_z_m = -1.25
        self.telemetry.landed_state = "IN_AIR"
        self.telemetry.vz_m_s = 0.0
        self.coordinator.poll_once()
        self.assertEqual(self.coordinator.snapshot()["phase"], "HOVER_VERIFY")

        self._elapse(1.1)
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertEqual(state["phase"], "HOLDING")
        self.assertTrue(state["active"])
        packets = state["packets_sent"]
        self._advance(0.1)
        self.coordinator.poll_once()
        self.assertGreater(self.coordinator.snapshot()["packets_sent"], packets)

    def test_switching_back_to_position_immediately_yields_to_pilot(self) -> None:
        self._reach_arming()
        self.telemetry.flight_mode = "POSCTL"
        self.telemetry.rc_channel_8_pwm = 1000
        self._touch()
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertFalse(state["active"])
        self.assertEqual(state["phase"], "PILOT_TAKEOVER")
        self.assertEqual(self.actions.land_calls, 0)
        self.assertEqual(self.actions.disarm_calls, 0)

    def test_early_channel_8_change_fails_without_arming(self) -> None:
        self.coordinator.begin(1.5)
        self.telemetry.rc_channel_8_pwm = 2000
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertEqual(state["phase"], "FAILED")
        self.assertFalse(state["active"])
        self.assertEqual(self.actions.arm_calls, 0)

    def test_stale_local_position_stops_without_sending_action(self) -> None:
        self.coordinator.begin(1.5)
        self.clock.advance(2.0)
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertEqual(state["phase"], "FAILED")
        self.assertFalse(state["active"])
        self.assertEqual(self.actions.land_calls, 0)
        self.assertEqual(self.actions.disarm_calls, 0)

    def test_latch_stops_state_machine_without_duplicate_flight_action(self) -> None:
        self.coordinator.begin(1.5)
        self.estop_latched = True
        self.coordinator.poll_once()
        state = self.coordinator.snapshot()
        self.assertFalse(state["active"])
        self.assertEqual(state["phase"], "FAILED")
        self.assertEqual(self.actions.land_calls, 0)
        self.assertEqual(self.actions.disarm_calls, 0)
        self.assertEqual(self.sock.sent, [])

    def _hold_for_handoff(self) -> list[int]:
        self.test_reaching_height_keeps_hold_stream_active()
        calls: list[int] = []
        self.coordinator._keyboard_handoff = lambda: calls.append(len(self.sock.sent)) or {"enabled": True}
        return calls

    def test_legacy_callback_cannot_handoff_after_three_seconds(self) -> None:
        calls = self._hold_for_handoff()
        self._elapse(2.8)
        self.coordinator.poll_once()
        self.assertEqual(calls, [])
        self._advance(0.2)
        self.coordinator.poll_once()
        self.assertEqual(calls, [])
        self.assertEqual(self.coordinator.snapshot()["phase"], "HOLDING")
        self.assertTrue(self.coordinator.snapshot()["active"])
        self.assertIsNone(self.coordinator.snapshot()["keyboard_handoff_remaining_seconds"])
        packets = len(self.sock.sent)
        self._elapse(1.0)
        self.coordinator.poll_once()
        self.assertGreater(len(self.sock.sent), packets)

    def test_unstable_hover_resets_handoff_countdown(self) -> None:
        calls = self._hold_for_handoff()
        self.telemetry.vz_m_s = 1.0
        self._elapse(3.1)
        self.coordinator.poll_once()
        self.telemetry.vz_m_s = 0.0
        self._advance(0.1)
        self.coordinator.poll_once()
        self.assertEqual(calls, [])
        self.assertIsNone(self.coordinator.snapshot()["keyboard_handoff_remaining_seconds"])

    def test_failed_handoff_keeps_position_stream(self) -> None:
        self._hold_for_handoff()
        def fail():
            raise ValueError("browser offline")
        self.coordinator._keyboard_handoff = fail
        self._elapse(3.1)
        self.coordinator.poll_once()
        self.assertTrue(self.coordinator.snapshot()["active"])
        self.assertEqual(self.coordinator.snapshot()["phase"], "HOLDING")
        packets = len(self.sock.sent)
        self._advance(0.1)
        self.coordinator.poll_once()
        self.assertGreater(len(self.sock.sent), packets)

    def test_pilot_takeover_prevents_automatic_handoff(self) -> None:
        calls = self._hold_for_handoff()
        self.telemetry.rc_channel_8_pwm = 1000
        self.telemetry.flight_mode = "POSCTL"
        self._advance(3.1)
        self.coordinator.poll_once()
        self.assertEqual(calls, [])
        self.assertFalse(self.coordinator.snapshot()["active"])


    def test_drift_does_not_recapture_target_or_enable_keyboard(self) -> None:
        calls = self._hold_for_handoff()
        original = self._decode(self.sock.sent[-1][0])
        self.telemetry.local_y_m += 0.2
        self.telemetry.vy_m_s = 0.12
        self._elapse(3.1)
        self.coordinator.poll_once()
        current = self._decode(self.sock.sent[-1][0])
        self.assertEqual(calls, [])
        self.assertEqual(current.type_mask, LOCAL_TRAJECTORY_TYPE_MASK)
        self.assertAlmostEqual(current.x, original.x)
        self.assertAlmostEqual(current.y, original.y)
        self.assertAlmostEqual(current.z, original.z)

    def test_nonfinite_flight_data_stops_without_sending_action(self) -> None:
        for field in ("local_x_m", "local_y_m", "local_z_m", "vx_m_s", "vy_m_s", "vz_m_s", "roll_deg", "pitch_deg", "yaw_deg"):
            for bad in (None, float("nan"), float("inf")):
                with self.subTest(field=field, value=bad):
                    self.setUp()
                    self._hold_for_handoff()
                    packets = len(self.sock.sent)
                    setattr(self.telemetry, field, bad)
                    self._advance(0.1)
                    self.coordinator.poll_once()
                    self.assertFalse(self.coordinator.snapshot()["active"])
                    self.assertEqual(len(self.sock.sent), packets)
                    self.assertEqual(self.actions.land_calls, 0)
                    self.assertEqual(self.actions.disarm_calls, 0)

    def test_pilot_mode_exit_wins_over_stale_position(self) -> None:
        self._hold_for_handoff()
        self.telemetry.flight_mode = "POSCTL"
        self.telemetry.last_local_position_monotonic = None
        self.coordinator.poll_once()
        self.assertEqual(self.coordinator.snapshot()["phase"], "PILOT_TAKEOVER")

    def test_land_mode_yields_without_reasserting_position(self) -> None:
        calls = self._hold_for_handoff()
        self.telemetry.flight_mode = "LAND"
        packets = len(self.sock.sent)
        self._advance(3.1)
        self.coordinator.poll_once()
        self.assertEqual(calls, [])
        self.assertEqual(len(self.sock.sent), packets)
        self.assertEqual(self.coordinator.snapshot()["phase"], "PILOT_TAKEOVER")
        self.assertEqual(self.actions.land_calls, 0)
        self.assertEqual(self.actions.disarm_calls, 0)

if __name__ == "__main__":
    unittest.main()
