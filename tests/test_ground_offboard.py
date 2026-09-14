from __future__ import annotations

from pathlib import Path
import socket
import sys
from time import monotonic, sleep
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pymavlink.dialects.v20 import common as mavlink2

from uav_preview.config import GroundOffboardTestConfig
from uav_preview.offboard_test import (
    GroundOffboardSetpointSender,
    ZERO_VELOCITY_TYPE_MASK,
)
from uav_preview.types import TelemetrySnapshot


class FakeSocket:
    def __init__(self, *_: object) -> None:
        self.sent: list[tuple[bytes, tuple[str, int]]] = []
        self.closed = False

    def sendto(self, packet: bytes, endpoint: tuple[str, int]) -> int:
        self.sent.append((packet, endpoint))
        return len(packet)

    def close(self) -> None:
        self.closed = True


class GroundOffboardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.telemetry = TelemetrySnapshot(
            connected=True,
            system_id=1,
            component_id=1,
            flight_mode="LAND",
            armed=False,
            landed_state="ON_GROUND",
            last_packet_monotonic=monotonic(),
        )
        self.fake_socket = FakeSocket()
        self.estop = False
        self.sender = GroundOffboardSetpointSender(
            GroundOffboardTestConfig(available=True),
            lambda: self.telemetry,
            lambda: self.estop,
            stale_after_seconds=1.5,
            socket_factory=lambda *_: self.fake_socket,
        )
        self.sender.start()

    def tearDown(self) -> None:
        self.sender.stop()

    def test_default_is_off_and_sends_nothing(self) -> None:
        sleep(0.08)
        self.assertFalse(self.sender.snapshot()["enabled"])
        self.assertEqual(self.fake_socket.sent, [])

    def test_only_zero_velocity_local_ned_setpoints_are_sent(self) -> None:
        self.sender.enable()
        sleep(0.24)
        self.sender.disable()
        self.assertGreaterEqual(len(self.fake_socket.sent), 2)
        parser = mavlink2.MAVLink(None)
        packet, endpoint = self.fake_socket.sent[0]
        decoded = (parser.parse_buffer(packet) or [None])[0]
        self.assertEqual(endpoint, ("127.0.0.1", 14560))
        self.assertEqual(decoded.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertEqual(decoded.coordinate_frame, mavlink2.MAV_FRAME_LOCAL_NED)
        self.assertEqual(decoded.type_mask, ZERO_VELOCITY_TYPE_MASK)
        self.assertEqual((decoded.vx, decoded.vy, decoded.vz), (0.0, 0.0, 0.0))

    def test_refuses_to_start_when_armed(self) -> None:
        self.telemetry.armed = True
        with self.assertRaisesRegex(ValueError, "已解锁"):
            self.sender.enable()
        self.assertEqual(self.fake_socket.sent, [])

    def test_refuses_to_start_with_stale_telemetry(self) -> None:
        self.telemetry.last_packet_monotonic = monotonic() - 5.0
        with self.assertRaisesRegex(ValueError, "遥测离线或已过期"):
            self.sender.enable()

    def test_automatically_stops_if_vehicle_arms_outside_offboard(self) -> None:
        self.sender.enable()
        sleep(0.05)
        self.telemetry.armed = True
        sleep(0.25)
        state = self.sender.snapshot()
        count_after_stop = len(self.fake_socket.sent)
        sleep(0.22)
        self.assertFalse(state["enabled"])
        self.assertIn("自动停止", state["last_stop_reason"])
        self.assertEqual(len(self.fake_socket.sent), count_after_stop)

    def test_stable_ground_stream_stops_immediately_when_vehicle_arms(self) -> None:
        self.sender.disable()
        self.sender.stop()
        self.sender = GroundOffboardSetpointSender(
            GroundOffboardTestConfig(
                available=True,
                frequency_hz=50.0,
                arm_ready_after_seconds=0.05,
            ),
            lambda: self.telemetry,
            lambda: self.estop,
            stale_after_seconds=1.5,
            socket_factory=lambda *_: self.fake_socket,
        )
        self.sender.start()
        self.sender.enable()
        sleep(0.12)
        self.assertTrue(self.sender.snapshot()["ready_for_offboard_arm"])
        count_before_arm = len(self.fake_socket.sent)
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.armed = True
        self.telemetry.rc_channel_6_pwm = 1000
        sleep(0.08)
        state = self.sender.snapshot()
        self.assertFalse(state["enabled"])
        self.assertIn("DISARMED", state["last_stop_reason"])
        count_after_stop = len(self.fake_socket.sent)
        sleep(0.08)
        self.assertEqual(len(self.fake_socket.sent), count_after_stop)
        self.assertGreaterEqual(count_after_stop, count_before_arm)

    def test_estop_prevents_start(self) -> None:
        self.estop = True
        with self.assertRaisesRegex(ValueError, "急停"):
            self.sender.enable()


if __name__ == "__main__":
    unittest.main()
