from __future__ import annotations

from pathlib import Path
import sys
from time import monotonic, sleep
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pymavlink.dialects.v20 import common as mavlink2

from uav_preview.config import GroundOffboardTestConfig, KeyboardControlConfig
from uav_preview.keyboard_control import (
    KEYBOARD_VELOCITY_TYPE_MASK,
    KeyboardOffboardSetpointSender,
)
from uav_preview.types import TelemetrySnapshot


class _Socket:
    def __init__(self) -> None:
        self.sent: list[tuple[bytes, tuple[str, int]]] = []

    def sendto(self, packet: bytes, endpoint: tuple[str, int]) -> int:
        self.sent.append((packet, endpoint))
        return len(packet)

    def close(self) -> None:
        pass


class KeyboardControlTests(unittest.TestCase):
    def setUp(self) -> None:
        self.telemetry = TelemetrySnapshot(
            connected=True,
            system_id=1,
            component_id=1,
            flight_mode="POSCTL",
            armed=False,
            landed_state="ON_GROUND",
            rc_channel_6_pwm=2000,
            last_packet_monotonic=monotonic(),
        )
        self.sock = _Socket()
        self.estop = False
        self.sender = KeyboardOffboardSetpointSender(
            KeyboardControlConfig(available=True),
            GroundOffboardTestConfig(available=True),
            lambda: self.telemetry,
            lambda: self.estop,
            1.5,
            socket_factory=lambda *_: self.sock,
        )
        self.sender.start()

    def tearDown(self) -> None:
        self.sender.stop()

    @staticmethod
    def _decode(packet: bytes) -> object:
        parser = mavlink2.MAVLink(None)
        return (parser.parse_buffer(packet) or [None])[0]

    def test_ground_qgc_test_requires_physical_kill_switch(self) -> None:
        self.telemetry.rc_channel_6_pwm = 1000
        with self.assertRaisesRegex(ValueError, "Kill Switch"):
            self.sender.enable()

    def test_keyboard_axes_encode_body_velocity_and_yaw_rate(self) -> None:
        self.sender.enable()
        self.sender.update_axes(1.0, -1.0, 1.0, 1.0)
        sleep(0.14)
        self.sender.disable()
        message = self._decode(self.sock.sent[-1][0])
        self.assertEqual(message.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertEqual(message.coordinate_frame, mavlink2.MAV_FRAME_BODY_NED)
        self.assertEqual(message.type_mask, KEYBOARD_VELOCITY_TYPE_MASK)
        self.assertAlmostEqual(message.vx, 0.25)
        self.assertAlmostEqual(message.vy, -0.25)
        self.assertAlmostEqual(message.vz, -0.20)
        self.assertAlmostEqual(message.yaw_rate, 0.1745329, places=5)

    def test_airborne_sender_stops_when_rc_leaves_offboard(self) -> None:
        self.telemetry.armed = True
        self.telemetry.landed_state = "IN_AIR"
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_6_pwm = 1000
        self.sender.enable()
        self.telemetry.flight_mode = "POSCTL"
        sleep(0.15)
        self.assertFalse(self.sender.snapshot()["enabled"])
        self.assertIn("自动停止", self.sender.snapshot()["last_stop_reason"])

    def test_ground_takeoff_accepts_only_shift_until_in_air(self) -> None:
        self.sender.config.allow_ground_takeoff = True
        self.telemetry.armed = True
        self.telemetry.landed_state = "ON_GROUND"
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_6_pwm = 1000
        self.sender.enable()
        self.sender.update_axes(1.0, -1.0, 1.0, 1.0)
        sleep(0.14)
        message = self._decode(self.sock.sent[-1][0])
        self.assertEqual(self.sender.snapshot()["mode"], "GROUND_TAKEOFF")
        self.assertEqual((message.vx, message.vy), (0.0, 0.0))
        self.assertAlmostEqual(message.vz, -0.20)
        self.assertEqual(message.yaw_rate, 0.0)

        self.telemetry.landed_state = "IN_AIR"
        self.sender.update_axes(1.0, -1.0, 1.0, 1.0)
        sleep(0.14)
        message = self._decode(self.sock.sent[-1][0])
        self.assertEqual(self.sender.snapshot()["mode"], "FLIGHT_OFFBOARD")
        self.assertAlmostEqual(message.vx, 0.25)
        self.assertAlmostEqual(message.vy, -0.25)
        self.assertAlmostEqual(message.vz, -0.20)
        self.assertAlmostEqual(message.yaw_rate, 0.1745329, places=5)

    def test_estop_blocks_sender(self) -> None:
        self.estop = True
        with self.assertRaisesRegex(ValueError, "紧急制动"):
            self.sender.enable()


if __name__ == "__main__":
    unittest.main()
