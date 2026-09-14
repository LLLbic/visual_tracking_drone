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
            rc_channel_8_pwm=1000,
            last_packet_monotonic=monotonic(),
        )
        self.sock = _Socket()
        now = monotonic()
        for field in ("last_heartbeat_monotonic", "last_extended_state_monotonic", "last_rc_channels_monotonic",
                      "last_local_position_monotonic", "last_attitude_monotonic"):
            setattr(self.telemetry, field, now)
        for field in ("local_x_m", "local_y_m", "local_z_m", "vx_m_s", "vy_m_s", "vz_m_s", "yaw_deg"):
            setattr(self.telemetry, field, 0.0)
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

    def test_real_keyboard_tx_is_forbidden_on_ground(self) -> None:
        with self.assertRaisesRegex(ValueError, "禁止从地面"):
            self.sender.enable()

    def test_automatic_handoff_requires_browser_and_first_packet_is_neutral(self) -> None:
        self.telemetry.armed = True
        self.telemetry.landed_state = "IN_AIR"
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_6_pwm = 1000
        self.telemetry.rc_channel_8_pwm = 2000
        now = monotonic()
        self.telemetry.last_heartbeat_monotonic = now
        self.telemetry.last_extended_state_monotonic = now
        self.telemetry.last_rc_channels_monotonic = now
        self.telemetry.last_local_position_monotonic = now
        with self.assertRaisesRegex(ValueError, "网页"):
            self.sender.enable_from_takeoff()
        self.assertEqual(self.sock.sent, [])
        self.sender.browser_presence()
        self.sender.enable_from_takeoff()
        message = self._decode(self.sock.sent[0][0])
        self.assertEqual((message.vx, message.vy, message.vz, message.yaw_rate), (0, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, "松开"):
            self.sender.update_axes(1, 0, 0, 0)
        self.sender.update_axes(0, 0, 0, 0)
        self.sender.update_axes(1, 0, 0, 0)

    def test_keyboard_axes_encode_body_velocity_and_yaw_rate(self) -> None:
        self.telemetry.armed = True
        self.telemetry.landed_state = "IN_AIR"
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_6_pwm = 1000
        self.telemetry.rc_channel_8_pwm = 2000
        self.sender.enable()
        self.sender.update_axes(1.0, -1.0, 1.0, 1.0)
        sleep(0.14)
        self.sender.disable()
        message = self._decode(self.sock.sent[-1][0])
        self.assertEqual(message.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertEqual(message.coordinate_frame, mavlink2.MAV_FRAME_BODY_NED)
        self.assertEqual(message.type_mask, KEYBOARD_VELOCITY_TYPE_MASK)
        self.assertGreater(message.vx, 0.0)
        self.assertLessEqual(message.vx, 0.075)
        self.assertAlmostEqual(message.vy, -message.vx)
        self.assertLess(message.vz, 0.0)
        self.assertGreaterEqual(message.vz, -0.045)
        self.assertGreater(message.yaw_rate, 0.0)
        self.assertLessEqual(message.yaw_rate, 0.053)

    def test_airborne_sender_stops_when_rc_leaves_offboard(self) -> None:
        self.telemetry.armed = True
        self.telemetry.landed_state = "IN_AIR"
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_6_pwm = 1000
        self.telemetry.rc_channel_8_pwm = 2000
        self.sender.enable()
        self.telemetry.flight_mode = "POSCTL"
        sleep(0.15)
        self.assertFalse(self.sender.snapshot()["enabled"])
        self.assertIn("自动停止", self.sender.snapshot()["last_stop_reason"])

    def test_ground_takeoff_stays_forbidden_even_if_legacy_flag_is_set(self) -> None:
        self.sender.config.allow_ground_takeoff = True
        self.telemetry.armed = True
        self.telemetry.landed_state = "ON_GROUND"
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_6_pwm = 1000
        self.telemetry.rc_channel_8_pwm = 2000
        with self.assertRaisesRegex(ValueError, "禁止从地面"):
            self.sender.enable()

    def test_airborne_sender_stops_when_physical_ch8_leaves_offboard(self) -> None:
        self.telemetry.armed = True
        self.telemetry.landed_state = "IN_AIR"
        self.telemetry.flight_mode = "OFFBOARD"
        self.telemetry.rc_channel_6_pwm = 1000
        self.telemetry.rc_channel_8_pwm = 2000
        self.sender.enable()
        self.telemetry.rc_channel_8_pwm = 1000
        sleep(0.14)
        state = self.sender.snapshot()
        self.assertFalse(state["enabled"])
        self.assertIn("CH8", state["last_stop_reason"])

    def test_estop_blocks_sender(self) -> None:
        self.estop = True
        with self.assertRaisesRegex(ValueError, "安全处置"):
            self.sender.enable()


if __name__ == "__main__":
    unittest.main()
