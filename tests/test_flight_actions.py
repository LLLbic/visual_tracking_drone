from __future__ import annotations

from pathlib import Path
import socket
import sys
from time import monotonic
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pymavlink.dialects.v20 import common as mavlink2

from uav_preview.config import GroundOffboardTestConfig
from uav_preview.flight_actions import ExplicitFlightActionSender
from uav_preview.safety import SafetyGate
from uav_preview.types import TelemetrySnapshot


class _CaptureSocket:
    def __init__(self, sent: list[tuple[bytes, tuple[str, int]]]) -> None:
        self.sent = sent

    def sendto(self, packet: bytes, endpoint: tuple[str, int]) -> None:
        self.sent.append((packet, endpoint))

    def close(self) -> None:
        pass


class ExplicitFlightActionTests(unittest.TestCase):
    def _sender(
        self,
        telemetry: TelemetrySnapshot,
        sent: list[tuple[bytes, tuple[str, int]]],
        *,
        ground_enabled: bool = False,
        ground_ready: bool = False,
        require_prestream: bool = False,
        estop_latched: bool = False,
    ) -> ExplicitFlightActionSender:
        gate = SafetyGate(initially_latched=estop_latched)
        return ExplicitFlightActionSender(
            GroundOffboardTestConfig(available=True),
            lambda: telemetry,
            lambda: ground_enabled,
            lambda: gate.estop_latched,
            lambda: True,
            1.5,
            ground_takeoff_ready=lambda: ground_ready,
            require_ground_takeoff_prestream=require_prestream,
            socket_factory=lambda _family, _kind: _CaptureSocket(sent),
            unlatched_action_guard=gate.run_if_unlatched,
        )

    @staticmethod
    def _telemetry(**overrides: object) -> TelemetrySnapshot:
        now = monotonic()
        values: dict[str, object] = {
            "connected": True,
            "system_id": 1,
            "component_id": 1,
            "flight_mode": "POSCTL",
            "armed": False,
            "landed_state": "ON_GROUND",
            "rc_channel_6_pwm": 1000,
            "rc_channel_8_pwm": 1000,
            "last_packet_monotonic": now,
            "last_heartbeat_monotonic": now,
            "last_extended_state_monotonic": now,
            "last_rc_channels_monotonic": now,
            "last_global_position_monotonic": now,
        }
        values.update(overrides)
        return TelemetrySnapshot(**values)

    @staticmethod
    def _parse(packet: bytes) -> object:
        parser = mavlink2.MAVLink(None)
        return (parser.parse_buffer(packet) or [None])[0]

    def test_arm_is_one_explicit_command_long(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        state = self._sender(self._telemetry(), sent).set_armed(True)
        self.assertEqual(len(sent), 1)
        message = self._parse(sent[0][0])
        self.assertEqual(message.get_type(), "COMMAND_LONG")
        self.assertEqual(message.command, 400)
        self.assertEqual(message.param1, 1.0)
        self.assertEqual(sent[0][1], ("127.0.0.1", 14560))
        self.assertEqual(state["commands_sent"], 1)

    def test_arm_rejects_offboard_or_unready_ground_sender(self) -> None:
        with self.assertRaisesRegex(ValueError, "当前模式为OFFBOARD"):
            self._sender(self._telemetry(flight_mode="OFFBOARD"), []).set_armed(True)
        with self.assertRaisesRegex(ValueError, "关闭所有电脑Offboard"):
            self._sender(self._telemetry(), [], ground_enabled=True).set_armed(True)

    def test_arm_rejects_ready_offboard_ground_prestream(self) -> None:
        with self.assertRaisesRegex(ValueError, "关闭所有电脑Offboard"):
            self._sender(
                self._telemetry(flight_mode="OFFBOARD", rc_channel_8_pwm=2000),
                [],
                ground_enabled=True,
                ground_ready=True,
                require_prestream=True,
            ).set_armed(True)

    def test_guarded_takeoff_build_rejects_arm_without_prestream(self) -> None:
        with self.assertRaisesRegex(ValueError, "安全策略禁用"):
            self._sender(
                self._telemetry(), [], require_prestream=True
            ).set_armed(True)

    def test_dedicated_local_takeoff_arm_requires_offboard_and_physical_ch8(self) -> None:
        sender = self._sender(self._telemetry(), [])
        with self.assertRaisesRegex(ValueError, "预发送"):
            sender.arm_for_local_offboard_takeoff(prestream_ready=False)
        with self.assertRaisesRegex(ValueError, "Offboard"):
            sender.arm_for_local_offboard_takeoff(prestream_ready=True)

        telemetry = self._telemetry(flight_mode="OFFBOARD", rc_channel_8_pwm=1000)
        with self.assertRaisesRegex(ValueError, "CH8"):
            self._sender(telemetry, []).arm_for_local_offboard_takeoff(
                prestream_ready=True
            )

    def test_dedicated_local_takeoff_arm_sends_one_normal_arm(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        telemetry = self._telemetry(flight_mode="OFFBOARD", rc_channel_8_pwm=2000)
        state = self._sender(telemetry, sent).arm_for_local_offboard_takeoff(
            prestream_ready=True
        )
        self.assertEqual(len(sent), 1)
        message = self._parse(sent[0][0])
        self.assertEqual(message.command, 400)
        self.assertEqual(message.param1, 1.0)
        self.assertEqual(state["last_action"], "ARM-LOCAL-OFFBOARD")

    def test_ground_arm_requires_known_released_kill_and_manual_ch8(self) -> None:
        with self.assertRaisesRegex(ValueError, "CH6"):
            self._sender(self._telemetry(rc_channel_6_pwm=None), []).set_armed(True)
        with self.assertRaisesRegex(ValueError, "仍处于触发状态"):
            self._sender(self._telemetry(rc_channel_6_pwm=2000), []).set_armed(True)
        with self.assertRaisesRegex(ValueError, "CH8"):
            self._sender(self._telemetry(rc_channel_8_pwm=None), []).set_armed(True)
        with self.assertRaisesRegex(ValueError, "仍选择Offboard"):
            self._sender(self._telemetry(rc_channel_8_pwm=2000), []).set_armed(True)

    def test_disarm_requires_confirmed_on_ground(self) -> None:
        with self.assertRaisesRegex(ValueError, "只允许.*ON_GROUND"):
            self._sender(
                self._telemetry(armed=True, landed_state="IN_AIR"), []
            ).set_armed(False)

    def test_brake_matches_qgc_px4_pause_command(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        telemetry = self._telemetry(armed=True, landed_state="IN_AIR")
        self._sender(telemetry, sent).brake()
        message = self._parse(sent[0][0])
        self.assertEqual(message.command, 192)
        self.assertEqual(message.param1, -1.0)
        self.assertEqual(message.param2, 1.0)
        self.assertTrue(message.param4 != message.param4)  # NaN

    def test_brake_allows_disarmed_on_ground_link_test(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        state = self._sender(self._telemetry(), sent).brake()
        message = self._parse(sent[0][0])
        self.assertEqual(message.command, 192)
        self.assertEqual(state["last_action"], "BRAKE/PAUSE-GROUND-TEST")

    def test_latch_blocks_arm_pause_rtl_and_takeoff_at_final_send(self) -> None:
        scenarios = (
            ("ARM", self._telemetry(), lambda sender: sender.set_armed(True)),
            (
                "Pause",
                self._telemetry(armed=True, landed_state="IN_AIR"),
                lambda sender: sender.brake(),
            ),
            (
                "RTL",
                self._telemetry(armed=True, landed_state="IN_AIR"),
                lambda sender: sender.rtl(),
            ),
            (
                "Takeoff",
                self._telemetry(
                    armed=True,
                    latitude_deg=30.25,
                    longitude_deg=119.75,
                    global_altitude_amsl_m=102.4,
                ),
                lambda sender: sender.takeoff(1.5, 102.4),
            ),
        )
        for label, telemetry, invoke in scenarios:
            with self.subTest(action=label):
                sent: list[tuple[bytes, tuple[str, int]]] = []
                with self.assertRaisesRegex(ValueError, "锁存"):
                    invoke(self._sender(telemetry, sent, estop_latched=True))
                self.assertEqual(sent, [])

    def test_latch_still_allows_controlled_land_and_ground_disarm(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        self._sender(
            self._telemetry(armed=True, landed_state="IN_AIR"),
            sent,
            estop_latched=True,
        ).land()
        self._sender(
            self._telemetry(), sent, estop_latched=True
        ).safety_disarm_on_ground()
        self.assertEqual(self._parse(sent[0][0]).command, 21)
        self.assertEqual(self._parse(sent[1][0]).command, 400)

    def test_safety_disarm_on_ground_cancels_immediately_after_arm(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        sender = self._sender(self._telemetry(), sent)
        sender.set_armed(True)
        state = sender.safety_disarm_on_ground()
        self.assertEqual(len(sent), 2)
        message = self._parse(sent[-1][0])
        self.assertEqual(message.command, 400)
        self.assertEqual(message.param1, 0.0)
        self.assertEqual(state["last_action"], "DISARM-SAFETY-LATCH")

    def test_brake_rejects_disarmed_in_air_or_unknown_landed_state(self) -> None:
        with self.assertRaisesRegex(ValueError, "状态不一致"):
            self._sender(
                self._telemetry(armed=False, landed_state="IN_AIR"), []
            ).brake()
        with self.assertRaisesRegex(ValueError, "ON_GROUND"):
            self._sender(
                self._telemetry(armed=False, landed_state="UNKNOWN"), []
            ).brake()

    def test_land_and_rtl_are_explicit_airborne_commands(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        telemetry = self._telemetry(armed=True, landed_state="IN_AIR")
        sender = self._sender(telemetry, sent)
        sender.land()
        sender.rtl()
        self.assertEqual(self._parse(sent[0][0]).command, 21)
        self.assertEqual(self._parse(sent[1][0]).command, 20)

    def test_land_and_rtl_reject_ground_state(self) -> None:
        sender = self._sender(self._telemetry(), [])
        with self.assertRaisesRegex(ValueError, "ARMED.*IN_AIR"):
            sender.land()
        with self.assertRaisesRegex(ValueError, "ARMED.*IN_AIR"):
            sender.rtl()

    def test_takeoff_is_one_command_int_with_absolute_target_altitude(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        telemetry = self._telemetry(
            armed=True,
            latitude_deg=30.25,
            longitude_deg=119.75,
            global_altitude_amsl_m=102.4,
        )
        state = self._sender(telemetry, sent).takeoff(1.5, 102.4)
        self.assertEqual(len(sent), 1)
        message = self._parse(sent[0][0])
        self.assertEqual(message.get_type(), "COMMAND_INT")
        self.assertEqual(message.command, mavlink2.MAV_CMD_NAV_TAKEOFF)
        self.assertEqual(message.frame, mavlink2.MAV_FRAME_GLOBAL_INT)
        self.assertEqual(message.x, 302_500_000)
        self.assertEqual(message.y, 1_197_500_000)
        self.assertAlmostEqual(message.z, 103.9, places=3)
        self.assertEqual(state["last_command"], 22)

    def test_takeoff_height_is_hard_limited_to_one_through_three_metres(self) -> None:
        telemetry = self._telemetry(
            armed=True,
            latitude_deg=30.25,
            longitude_deg=119.75,
            global_altitude_amsl_m=102.4,
        )
        sender = self._sender(telemetry, [])
        with self.assertRaisesRegex(ValueError, "1.0至3.0"):
            sender.takeoff(0.9, 102.4)
        with self.assertRaisesRegex(ValueError, "1.0至3.0"):
            sender.takeoff(3.1, 102.4)


if __name__ == "__main__":
    unittest.main()
