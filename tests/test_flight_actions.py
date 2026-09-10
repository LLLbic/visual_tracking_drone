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
    ) -> ExplicitFlightActionSender:
        return ExplicitFlightActionSender(
            GroundOffboardTestConfig(available=True),
            lambda: telemetry,
            lambda: ground_enabled,
            lambda: False,
            lambda: True,
            1.5,
            ground_takeoff_ready=lambda: ground_ready,
            require_ground_takeoff_prestream=require_prestream,
            socket_factory=lambda _family, _kind: _CaptureSocket(sent),
        )

    @staticmethod
    def _telemetry(**overrides: object) -> TelemetrySnapshot:
        values: dict[str, object] = {
            "connected": True,
            "system_id": 1,
            "component_id": 1,
            "flight_mode": "POSCTL",
            "armed": False,
            "landed_state": "ON_GROUND",
            "last_packet_monotonic": monotonic(),
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
        with self.assertRaisesRegex(ValueError, "尚未达到稳定时长"):
            self._sender(self._telemetry(), [], ground_enabled=True).set_armed(True)

    def test_arm_allows_ready_offboard_ground_prestream(self) -> None:
        sent: list[tuple[bytes, tuple[str, int]]] = []
        telemetry = self._telemetry(
            flight_mode="OFFBOARD",
            rc_channel_6_pwm=1000,
        )
        state = self._sender(
            telemetry,
            sent,
            ground_enabled=True,
            ground_ready=True,
            require_prestream=True,
        ).set_armed(True)
        self.assertEqual(len(sent), 1)
        self.assertEqual(self._parse(sent[0][0]).command, 400)
        self.assertEqual(state["last_action"], "ARM")

    def test_guarded_takeoff_build_rejects_arm_without_prestream(self) -> None:
        with self.assertRaisesRegex(ValueError, "要求先开启"):
            self._sender(
                self._telemetry(), [], require_prestream=True
            ).set_armed(True)

    def test_ground_takeoff_arm_requires_known_released_kill_switch(self) -> None:
        kwargs = {"ground_enabled": True, "ground_ready": True}
        with self.assertRaisesRegex(ValueError, "尚未收到实体Kill Switch"):
            self._sender(
                self._telemetry(flight_mode="OFFBOARD"), [], **kwargs
            ).set_armed(True)
        with self.assertRaisesRegex(ValueError, "仍处于触发状态"):
            self._sender(
                self._telemetry(flight_mode="OFFBOARD", rc_channel_6_pwm=2000),
                [],
                **kwargs,
            ).set_armed(True)

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


if __name__ == "__main__":
    unittest.main()
