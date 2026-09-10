from __future__ import annotations

from pathlib import Path
import socket
import sys
import tempfile
from time import sleep
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.config import TelemetryConfig, load_config
from uav_preview.telemetry import PassiveMavlinkReceiver


def _free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


class TelemetryRelayTests(unittest.TestCase):
    @staticmethod
    def _message(message_type: str, **values: object) -> object:
        message = SimpleNamespace(**values)
        message.get_type = lambda: message_type
        message.get_srcSystem = lambda: 1
        message.get_srcComponent = lambda: 1
        return message

    def test_config_rejects_non_loopback_qgc_target(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-relay.toml"
            path.write_text(
                "[telemetry]\n"
                "forward_qgc = true\n"
                "allowed_source_ips = ['192.168.1.201']\n"
                "qgc_host = '192.168.1.201'\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "loopback-only"):
                load_config(path)

    def test_forwards_exact_datagram_to_loopback(self) -> None:
        input_port = _free_udp_port()
        output_port = _free_udp_port()
        output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        output.bind(("127.0.0.1", output_port))
        output.settimeout(2.0)
        receiver = PassiveMavlinkReceiver(
            TelemetryConfig(
                bind_host="127.0.0.1",
                bind_port=input_port,
                allowed_source_ips=["127.0.0.1"],
                forward_qgc=True,
                qgc_host="127.0.0.1",
                qgc_port=output_port,
            )
        )
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = b"local-telemetry-copy-test"
        try:
            receiver.start()
            sleep(0.15)
            sender.sendto(payload, ("127.0.0.1", input_port))
            received, _ = output.recvfrom(4096)
            self.assertEqual(received, payload)
            sleep(0.05)
            state = receiver.snapshot()
            self.assertEqual(state.received_datagrams, 1)
            self.assertEqual(state.forwarded_datagrams, 1)
            self.assertEqual(state.dropped_datagrams, 0)
            self.assertEqual(state.qgc_forward_endpoint, f"127.0.0.1:{output_port}")
        finally:
            sender.close()
            receiver.stop()
            output.close()

    def test_captures_mode_mapping_and_status_text_read_only(self) -> None:
        receiver = PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        receiver._accept(
            self._message("PARAM_VALUE", param_id=b"RC_MAP_OFFB_SW\0", param_value=8.0),
            "127.0.0.1",
            object(),
        )
        receiver._accept(
            self._message("STATUSTEXT", text=b"Offboard mode rejected\0", severity=4),
            "127.0.0.1",
            object(),
        )
        state = receiver.snapshot()
        self.assertEqual(state.rc_map_offb_sw, 8)
        self.assertEqual(state.last_status_text, "Offboard mode rejected")
        self.assertEqual(state.last_status_severity, 4)
        self.assertIsNotNone(state.to_dict()["last_status_age_seconds"])

    def test_decodes_disarmed_px4_offboard_custom_main_mode(self) -> None:
        receiver = PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(
                MAV_COMP_ID_AUTOPILOT1=1,
                MAV_AUTOPILOT_PX4=12,
            ),
            mode_string_v10=lambda _message: "UNKNOWN",
        )
        receiver._accept(
            self._message(
                "HEARTBEAT",
                autopilot=12,
                base_mode=0x11,
                custom_mode=6 << 16,
            ),
            "127.0.0.1",
            mavutil,
        )
        state = receiver.snapshot()
        self.assertEqual(state.flight_mode, "OFFBOARD")
        self.assertFalse(state.armed)

    def test_uses_only_downward_distance_sensor_as_laser_height(self) -> None:
        receiver = PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_SENSOR_ROTATION_PITCH_270=25)
        )
        receiver._accept(
            self._message(
                "DISTANCE_SENSOR",
                orientation=0,
                current_distance=321,
                min_distance=20,
                max_distance=1200,
                id=1,
            ),
            "127.0.0.1",
            mavutil,
        )
        self.assertIsNone(receiver.snapshot().laser_height_m)

        receiver._accept(
            self._message(
                "DISTANCE_SENSOR",
                orientation=25,
                current_distance=234,
                min_distance=20,
                max_distance=1200,
                id=7,
            ),
            "127.0.0.1",
            mavutil,
        )
        state = receiver.snapshot()
        self.assertEqual(state.laser_height_m, 2.34)
        self.assertEqual(state.laser_min_m, 0.2)
        self.assertEqual(state.laser_max_m, 12.0)
        self.assertEqual(state.laser_sensor_id, 7)
        self.assertIsNotNone(state.to_dict()["laser_age_seconds"])

    def test_captures_landed_state_and_command_ack(self) -> None:
        receiver = PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        receiver._accept(
            self._message("EXTENDED_SYS_STATE", landed_state=2),
            "127.0.0.1",
            object(),
        )
        receiver._accept(
            self._message("COMMAND_ACK", command=400, result=0, progress=100),
            "127.0.0.1",
            object(),
        )
        state = receiver.snapshot()
        self.assertEqual(state.landed_state, "IN_AIR")
        self.assertEqual(state.last_command_ack_command, 400)
        self.assertEqual(state.last_command_ack_result, 0)
        self.assertEqual(state.last_command_ack_progress, 100)
        self.assertIsNotNone(state.to_dict()["last_command_ack_age_seconds"])

    def test_nan_parameter_value_does_not_break_passive_receiver(self) -> None:
        receiver = PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        receiver._accept(
            self._message("PARAM_VALUE", param_id=b"RC_MAP_OFFB_SW\0", param_value=float("nan")),
            "127.0.0.1",
            object(),
        )
        receiver._accept(
            self._message("PARAM_VALUE", param_id=b"RC_MAP_OFFB_SW\0", param_value=8.0),
            "127.0.0.1",
            object(),
        )
        self.assertEqual(receiver.snapshot().rc_map_offb_sw, 8)

    def test_non_autopilot_heartbeat_cannot_override_flight_state(self) -> None:
        receiver = PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_COMP_ID_AUTOPILOT1=1),
            mode_string_v10=lambda message: message.mode,
        )
        autopilot = self._message("HEARTBEAT", base_mode=0, mode="LAND")
        receiver._accept(autopilot, "127.0.0.1", mavutil)
        gcs = self._message("HEARTBEAT", base_mode=128, mode="Mode(0xC0)")
        gcs.get_srcComponent = lambda: 190
        receiver._accept(gcs, "127.0.0.1", mavutil)
        state = receiver.snapshot()
        self.assertEqual(state.flight_mode, "LAND")
        self.assertFalse(state.armed)
        self.assertEqual(state.component_id, 1)


if __name__ == "__main__":
    unittest.main()
