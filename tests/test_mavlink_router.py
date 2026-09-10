from __future__ import annotations

from pathlib import Path
import socket
import sys
from time import sleep
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.config import MavlinkRouterConfig
from uav_preview.mavlink_router import LocalMavlinkRouter


def _free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def _mavlink2(
    message_id: int,
    *,
    system_id: int = 255,
    component_id: int = 190,
    payload: bytes = b"",
) -> bytes:
    """Minimal MAVLink 2 frame; router policy does not need CRC validation."""

    header = bytes(
        (
            0xFD,
            len(payload),
            0,
            0,
            0,
            system_id,
            component_id,
            message_id & 0xFF,
            (message_id >> 8) & 0xFF,
            (message_id >> 16) & 0xFF,
        )
    )
    return header + payload + b"\x00\x00"


def _command_long(command: int) -> bytes:
    return _mavlink2(
        76,
        payload=b"\x00" * 28 + command.to_bytes(2, "little") + b"\x00" * 3,
    )


class MavlinkRouterTests(unittest.TestCase):
    def test_bidirectional_fanout_preserves_vehicle_facing_source_port(self) -> None:
        router_port = _free_udp_port()
        vehicle_port = _free_udp_port()
        ingress_port = _free_udp_port()
        qgc_port = _free_udp_port()
        telemetry_port = _free_udp_port()
        config = MavlinkRouterConfig(
            enabled=True,
            vehicle_bind_host="127.0.0.1",
            vehicle_bind_port=router_port,
            vehicle_host="127.0.0.1",
            vehicle_port=vehicle_port,
            local_host="127.0.0.1",
            local_ingress_port=ingress_port,
            qgc_host="127.0.0.1",
            qgc_port=qgc_port,
            telemetry_host="127.0.0.1",
            telemetry_port=telemetry_port,
        )
        router = LocalMavlinkRouter(config)
        vehicle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        local_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for sock in (vehicle, qgc, telemetry):
            sock.settimeout(2.0)
        vehicle.bind(("127.0.0.1", vehicle_port))
        qgc.bind(("127.0.0.1", qgc_port))
        telemetry.bind(("127.0.0.1", telemetry_port))
        try:
            router.start()
            self.assertTrue(router.snapshot()["ready"])
            vehicle.sendto(b"vehicle-telemetry", ("127.0.0.1", router_port))
            self.assertEqual(qgc.recvfrom(1024)[0], b"vehicle-telemetry")
            self.assertEqual(telemetry.recvfrom(1024)[0], b"vehicle-telemetry")

            heartbeat = _mavlink2(0, payload=b"\x00" * 9)
            local_client.sendto(heartbeat, ("127.0.0.1", ingress_port))
            packet, source = vehicle.recvfrom(1024)
            self.assertEqual(packet, heartbeat)
            self.assertEqual(source[1], router_port)

            reflected_vehicle = _mavlink2(
                0, system_id=1, component_id=1, payload=b"\x00" * 9
            )
            vehicle.settimeout(0.15)
            local_client.sendto(reflected_vehicle, ("127.0.0.1", ingress_port))
            with self.assertRaises(socket.timeout):
                vehicle.recvfrom(1024)
            sleep(0.05)
            state = router.snapshot()
            self.assertEqual(state["vehicle_datagrams_received"], 1)
            self.assertEqual(state["local_datagrams_forwarded"], 1)
            self.assertEqual(state["blocked_reflected_vehicle"], 1)
        finally:
            router.stop()
            vehicle.close()
            qgc.close()
            telemetry.close()
            local_client.close()

    def test_rc_priority_blocks_competing_manual_but_allows_qgc_high_level_actions(self) -> None:
        router_port = _free_udp_port()
        vehicle_port = _free_udp_port()
        ingress_port = _free_udp_port()
        qgc_port = _free_udp_port()
        telemetry_port = _free_udp_port()
        config = MavlinkRouterConfig(
            enabled=True,
            vehicle_bind_host="127.0.0.1",
            vehicle_bind_port=router_port,
            vehicle_host="127.0.0.1",
            vehicle_port=vehicle_port,
            local_host="127.0.0.1",
            local_ingress_port=ingress_port,
            qgc_host="127.0.0.1",
            qgc_port=qgc_port,
            telemetry_host="127.0.0.1",
            telemetry_port=telemetry_port,
            enforce_rc_priority=True,
            approved_setpoint_system=245,
            approved_setpoint_component=191,
        )
        router = LocalMavlinkRouter(config)
        vehicle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        local_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        vehicle.settimeout(0.15)
        qgc.settimeout(1.0)
        telemetry.settimeout(1.0)
        vehicle.bind(("127.0.0.1", vehicle_port))
        qgc.bind(("127.0.0.1", qgc_port))
        telemetry.bind(("127.0.0.1", telemetry_port))
        try:
            router.start()
            blocked_packets = (
                _mavlink2(69, payload=b"\x00" * 11),
                _mavlink2(70, payload=b"\x00" * 38),
                _mavlink2(84, payload=b"\x00" * 53),
                b"not-mavlink",
            )
            for packet in blocked_packets:
                local_client.sendto(packet, ("127.0.0.1", ingress_port))
                with self.assertRaises(socket.timeout):
                    vehicle.recvfrom(1024)

            set_mode = _mavlink2(11, payload=b"\x00" * 6)
            qgc_high_level_commands = (
                _command_long(400),  # arm/disarm
                _command_long(22),   # takeoff
                _command_long(21),   # land
                _command_long(20),   # return to launch
                _command_long(192),  # reposition/pause used for guided braking
            )
            approved_setpoint = _mavlink2(
                84,
                system_id=245,
                component_id=191,
                payload=b"\x00" * 53,
            )
            for allowed in (set_mode, *qgc_high_level_commands, approved_setpoint):
                local_client.sendto(allowed, ("127.0.0.1", ingress_port))
                self.assertEqual(vehicle.recvfrom(1024)[0], allowed)

            self.assertEqual(qgc.recvfrom(1024)[0], approved_setpoint)

            # QGC may forward the mirrored Inspector packet back to the local
            # ingress. It must be consumed once instead of being mirrored
            # repeatedly into an accelerating feedback loop.
            qgc.sendto(approved_setpoint, ("127.0.0.1", ingress_port))
            with self.assertRaises(socket.timeout):
                vehicle.recvfrom(1024)
            sleep(0.05)

            state = router.snapshot()
            self.assertEqual(state["blocked_manual_control"], 1)
            self.assertEqual(state["blocked_rc_override"], 1)
            self.assertEqual(state["blocked_unapproved_setpoint"], 1)
            self.assertEqual(state["blocked_malformed_uplink"], 1)
            self.assertEqual(state["local_datagrams_forwarded"], 7)
            self.assertEqual(state["local_setpoints_mirrored_to_qgc"], 1)
            self.assertEqual(state["blocked_reflected_setpoint"], 1)
        finally:
            router.stop()
            vehicle.close()
            qgc.close()
            telemetry.close()
            local_client.close()


if __name__ == "__main__":
    unittest.main()
