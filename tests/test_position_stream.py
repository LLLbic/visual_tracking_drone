from __future__ import annotations

from pathlib import Path
import socket
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pymavlink.dialects.v20 import common as mavlink2

from uav_preview.config import PositionStreamConfig
from uav_preview.position_stream import PositionStreamRateRequester
from uav_preview.types import TelemetrySnapshot


class _RecordingSocket:
    def __init__(self, packets: list[tuple[bytes, tuple[str, int]]]) -> None:
        self._packets = packets

    def sendto(self, packet: bytes, endpoint: tuple[str, int]) -> None:
        self._packets.append((packet, endpoint))

    def close(self) -> None:
        return


class PositionStreamRateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now = 100.0
        self.telemetry = TelemetrySnapshot(
            connected=True,
            system_id=1,
            component_id=1,
            armed=False,
            landed_state="ON_GROUND",
            local_position_hz=0.5,
            global_position_hz=None,
            extended_state_hz=0.5,
            last_local_position_monotonic=self.now,
            last_extended_state_monotonic=self.now,
            last_heartbeat_monotonic=self.now,
        )
        self.packets: list[tuple[bytes, tuple[str, int]]] = []
        self.requester = PositionStreamRateRequester(
            PositionStreamConfig(),
            lambda: self.telemetry,
            lambda: True,
            socket_factory=lambda _family, _kind: _RecordingSocket(self.packets),
        )

    def test_requests_local_and_global_position_at_five_hz(self) -> None:
        self.requester._step(self.now)

        self.assertEqual(len(self.packets), 4)
        decoded = []
        for packet, endpoint in self.packets:
            parser = mavlink2.MAVLink(None)
            message = parser.parse_char(packet)
            self.assertIsNotNone(message)
            self.assertEqual(endpoint, ("127.0.0.1", 14560))
            self.assertEqual(message.command, mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL)
            message_id = int(message.param1)
            expected_interval = 500_000.0 if message_id == 245 else 200_000.0
            self.assertAlmostEqual(message.param2, expected_interval)
            decoded.append(message_id)
        self.assertEqual(set(decoded), {32, 33, 230, 245})
        state = self.requester.snapshot()
        self.assertEqual(state["attempts"], 1)
        self.assertEqual(state["requests_sent"], 4)

    def test_never_requests_while_armed(self) -> None:
        self.telemetry.armed = True
        self.telemetry.landed_state = "IN_AIR"
        self.requester._step(self.now)
        self.assertEqual(self.packets, [])
        self.assertEqual(self.requester.snapshot()["status"], "WAITING")

    def test_effective_rates_need_no_request(self) -> None:
        self.telemetry.local_position_hz = 5.0
        self.telemetry.global_position_hz = 5.0
        self.telemetry.extended_state_hz = 2.0
        self.telemetry.estimator_hz = 5.0
        self.telemetry.last_estimator_monotonic = self.now
        self.telemetry.last_global_position_monotonic = self.now
        self.requester._step(self.now)
        self.assertEqual(self.packets, [])
        state = self.requester.snapshot()
        self.assertEqual(state["status"], "EFFECTIVE")
        self.assertTrue(state["local_effective"])
        self.assertTrue(state["global_effective"])

    def fresh(self, now):
        for name in ('last_heartbeat_monotonic', 'last_extended_state_monotonic',
                     'last_local_position_monotonic', 'last_global_position_monotonic',
                     'last_estimator_monotonic'):
            setattr(self.telemetry, name, now)

    def test_indoor_global_absence_not_reported_as_local_failure(self):
        self.requester.config.require_global_position = False
        self.telemetry.local_position_hz = self.telemetry.estimator_hz = 5
        self.telemetry.extended_state_hz = 2
        self.fresh(self.now)
        self.telemetry.last_global_position_monotonic = None
        self.requester._step(self.now)
        self.assertEqual(self.packets, [])
        self.assertEqual(self.requester.snapshot()['status'], 'EFFECTIVE')
        self.assertFalse(self.requester.snapshot()['global_effective'])

    def test_stale_or_future_ground_evidence_prevents_request(self):
        for field in ('last_heartbeat_monotonic', 'last_extended_state_monotonic'):
            for value in (None, 90, 101, float('nan')):
                self.fresh(100)
                setattr(self.telemetry, field, value)
                self.requester._step(100)
                self.assertEqual(self.packets, [])

    def test_global_exhaustion_does_not_use_local_retry_budget(self):
        self.telemetry.local_position_hz = self.telemetry.estimator_hz = 5
        self.telemetry.extended_state_hz = 2
        for now in (100, 103, 106):
            self.fresh(now)
            self.requester._step(now)
        self.assertEqual(len(self.packets), 3)  # only missing GPS
        self.fresh(110)
        self.telemetry.local_position_hz = 0.5
        self.requester._step(110)
        self.assertEqual(len(self.packets), 4)
        self.assertEqual(self.requester.snapshot()['stream_attempts']['LOCAL_POSITION_NED'], 1)

    def test_rechecks_disarm_before_each_request(self):
        original = self.requester._send_interval_request
        def send(*args):
            original(*args)
            self.telemetry.armed = True
        self.requester._send_interval_request = send
        self.requester._step(100)
        self.assertEqual(len(self.packets), 1)
        self.assertEqual(self.requester.snapshot()['status'], 'WAITING')

    def test_socket_errors_are_rate_limited_and_attempts_bounded(self):
        self.requester.config.require_global_position = False
        self.telemetry.extended_state_hz = 2
        self.telemetry.estimator_hz = 5
        calls = []
        def failing(*args):
            calls.append(args)
            raise OSError('test only')
        self.requester._send_interval_request = failing
        for index in range(100):
            now = 100 + index * 0.2
            self.fresh(now)
            self.requester._step(now)
        self.assertEqual(len(calls), 3)
        self.assertEqual(self.requester.snapshot()['status'], 'EXHAUSTED')

    def test_nonfinite_or_future_observation_cannot_mark_stream_effective(self):
        self.assertFalse(self.requester._rate_effective(float('inf'), 100, 100))
        self.assertFalse(self.requester._rate_effective(5, 101, 100))


if __name__ == "__main__":
    unittest.main()
