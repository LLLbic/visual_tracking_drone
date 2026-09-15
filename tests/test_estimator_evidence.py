"""Offline estimator safety diagnostics; no flight endpoint or service used."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from uav_preview.config import TelemetryConfig, load_config
from uav_preview.navigation_health import estimator_ratio_reason, navigation_block_reasons
from uav_preview.smooth_handoff import handoff_evidence_reasons
from uav_preview.telemetry import PassiveMavlinkReceiver
from uav_preview.types import TelemetrySnapshot


class EstimatorEvidenceTests(unittest.TestCase):
    def message(self, sample_id=1000000, ratio=0.1, velocity_ratio=0.1):
        return SimpleNamespace(get_type=lambda: 'ESTIMATOR_STATUS',
            get_srcSystem=lambda: 1, get_srcComponent=lambda: 1,
            time_usec=sample_id, flags=367, vel_ratio=velocity_ratio, pos_horiz_ratio=ratio)

    def receiver(self):
        return PassiveMavlinkReceiver(TelemetryConfig())

    def accept(self, receiver, message, now=100):
        with patch('uav_preview.telemetry.monotonic', return_value=now):
            receiver._accept(message, '127.0.0.1', SimpleNamespace())

    def test_nonfinite_ratio_serializes_null_with_explicit_status(self):
        for value in (float('nan'), float('inf'), float('-inf')):
            r = self.receiver()
            self.accept(r, self.message(ratio=value))
            t = r.snapshot()
            self.assertIsNone(t.estimator_position_ratio)
            self.assertEqual(t.estimator_position_ratio_status, 'nonfinite')
            json.dumps(t.to_dict(), allow_nan=False)
            self.assertIn('无法验证', estimator_ratio_reason(t.estimator_position_ratio, '水平位置'))

    def test_missing_and_measured_failure_are_not_conflated(self):
        self.assertIn('不等同检验失败', estimator_ratio_reason(None, '速度'))
        self.assertIn('> 1', estimator_ratio_reason(1.1, '速度'))
        self.assertIn('负值', estimator_ratio_reason(-0.1, '速度'))
        for ratio in (0, 0.1, 1):
            self.assertEqual(estimator_ratio_reason(ratio, '速度'), '')

    def test_duplicate_cannot_refresh_estimator_evidence_or_rate(self):
        r = self.receiver()
        self.accept(r, self.message(), 100)
        self.accept(r, self.message(1200000), 100.2)
        self.assertAlmostEqual(r.snapshot().estimator_hz, 5)
        self.accept(r, self.message(1200000), 101)
        self.assertEqual(r.snapshot().last_estimator_monotonic, 100.2)
        self.assertAlmostEqual(r.snapshot().estimator_hz, 5)

    def test_backwards_source_time_latches_no_automatic_recovery(self):
        r = self.receiver()
        self.accept(r, self.message(1200000), 100)
        self.accept(r, self.message(1100000), 101)
        self.assertIn('源时间倒退', r.snapshot().navigation_fault)
        self.accept(r, self.message(1300000), 102)
        self.assertTrue(r.snapshot().navigation_fault)

    def test_missing_source_timestamp_does_not_refresh_evidence(self):
        r = self.receiver()
        self.accept(r, self.message(None), 100)
        self.assertIsNone(r.snapshot().last_estimator_monotonic)

    def test_missing_ratio_is_not_latched_as_observed_failure_but_strict_gate_still_denies(self):
        r = self.receiver()
        r._snapshot.armed = True
        self.accept(r, self.message(ratio=float('nan')))
        self.assertEqual(r.snapshot().navigation_fault, '')
        self.assertTrue(r.snapshot().estimator_position_ratio_is_nan)
        self.assertTrue(any('水平位置创新比缺失' in s for s in navigation_block_reasons(r.snapshot(),100)))
        self.accept(r, self.message(1200000,ratio=1.1),100.2)
        self.assertIn('> 1', r.snapshot().navigation_fault)
        self.accept(r, self.message(1400000,ratio=.1),100.4)
        self.assertIn('> 1', r.snapshot().navigation_fault)  # actual fault remains latched

    def test_velocity_ratio_single_packet_never_latches(self):
        r = self.receiver()
        r._snapshot.armed = True
        self.accept(r, self.message(1000000, velocity_ratio=1.50), 100.0)
        t = r.snapshot()
        self.assertEqual(t.navigation_fault, '')
        self.assertEqual(t.estimator_velocity_ratio_bad_samples, 1)
        self.assertFalse(t.estimator_velocity_ratio_confirmed_bad)

    def test_velocity_ratio_requires_short_consecutive_distinct_samples(self):
        r = self.receiver()
        r._snapshot.armed = True
        self.accept(r, self.message(1000000, velocity_ratio=1.50), 100.0)
        # Duplicate source timestamps cannot confirm an excursion.
        self.accept(r, self.message(1000000, velocity_ratio=1.60), 100.3)
        self.assertEqual(r.snapshot().estimator_velocity_ratio_bad_samples, 1)
        self.assertEqual(r.snapshot().navigation_fault, '')
        self.accept(r, self.message(1200000, velocity_ratio=1.30), 100.3)
        t = r.snapshot()
        self.assertTrue(t.estimator_velocity_ratio_confirmed_bad)
        self.assertIn('> 1', t.navigation_fault)

    def test_velocity_ratio_healthy_sample_resets_pending_excursion(self):
        r = self.receiver()
        r._snapshot.armed = True
        self.accept(r, self.message(1000000, velocity_ratio=1.50), 100.0)
        self.accept(r, self.message(1200000, velocity_ratio=0.80), 100.3)
        t = r.snapshot()
        self.assertEqual(t.estimator_velocity_ratio_bad_samples, 0)
        self.assertFalse(t.estimator_velocity_ratio_confirmed_bad)
        self.assertEqual(t.navigation_fault, '')
        self.accept(r, self.message(1400000, velocity_ratio=1.40), 100.6)
        self.assertEqual(r.snapshot().estimator_velocity_ratio_bad_samples, 1)
        self.assertEqual(r.snapshot().navigation_fault, '')

    def test_primary_ekf_change_status_is_immediate_even_without_ratio_confirmation(self):
        r = self.receiver()
        r._snapshot.armed = True
        message = SimpleNamespace(
            get_type=lambda: 'STATUSTEXT',
            get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1,
            text=b'Primary EKF changed 0 (filter fault)\0',
            severity=4,
        )
        self.accept(r, message, 100.0)
        self.assertIn('Primary EKF changed', r.snapshot().navigation_fault)

    def test_all_missing_evidence_visible_even_with_other_failure(self):
        t = TelemetrySnapshot()
        reasons = navigation_block_reasons(t, 100) + handoff_evidence_reasons(t, 100)
        for word in ('心跳', '水平位置', '重置计数', '光流融合'):
            self.assertTrue(any(word in reason for reason in reasons), word)
        self.assertIsNone(t.flow_fusion_active)
        self.assertIsNone(t.estimator_reset_signature)

    def test_bad_request_rate_config_rejected(self):
        for value in ('nan', 'inf', '-1', '21'):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'test.toml'
                path.write_text('[position_stream]\nestimator_hz = ' + value, encoding='utf-8')
                with self.assertRaises(ValueError):
                    load_config(path)


if __name__ == '__main__':
    unittest.main()
