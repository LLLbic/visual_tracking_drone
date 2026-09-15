"""Takeoff/hover-only regressions: fake clock/socket/actions; never contact hardware."""
import unittest
from threading import RLock
from unittest.mock import Mock, patch

from fastapi.testclient import TestClient

import test_local_takeoff as fixtures
from uav_preview.runtime import Runtime
from uav_preview.server import create_app
from uav_preview.smooth_handoff import detailed_health_reason, position_control_health_reason


class TakeoffOnlyTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.LocalOffboardTakeoffTests()
        self.f.keyboard_handoff_enabled = False
        self.f.setUp()
        self.c = self.f.coordinator

    def test_missing_handoff_window_does_not_block_valid_takeoff_or_hold(self):
        t = self.f.telemetry
        t.flow_good_since_monotonic = None
        t.flow_good_samples = 1
        self.assertEqual(position_control_health_reason(t, self.f.clock.now), "")
        self.assertIn("15个不同样本", detailed_health_reason(t, self.f.clock.now))
        with patch("uav_preview.local_takeoff.SmoothHandoff.step", side_effect=AssertionError("handoff must not run")):
            self.f.test_reaching_height_keeps_hold_stream_active()
            original = self.f._decode(self.f.sock.sent[-1][0])
            # Small position error must never redefine the captured hover anchor.
            t.local_y_m += 0.1
            self.f._elapse(4.0)  # no browser input; no automatic 3-second handoff
        s = self.c.snapshot()
        self.assertEqual(s["control_scope"], "TAKEOFF_HOLD_ONLY")
        self.assertFalse(s["keyboard_handoff_enabled"])
        self.assertIsNone(s["handoff"])
        self.assertTrue(s["active"])
        self.assertEqual(s["phase"], "HOLDING")
        self.assertIn("键盘交接已关闭", s["last_reason"])
        current = self.f._decode(self.f.sock.sent[-1][0])
        for field in ("x", "y", "z", "yaw", "type_mask", "coordinate_frame"):
            self.assertEqual(getattr(current, field), getattr(original, field))
        self.assertEqual((current.vx, current.vy, current.vz), (0., 0., 0.))
        self.assertIsNone(self.c._handoff.reference)
        self.assertEqual(self.f.actions.arm_calls, 1)
        self.assertEqual(self.f.actions.land_calls, 0)

    def test_direct_keyboard_calls_are_rejected_and_config_cannot_hot_enable(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        self.c.config.keyboard_handoff_enabled = True  # fixed at service construction
        run = self.c._handoff.run_id
        count = len(self.f.sock.sent)
        for operation in (
            lambda: self.c.report_keyboard(run, "client", 1, (1., 0., 0., 0.), True, True, False),
            lambda: self.c.authorize_keyboard(run, "client"),
            lambda: self.c.revoke_keyboard(run, "client", "token"),
        ):
            with self.assertRaisesRegex(ValueError, "键盘交接已关闭"):
                operation()
        self.assertEqual(len(self.f.sock.sent), count)
        self.assertFalse(self.c.snapshot()["keyboard_handoff_enabled"])
        self.assertFalse(self.c._handoff.authorized)

    def test_http_handoff_endpoints_cannot_bypass_disabled_mode(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        runtime = Mock()
        runtime.report_handoff_input.side_effect = self.c.report_keyboard
        runtime.authorize_handoff.side_effect = self.c.authorize_keyboard
        runtime.revoke_handoff.side_effect = self.c.revoke_keyboard
        with patch("uav_preview.server.Runtime", return_value=runtime):
            app = create_app(Mock())
        client = TestClient(app)  # No lifespan: no worker, socket or camera started.
        identity = {"run_id": self.c._handoff.run_id, "client_id": "test-browser-session-A"}
        count = len(self.f.sock.sent)
        try:
            for action, body in (
                ("input", {**identity, "sequence": 1, "pitch": 1, "roll": 0, "throttle": 0, "yaw": 0,
                           "foreground": True, "confirmed": True, "keys_released": False}),
                ("authorize", {**identity, "confirmation": "HANDOFF"}),
                ("revoke", {**identity, "token": "x" * 24}),
            ):
                response = client.post("/api/local-takeoff/keyboard/" + action, json=body)
                self.assertEqual(response.status_code, 409)
                self.assertIn("键盘交接已关闭", response.text)
        finally:
            client.close()
        runtime.start.assert_not_called()
        self.assertEqual(len(self.f.sock.sent), count)

    def test_legacy_velocity_sender_cannot_replace_active_hover(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        runtime = Runtime.__new__(Runtime)  # Do not construct any real dependencies.
        runtime._safety_transition_lock = RLock()
        runtime.takeoff = Mock()
        runtime.takeoff.snapshot.return_value = {"active": False}
        runtime.local_takeoff = self.c
        runtime.keyboard_control = Mock()
        runtime.ground_offboard = Mock()
        with self.assertRaisesRegex(ValueError, "尚不能开启键盘真实TX"):
            runtime.enable_keyboard_control()
        runtime.keyboard_control.enable.assert_not_called()
        runtime.ground_offboard.disable.assert_not_called()

    def test_core_evidence_still_blocks_start_without_packets_or_arm(self):
        for field, value in (
            ("flow_quality", 4), ("flow_source", ""), ("last_flow_monotonic", 99.4),
            ("flow_fusion_active", None), ("flow_innovation_x_ratio", 17.3),
            ("flow_innovation_rejected", True), ("estimator_reset_signature", None),
            ("estimator_dead_reckoning", True), ("last_reset_evidence_monotonic", None),
            ("estimator_position_ratio", None), ("last_laser_monotonic", None),
            ("local_x_m", float("nan")),
        ):
            with self.subTest(field=field):
                self.setUp()
                setattr(self.f.telemetry, field, value)
                with self.assertRaises(ValueError):
                    self.c.begin(1.5)
                self.assertEqual(self.f.sock.sent, [])
                self.assertEqual(self.f.actions.arm_calls, 0)

    def test_reset_during_hold_invalidates_target_without_recapture_or_restart(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        count = len(self.f.sock.sent)
        self.f.telemetry.estimator_reset_signature = (1, 1, 1, 1, 1, 1)
        self.f.telemetry.flow_fusion_instance = 1
        self.f._advance(.1)
        self.c.poll_once()
        self.assertTrue(self.c.snapshot()["active"])
        self.assertEqual(self.c.snapshot()["phase"], "LANDING")
        self.assertIn("重置计数变化", self.c.snapshot()["error"])
        self.f._elapse(1.0)
        self.assertEqual(len(self.f.sock.sent), count)
        self.assertEqual(self.f.actions.land_calls, 1)

    def test_flow_failure_in_hold_does_not_continue_sending(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        count = len(self.f.sock.sent)
        self.f.telemetry.flow_quality = 0
        self.f._advance(.1)
        self.c.poll_once()
        self.assertTrue(self.c.snapshot()["active"])
        self.assertEqual(self.c.snapshot()["phase"], "LANDING")
        self.assertEqual(self.f.actions.land_calls, 1)
        self.assertEqual(len(self.f.sock.sent), count)

    def test_pilot_mode_changes_still_win_over_bad_navigation_evidence(self):
        for mode in ("POSCTL", "AUTO.LAND", "AUTO.RTL"):
            with self.subTest(mode=mode):
                self.setUp()
                self.f.test_reaching_height_keeps_hold_stream_active()
                count = len(self.f.sock.sent)
                self.f.telemetry.flight_mode = mode
                self.f.telemetry.rc_channel_8_pwm = 1000
                self.f.telemetry.flow_fusion_active = None
                self.f._advance(.1)
                self.c.poll_once()
                self.assertEqual(self.c.snapshot()["phase"], "PILOT_TAKEOVER")
                self.assertEqual(len(self.f.sock.sent), count)
                self.assertEqual(self.f.actions.land_calls, 0)
                self.assertEqual(self.f.actions.disarm_calls, 0)

    def test_takeoff_only_hover_keeps_altitude_overshoot_protection(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        self.f.telemetry.local_z_m = -1.9  # +2.15 m from captured +0.25, target +1.5
        self.f._advance(.1)
        self.c.poll_once()
        self.assertTrue(self.c.snapshot()["active"])
        self.assertEqual(self.c.snapshot()["phase"], "LANDING")
        self.assertEqual(self.f.actions.land_calls, 1)


if __name__ == "__main__":
    unittest.main()
