"""Offline position-only keyboard handoff; fake telemetry/socket/actions only."""
import math
import unittest

import test_local_takeoff as fixtures
from uav_preview.local_takeoff import LOCAL_POSITION_TYPE_MASK


CLIENT = "position-keyboard-test-client"


class PositionOnlyKeyboardHandoffTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.LocalOffboardTakeoffTests()
        self.f.keyboard_handoff_enabled = True
        self.f.navigation_profile = "px4_position_keyboard"
        self.f.setUp()
        self.c, self.t = self.f.coordinator, self.f.telemetry
        # This is the deployed adapter condition: the raw value can be a
        # constant placeholder and must not authorize or reject motion.
        self.t.flow_quality_authoritative = False
        self.t.flow_quality = 4
        self.t.flow_error = ""
        self.sequence = 0

    def tick(self, **changes):
        self.f._advance(.1)
        for field, value in changes.items():
            setattr(self.t, field, value)
        self.c.poll_once()

    def reach_hold(self):
        self.f._reach_arming()
        self.tick(armed=True, last_command_ack_command=400, last_command_ack_result=0,
                  last_command_ack_monotonic=self.f.clock.now)
        for i in range(1, 55):
            height = min(1.5, i*.05)
            self.tick(local_z_m=.25-height, landed_state='IN_AIR',
                      vz_m_s=-.5 if height < 1.5 else 0.)
        self.assertEqual(self.c.snapshot()['phase'], 'HOLDING')
        self.assertTrue(self.c.snapshot()['active'])

    def report(self, axes=(0., 0., 0., 0.), token=None, foreground=True):
        self.sequence += 1
        return self.c.report_keyboard(
            self.c.snapshot()['handoff']['run_id'], CLIENT, self.sequence, axes,
            foreground, True, not any(axes), token,
        )

    def make_ready(self):
        self.reach_hold()
        for _ in range(35):
            self.report()
            self.tick()
        state = self.c.snapshot()['handoff']
        self.assertTrue(state['ready'], state)

    def authorize(self):
        self.make_ready()
        run_id = self.c.snapshot()['handoff']['run_id']
        return self.c.authorize_keyboard(run_id, CLIENT)['token']

    def decoded(self):
        return [self.f._decode(packet) for packet, _endpoint in self.f.sock.sent]

    def test_authorization_is_packetless_and_first_packet_matches_hover(self):
        self.make_ready()
        before_count = len(self.f.sock.sent)
        before = self.decoded()[-1]
        token = self.c.authorize_keyboard(self.c.snapshot()['handoff']['run_id'], CLIENT)['token']
        self.assertEqual(len(self.f.sock.sent), before_count)
        self.report(token=token)  # Mandatory neutral report after authorization.
        self.tick()
        after = self.decoded()[-1]
        self.assertEqual(len(self.f.sock.sent), before_count + 1)
        self.assertEqual(after.coordinate_frame, 1)
        self.assertEqual(after.type_mask, LOCAL_POSITION_TYPE_MASK)
        self.assertEqual((after.x, after.y, after.z, after.yaw),
                         (before.x, before.y, before.z, before.yaw))
        self.assertEqual((after.vx, after.vy, after.vz, after.yaw_rate), (0., 0., 0., 0.))

    def test_keyboard_moves_only_continuous_position_and_release_holds(self):
        token = self.authorize()
        self.report(token=token)
        self.tick()
        origin = self.decoded()[-1]
        # At the captured 30-degree heading, W advances in both local N and E.
        for _ in range(5):
            self.report((1., 0., 0., 0.), token)
            self.tick()
        moving = self.decoded()[-1]
        self.assertGreater(moving.x, origin.x)
        self.assertGreater(moving.y, origin.y)
        self.assertEqual(moving.coordinate_frame, origin.coordinate_frame)
        self.assertEqual(moving.type_mask, origin.type_mask)
        self.assertEqual((moving.vx, moving.vy, moving.vz), (0., 0., 0.))
        # Release ramps the internal reference velocity to zero, then holds the
        # final generated position without recapturing the measured position.
        for _ in range(12):
            self.report(token=token)
            self.tick()
        messages = self.decoded()
        self.assertAlmostEqual(messages[-1].x, messages[-2].x, places=6)
        self.assertAlmostEqual(messages[-1].y, messages[-2].y, places=6)
        self.assertEqual(self.c.snapshot()['handoff']['phase'], 'POSITION_HOLD')

    def test_input_timeout_revokes_then_brakes_without_stopping_hover(self):
        token = self.authorize()
        self.report(token=token)
        self.tick()
        self.report((1., 0., 0., 0.), token)
        self.tick()
        self.f._elapse(1.0)  # Browser reports stop; control loop and telemetry continue.
        handoff = self.c.snapshot()['handoff']
        self.assertFalse(handoff['authorized'])
        self.assertEqual(handoff['phase'], 'POSITION_HOLD')
        self.assertTrue(self.c.snapshot()['active'])
        self.assertEqual(self.c.snapshot()['setpoint_type'], 'POSITION_ONLY')

    def test_two_point_five_hz_position_samples_can_become_ready(self):
        self.reach_hold()
        sample = self.t.local_position_sample_id
        received = self.t.last_distinct_position_monotonic
        for index in range(64):
            self.report()
            self.f._advance(.1)
            if index % 4:
                self.t.local_position_sample_id = sample
                self.t.last_distinct_position_monotonic = received
                self.t.last_local_position_monotonic = received
            else:
                sample = self.t.local_position_sample_id
                received = self.t.last_distinct_position_monotonic
            self.c.poll_once()
        self.assertTrue(self.c.snapshot()['handoff']['ready'])
        self.assertTrue(self.c.snapshot()['active'])

    def test_pilot_mode_switch_stops_same_sender_and_never_restarts(self):
        token = self.authorize()
        self.report(token=token)
        self.tick()
        before = len(self.f.sock.sent)
        self.tick(flight_mode='POSCTL', rc_channel_8_pwm=1000)
        self.assertFalse(self.c.snapshot()['active'])
        self.assertEqual(self.c.snapshot()['phase'], 'PILOT_TAKEOVER')
        self.f._elapse(1.0)
        self.assertEqual(len(self.f.sock.sent), before)

    def test_coordinate_reset_invalidates_reference_before_next_packet(self):
        token = self.authorize()
        self.report(token=token)
        self.tick()
        before = len(self.f.sock.sent)
        signature = list(self.t.estimator_reset_signature)
        signature[1] += 1
        self.tick(estimator_reset_signature=tuple(signature))
        state = self.c.snapshot()
        self.assertTrue(state['active'])
        self.assertEqual(state['phase'], 'LANDING')
        self.assertEqual(self.f.actions.land_calls, 1)
        self.assertEqual(len(self.f.sock.sent), before)
        self.assertIn('重置', state['last_reason'])

    def test_position_jump_stops_without_recapturing_drifted_pose(self):
        token = self.authorize()
        self.report(token=token)
        self.tick()
        before = len(self.f.sock.sent)
        self.tick(local_x_m=self.t.local_x_m + .35)
        state = self.c.snapshot()
        self.assertTrue(state['active'])
        self.assertEqual(state['phase'], 'LANDING')
        self.assertEqual(self.f.actions.land_calls, 1)
        self.assertEqual(len(self.f.sock.sent), before)
        self.assertIn('坐标', state['last_reason'])

    def test_all_packets_keep_one_position_only_protocol(self):
        token = self.authorize()
        self.report(token=token)
        self.tick()
        for axes in ((1.,0.,0.,0.), (0.,1.,0.,0.), (0.,0.,1.,0.), (0.,0.,0.,1.), (0.,0.,0.,0.)):
            self.report(axes, token)
            self.tick()
        messages = self.decoded()
        self.assertGreater(len(messages), 20)
        self.assertTrue(all(message.coordinate_frame == 1 for message in messages))
        self.assertTrue(all(message.type_mask == LOCAL_POSITION_TYPE_MASK for message in messages))
        self.assertTrue(all((message.vx, message.vy, message.vz, message.yaw_rate) == (0.,0.,0.,0.)
                            for message in messages))
        self.assertEqual([message.get_seq() for message in messages],
                         [index % 256 for index in range(len(messages))])


if __name__ == '__main__':
    unittest.main()
