"""Offline fixed-hover baseline: fake telemetry/clock/socket/actions only."""
import math
from pathlib import Path
import tempfile
import unittest

import test_local_takeoff as fixtures
from uav_preview.config import load_config
from uav_preview.fixed_hover import fixed_hover_health
from uav_preview.local_takeoff import LOCAL_POSITION_TYPE_MASK


class FixedHoverTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.LocalOffboardTakeoffTests()
        self.f.keyboard_handoff_enabled = False
        self.f.navigation_profile = "px4_fixed_hover"
        self.f.setUp()
        self.c, self.t = self.f.coordinator, self.f.telemetry
        self.t.flow_source = ""
        self.t.flow_quality = None
        self.t.flow_good_since_monotonic = None
        self.t.flow_good_samples = 0
        self.t.flow_fusion_active = None
        self.t.flow_innovation_rejected = None
        self.t.flow_fusion_instance = None
        self.t.estimator_dead_reckoning = None
        self.t.estimator_reset_signature = None
        self.t.flow_innovation_x_ratio = self.t.flow_innovation_y_ratio = None
        self.t.estimator_position_ratio = None
        self.t.estimator_position_ratio_status = 'nonfinite'
        self.t.estimator_position_ratio_is_nan = True

    def tick(self, **changes):
        self.f._advance(.1)
        for field, value in changes.items():
            setattr(self.t, field, value)
        self.c.poll_once()

    def reach_hold(self):
        self.f._reach_arming()
        self.assertEqual(self.f.actions.arm_calls, 1)
        self.tick(armed=True, last_command_ack_command=400, last_command_ack_result=0,
                  last_command_ack_monotonic=self.f.clock.now)
        self.assertEqual(self.c.snapshot()['phase'], 'CLIMBING')
        # Move synthetic telemetry gradually; never teleport to the target.
        for i in range(1, 55):
            height = min(1.5, i*.05)
            self.tick(local_z_m=.25-height, landed_state='IN_AIR', vz_m_s=-.5 if height<1.5 else 0.)
        self.assertEqual(self.c.snapshot()['phase'], 'HOLDING')
        self.assertTrue(self.c.snapshot()['active'])

    def test_missing_optional_downlink_is_visible_not_fake_sensor_success(self):
        self.t.last_laser_monotonic = None
        self.t.last_flow_monotonic = None
        self.t.last_flow_fusion_monotonic = None
        blockers, warnings = fixed_hover_health(self.t, self.f.clock.now)
        self.assertEqual(blockers, [])
        for phrase in ('水平位置创新比未提供', '光流质量未验证', '详细光流融合', '完整重置计数', '对地测距未更新'):
            self.assertTrue(any(phrase in w for w in warnings),phrase)
        self.assertIsNone(self.t.flow_quality)
        self.assertIsNone(self.t.flow_fusion_active)
        self.assertIsNone(self.t.estimator_reset_signature)

    def test_complete_sequence_uses_only_original_position_and_never_handoffs(self):
        self.reach_hold()
        self.f._elapse(5)
        state = self.c.snapshot()
        self.assertIsNone(state['handoff'])
        self.assertFalse(state['keyboard_handoff_enabled'])
        self.assertEqual(state['setpoint_type'], 'POSITION_ONLY')
        self.assertEqual(state['phase'], 'HOLDING')
        messages=[self.f._decode(packet) for packet,_ in self.f.sock.sent]
        for i,m in enumerate(messages):
            self.assertEqual(m.type_mask, LOCAL_POSITION_TYPE_MASK)
            self.assertEqual(m.coordinate_frame,1)
            self.assertEqual(m.get_seq(),i%256)
            self.assertEqual((m.x,m.y,m.vx,m.vy,m.vz),(4.,-2.,0.,0.,0.))
            self.assertAlmostEqual(m.yaw, math.radians(30),places=5)
            self.assertTrue(-1.25001 <= m.z <= .25001)
        self.assertAlmostEqual(messages[-1].z,-1.25)
        self.assertEqual((self.f.actions.arm_calls,self.f.actions.land_calls,self.f.actions.disarm_calls),(1,0,0))
        with self.assertRaisesRegex(ValueError,'键盘交接已关闭'):
            self.c.authorize_keyboard('any','any')

    def test_two_point_five_hz_position_input_can_support_ten_hz_hold_output(self):
        self.reach_hold()
        stamp=self.t.local_position_sample_id
        received=self.t.last_distinct_position_monotonic
        before=len(self.f.sock.sent)
        for i in range(60):
            self.f._advance(.1)
            if i%4==0:
                stamp=self.t.local_position_sample_id;received=self.f.clock.now
            self.t.local_position_sample_id=stamp
            self.t.last_distinct_position_monotonic=received
            self.t.last_local_position_monotonic=received
            self.c.poll_once()
        self.assertTrue(self.c.snapshot()['active'])
        self.assertEqual(len(self.f.sock.sent)-before,60)

    def test_bad_required_evidence_still_denies_start(self):
        for field,value in (
            ('estimator_flags',15), ('estimator_flags',15|32|128), ('estimator_flags',32|256),
            ('last_estimator_monotonic',98.), ('last_heartbeat_monotonic',98.),
            ('local_x_m',math.nan), ('vz_m_s',math.inf), ('last_distinct_position_monotonic',98.),
            ('estimator_velocity_ratio',None), ('estimator_velocity_ratio',1.01),
            ('estimator_position_ratio',1.1), ('estimator_position_ratio_is_nan',False),
            ('rc_channel_6_pwm',None), ('navigation_fault','real fault'),
        ):
            with self.subTest(field=field,value=value):
                self.setUp();setattr(self.t,field,value)
                with self.assertRaises(ValueError):self.c.begin(1.5)
                self.assertEqual(self.f.sock.sent,[])
                self.assertEqual(self.f.actions.arm_calls,0)

    def test_optional_observed_failure_is_not_hidden(self):
        for changes in (
            dict(flow_source='1/1/OPTICAL_FLOW_RAD/0',flow_quality=0),
            dict(flow_error='source backwards'),dict(flow_fusion_active=False),
            dict(flow_innovation_rejected=True),dict(estimator_dead_reckoning=True),
            dict(flow_innovation_x_ratio=17.3),dict(laser_height_m=99),
        ):
            with self.subTest(changes=changes):
                self.setUp();self.reach_hold();before=len(self.f.sock.sent)
                self.tick(**changes)
                self.assertFalse(self.c.snapshot()['active'])
                self.assertEqual(len(self.f.sock.sent),before)
                self.f._elapse(.5)
                self.assertEqual(len(self.f.sock.sent),before)

    def test_coordinate_heading_and_source_changes_invalidate_original_target(self):
        for changes in (
            dict(local_y_m=-1.75),dict(local_z_m=-1.5),dict(yaw_deg=50),
            dict(estimator_reset_signature=(0,1,1,1,1,1)),
            dict(component_id=2),dict(local_position_sample_id=1),
        ):
            with self.subTest(changes=changes):
                self.setUp();self.reach_hold();before=len(self.f.sock.sent)
                self.tick(**changes)
                self.assertFalse(self.c.snapshot()['active'])
                self.assertEqual(len(self.f.sock.sent),before)

    def test_small_drift_never_recaptures_hover_anchor(self):
        self.reach_hold()
        self.tick(local_y_m=-1.9)
        self.assertTrue(self.c.snapshot()['active'])
        self.assertEqual(self.f._decode(self.f.sock.sent[-1][0]).y,-2.)

    def test_stall_stops_instead_of_catching_up_climb(self):
        self.c.begin(1.5);self.c.poll_once();before=len(self.f.sock.sent)
        self.f._advance(.26);self.c.poll_once()
        self.assertFalse(self.c.snapshot()['active'])
        self.assertEqual(len(self.f.sock.sent),before)

    def test_rc_takeover_and_latch_stop_without_duplicate_actions(self):
        for mode in ('POSCTL','AUTO.LAND','AUTO.RTL','KILL','LATCH'):
            with self.subTest(mode=mode):
                self.setUp();self.reach_hold();before=len(self.f.sock.sent)
                changes={}
                if mode=='KILL':changes['rc_channel_6_pwm']=2000
                elif mode=='LATCH':self.f.estop_latched=True
                else:changes.update(flight_mode=mode,rc_channel_8_pwm=1000)
                self.tick(**changes)
                self.assertFalse(self.c.snapshot()['active'])
                self.assertEqual(len(self.f.sock.sent),before)
                self.assertEqual(self.f.actions.land_calls,0)

    def test_land_request_stops_hold_before_future_ticks(self):
        self.reach_hold();before=len(self.f.sock.sent)
        self.c.abort()
        self.f._elapse(1.)
        self.assertEqual(self.f.actions.land_calls,1)
        self.assertEqual(len(self.f.sock.sent),before)

    def test_ground_motion_cancels_before_arm(self):
        self.c.begin(1.5)
        self.tick(vx_m_s=.2)
        self.assertFalse(self.c.snapshot()['active'])
        self.assertEqual(self.f.actions.arm_calls,0)

    def test_profile_cannot_be_combined_with_keyboard_handoff(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'test.toml'
            path.write_text('[local_offboard_takeoff]\nnavigation_profile="px4_fixed_hover"\nkeyboard_handoff_enabled=true\n',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'forbids keyboard'):load_config(path)


if __name__=='__main__':unittest.main()
