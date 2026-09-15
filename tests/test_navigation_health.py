import unittest
from uav_preview.types import TelemetrySnapshot
from uav_preview.navigation_health import navigation_block_reason
from uav_preview.motion_plan import KeyboardMotionPlan


class NavigationTests(unittest.TestCase):
    def healthy(self):
        t = TelemetrySnapshot(flow_quality=245, flow_quality_authoritative=True, estimator_flags=15,
            estimator_velocity_ratio=0.1, estimator_position_ratio=0.1,
            laser_height_m=1.5, laser_min_m=0.1, laser_max_m=3.0,
            rc_channel_6_pwm=1000, rc_channel_8_pwm=2000)
        for field in ('last_heartbeat_monotonic','last_rc_channels_monotonic','last_attitude_monotonic',
                      'last_local_position_monotonic','last_extended_state_monotonic','last_estimator_monotonic',
                      'last_flow_monotonic','last_laser_monotonic'):
            setattr(t, field, 100.0)
        for field in ('local_x_m','local_y_m','local_z_m','vx_m_s','vy_m_s','vz_m_s','roll_deg','pitch_deg','yaw_deg'):
            setattr(t, field, 0.0)
        return t

    def test_missing_or_invalid_health_blocks(self):
        self.assertTrue(navigation_block_reason(TelemetrySnapshot(), 100))
        self.assertEqual(navigation_block_reason(self.healthy(), 100), '')
        for field, value in [('flow_quality',0),('estimator_flags',0),('estimator_flags',143),
                             ('flow_quality_authoritative',False),
                             ('estimator_position_ratio',float('nan')),
                             ('last_rc_channels_monotonic',98),('last_local_position_monotonic',101),
                             ('laser_height_m',5),('navigation_fault','reset'),('vx_m_s',float('inf'))]:
            with self.subTest(field=field, value=value):
                t=self.healthy(); setattr(t,field,value)
                self.assertTrue(navigation_block_reason(t,100))

    def test_velocity_ratio_needs_confirmed_consecutive_samples(self):
        t = self.healthy()
        t.estimator_velocity_ratio = 1.01
        t.estimator_velocity_ratio_bad_samples = 1
        self.assertEqual(navigation_block_reason(t, 100), '')
        t.estimator_velocity_ratio_bad_samples = 2
        t.estimator_velocity_ratio_confirmed_bad = True
        self.assertIn('> 1', navigation_block_reason(t, 100))

    def test_stationary_capture_and_hold_does_not_follow_drift(self):
        t=self.healthy(); p=KeyboardMotionPlan()
        p.step((0,0,0,0),t,0)
        kind, goal=p.step((0,0,0,0),t,1.1)
        self.assertEqual(kind,'position')
        t.local_y_m=0.15
        self.assertEqual(p.step((0,0,0,0),t,1.2),('position',goal))

    def test_moving_estimate_cannot_capture(self):
        t=self.healthy(); t.vy_m_s=0.12; p=KeyboardMotionPlan()
        for sec in range(10):
            self.assertEqual(p.step((0,0,0,0),t,sec)[0],'velocity')

    def test_velocity_slew_and_release_braking(self):
        t=self.healthy(); p=KeyboardMotionPlan()
        p.step((0,0,0,0),t,0)
        _, goal=p.step((0.25,0,0,0),t,0.1)
        self.assertAlmostEqual(goal[0],0.05)
        _, goal=p.step((0,0,0,0),t,0.2)
        self.assertAlmostEqual(goal[0],0)
        self.assertEqual(p.phase,'BRAKING')

if __name__ == '__main__':
    unittest.main()
