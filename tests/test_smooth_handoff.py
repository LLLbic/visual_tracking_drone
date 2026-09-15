"""Offline only: synthetic navigation evidence, fake clocks, captured packets."""
import math
import unittest
from dataclasses import replace

from uav_preview.smooth_handoff import (
    ContinuousReference, HandoffFault, HandoffLimits, SmoothHandoff,
    detailed_health_reason,
)
from uav_preview.types import TelemetrySnapshot


ZERO = (0.0,)*4
CLIENT = "test-browser-session-A"


def fresh(t, now):
    for name in (
        "last_packet_monotonic", "last_heartbeat_monotonic", "last_rc_channels_monotonic",
        "last_attitude_monotonic", "last_local_position_monotonic", "last_extended_state_monotonic",
        "last_estimator_monotonic", "last_flow_monotonic", "last_laser_monotonic",
        "last_flow_fusion_monotonic", "last_reset_evidence_monotonic",
        "last_distinct_position_monotonic",
    ):
        setattr(t,name,now)
    t.local_position_sample_id=round(now*1000)
    return t


def healthy(now=100.0):
    return fresh(TelemetrySnapshot(
        connected=True, armed=True, flight_mode="OFFBOARD", landed_state="IN_AIR",
        local_x_m=4.0,local_y_m=-2.0,local_z_m=-1.25,
        vx_m_s=0.0,vy_m_s=0.0,vz_m_s=0.0,roll_deg=0.0,pitch_deg=0.0,yaw_deg=0.0,
        flow_quality=240,flow_quality_authoritative=True,
        estimator_flags=15|32,estimator_velocity_ratio=0.1,estimator_position_ratio=0.1,
        flow_source='1/1/OPTICAL_FLOW_RAD/0',flow_good_since_monotonic=now-10.,flow_good_samples=100,
        laser_height_m=1.5,laser_min_m=0.1,laser_max_m=5.0,
        rc_channel_6_pwm=1000,rc_channel_8_pwm=2000,
        flow_fusion_active=True,flow_innovation_rejected=False,flow_fusion_instance=0,
        flow_innovation_x_ratio=.1,flow_innovation_y_ratio=.1,estimator_dead_reckoning=False,
        estimator_reset_signature=(0,0,0,0,0,0),
    ),now)


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.now=100.0
        self.t=healthy(self.now)
        self.h=SmoothHandoff((4.0,-2.0,-1.25,0.0),0.25,(4.0,-2.0),1.0,
                             frame_signature=self.t.estimator_reset_signature)
        self.seq=0

    def report(self,axes=ZERO,**kwargs):
        self.seq+=1
        args=dict(run_id=self.h.run_id,client=CLIENT,sequence=self.seq,axes=axes,
                  foreground=True,confirmed=True,token=self.h.token,now=self.now,
                  keys_released=not any(axes))
        args.update(kwargs)
        self.h.report(**args)

    def tick(self,dt=.1,axes=ZERO):
        self.now+=dt
        fresh(self.t,self.now)
        self.report(axes)
        return self.h.step(self.t,self.now)

    def ready(self):
        for _ in range(33):
            self.tick()
        self.assertTrue(self.h.ready(self.now))

    def arm_permission(self):
        self.ready()
        return self.h.authorize(CLIENT,self.t,self.now)

    def test_three_seconds_is_permission_not_automatic_tx(self):
        self.ready()
        self.assertFalse(self.h.authorized)
        self.assertIsNone(self.h.reference)
        goal=self.h.step(self.t,self.now)
        token=self.h.authorize(CLIENT,self.t,self.now)
        self.assertEqual(self.h.step(self.t,self.now),goal)
        self.assertNotIn(token,str(self.h.snapshot(self.now)))
        self.assertTrue(self.h.awaiting_neutral)

    def test_every_required_health_field_fails_closed(self):
        cases={
            "flow_fusion_active":[None,False],"flow_innovation_rejected":[None,True],
            "flow_fusion_instance":[None,1],"last_flow_fusion_monotonic":[None,99.0,101.0],
            "flow_innovation_x_ratio":[None,float('nan'),17.3],"flow_innovation_y_ratio":[None,1.01],
            "estimator_dead_reckoning":[None,True],
            "estimator_reset_signature":[None,(0,0),(True,0,0,0,0,0)],
            "last_reset_evidence_monotonic":[None,99.0,101.0],
            "flow_quality":[0,4,99,None],"estimator_flags":[0,15,15|32|1024],
            "flow_good_since_monotonic":[None,99.,101.],"flow_good_samples":[0,14],
            "flow_source":[''],
            "local_position_sample_id":[None,True,-1],"last_distinct_position_monotonic":[None,99.0],
            "estimator_velocity_ratio":[None],"estimator_position_ratio":[float('nan')],
            "local_x_m":[None,float('inf')],"vx_m_s":[float('nan')],
        }
        for field,values in cases.items():
            for value in values:
                with self.subTest(field=field,value=value):
                    t=replace(self.t,**{field:value})
                    self.assertTrue(detailed_health_reason(t,self.now))
        t = replace(
            self.t,
            estimator_velocity_ratio=1.01,
            estimator_velocity_ratio_bad_samples=2,
            estimator_velocity_ratio_confirmed_bad=True,
        )
        self.assertTrue(detailed_health_reason(t,self.now))
        self.assertEqual(detailed_health_reason(self.t,self.now),'')

    def test_position_speed_tilt_and_actual_mode_must_remain_stable(self):
        for field,value in [('local_x_m',4.16),('local_z_m',-1.41),('vx_m_s',.081),
                            ('vz_m_s',.101),('roll_deg',5.1),('flight_mode','POSCTL'),
                            ('landed_state','ON_GROUND'),('armed',False),('rc_channel_6_pwm',2000),
                            ('rc_channel_8_pwm',1500)]:
            with self.subTest(field=field):
                self.setUp();self.ready()
                setattr(self.t,field,value)
                self.h.observe(self.t,self.now)
                self.assertFalse(self.h.ready(self.now))
                with self.assertRaises(ValueError):self.h.authorize(CLIENT,self.t,self.now)

    def test_frozen_position_cannot_pass_with_fresh_other_telemetry(self):
        for _ in range(35):
            self.now+=.1;fresh(self.t,self.now)
            self.t.last_local_position_monotonic=100.0
            self.report();self.h.observe(self.t,self.now)
        self.assertFalse(self.h.ready(self.now))

    def test_repeated_source_sample_does_not_count_as_distinct_observations(self):
        for _ in range(35):
            self.now+=.1;fresh(self.t,self.now)
            self.t.local_position_sample_id=100000
            self.report();self.h.observe(self.t,self.now)
        self.assertEqual(self.h.samples,1)
        self.assertFalse(self.h.ready(self.now))

    def test_opposing_held_keys_are_not_released(self):
        self.ready()
        self.report(keys_released=False)
        self.h.observe(self.t,self.now)
        self.assertFalse(self.h.ready(self.now))

    def test_browser_foreground_confirmation_and_neutral_gate(self):
        for arg in ('foreground','confirmed','keys_released'):
            with self.subTest(arg=arg):
                self.setUp();self.ready();self.report(**{arg:False})
                with self.assertRaises(ValueError):self.h.authorize(CLIENT,self.t,self.now)
        self.setUp();self.arm_permission()
        with self.assertRaises(ValueError):self.report((1,0,0,0))
        self.assertEqual(self.h.axes,ZERO)
        self.report();self.tick(axes=(1,0,0,0))
        self.assertGreater(self.h.reference.velocity[0],0)

    def test_session_sequence_run_token_and_repeat_authorize(self):
        self.arm_permission()
        for overrides in ({'run_id':'old-task'},{'client':'other-tab'}, {'sequence':0},
                          {'token':'old-token'},{'sequence':True}):
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):self.report(**overrides)
        with self.assertRaises(ValueError):self.h.authorize(CLIENT,self.t,self.now)
        self.assertEqual(self.h.axes,ZERO)

    def test_focus_loss_brakes_without_reacquiring_or_recapture(self):
        self.arm_permission();self.report()
        for _ in range(5):self.tick(axes=(1,0,0,0))
        token=self.h.token
        self.report(foreground=False)
        self.assertFalse(self.h.authorized)
        for _ in range(40):self.tick()
        self.assertEqual(self.h.reference.velocity,(0.,0.,0.))
        target=self.h.reference.position
        self.t.local_y_m+=.08
        for _ in range(5):self.tick()
        self.assertEqual(self.h.reference.position,target)
        self.assertFalse(self.h.authorized)
        new=self.h.authorize(CLIENT,self.t,self.now)
        self.assertNotEqual(new,token)
        with self.assertRaises(ValueError):self.report(token=token)

    def test_watchdog_drops_permission_but_keeps_same_reference(self):
        self.arm_permission();self.report();self.tick(axes=(1,0,0,0))
        reference=self.h.reference
        for _ in range(5):
            self.now+=.1;fresh(self.t,self.now);self.h.step(self.t,self.now)
        self.assertFalse(self.h.authorized)
        self.assertIs(self.h.reference,reference)
        self.report();self.h.step(self.t,self.now)
        self.assertFalse(self.h.authorized)

    def test_reset_before_or_after_authorize_invalidates_entire_task(self):
        for authorized in (False,True):
            for index in range(6):
                with self.subTest(authorized=authorized,index=index):
                    self.setUp();self.ready()
                    if authorized:self.h.authorize(CLIENT,self.t,self.now)
                    counts=list(self.t.estimator_reset_signature);counts[index]+=1
                    self.t.estimator_reset_signature=tuple(counts)
                    with self.assertRaises(HandoffFault):self.h.step(self.t,self.now)
                    self.t.estimator_reset_signature=(0,0,0,0,0,0)
                    with self.assertRaises(ValueError):self.h.authorize(CLIENT,self.t,self.now)
                    self.assertTrue(self.h.terminal)

    def test_health_loss_and_tick_stall_after_authorization_are_terminal(self):
        for kind in ('flow','reset_stale','gap','backward_time'):
            with self.subTest(kind=kind):
                self.setUp();self.arm_permission();self.report()
                if kind=='flow':self.t.flow_fusion_active=False
                if kind=='reset_stale':self.t.last_reset_evidence_monotonic=None
                if kind=='gap':self.now+=.26;fresh(self.t,self.now)
                if kind=='backward_time':self.now-=.01;fresh(self.t,self.now)
                with self.assertRaises(HandoffFault):self.h.step(self.t,self.now)
                self.assertTrue(self.h.terminal)


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.t=healthy(0.0)
        self.p=ContinuousReference((4.,-2.,-1.25,0.),.25,(4.,-2.),1.,HandoffLimits())
        self.p.step(ZERO,self.t,0.0)

    def follow(self):
        self.t.local_x_m,self.t.local_y_m,self.t.local_z_m=self.p.position

    def test_position_and_velocity_continuity_diagonal_limit_and_frame(self):
        self.t.yaw_deg=90
        self.p.yaw=math.radians(90)
        previous_position=self.p.position;previous_velocity=self.p.velocity
        for i in range(1,21):
            self.follow();self.p.step((1,1,0,1),self.t,i*.05)
            v=self.p.velocity
            self.assertLessEqual(math.hypot(*v[:2]),.25+1e-9)
            self.assertLessEqual(math.hypot(v[0]-previous_velocity[0],v[1]-previous_velocity[1]),.4*.05+1e-9)
            for p0,p1,v0,v1 in zip(previous_position,self.p.position,previous_velocity,v):
                self.assertAlmostEqual(p1-p0,(v0+v1)*.05/2)
            previous_position,previous_velocity=self.p.position,v
        self.assertLess(self.p.velocity[0],0);self.assertGreater(self.p.velocity[1],0)
        self.assertGreater(self.p.yaw,0)
        self.assertLessEqual(abs(self.p.yaw_rate),math.radians(10))

    def test_aircraft_not_following_cannot_accumulate_target(self):
        for i in range(1,1500):self.p.step((1,0,0,0),self.t,i*.05)
        self.assertLessEqual(self.p.position[0]-4.,.22)
        self.assertEqual(self.p.velocity,(0.,0.,0.))

    def test_key_release_decelerates_then_holds_reference_endpoint(self):
        for i in range(1,11):self.follow();self.p.step((1,0,0,0),self.t,i*.05)
        speed=self.p.velocity[0]
        self.follow();self.p.step(ZERO,self.t,.55)
        self.assertGreater(self.p.velocity[0],0);self.assertLess(self.p.velocity[0],speed)
        for i in range(12,31):self.follow();self.p.step(ZERO,self.t,i*.05)
        goal=self.p.position
        self.t.local_y_m+=.1
        self.p.step(ZERO,self.t,1.55)
        self.assertEqual(self.p.position,goal)
        self.assertEqual(self.p.phase,'POSITION_HOLD')

    def test_geofence_and_height_brake_before_boundary(self):
        for axes in ((1,0,0,0),(0,0,1,0),(0,0,-1,0)):
            with self.subTest(axes=axes):
                self.setUp()
                for i in range(1,601):
                    self.follow();self.p.step(axes,self.t,i*.05)
                    self.assertLessEqual(math.hypot(self.p.position[0]-4,self.p.position[1]+2),1.0)
                    self.assertTrue(1.<=.25-self.p.position[2]<=3.)
                self.assertEqual(self.p.velocity,(0.,0.,0.))

    def test_large_error_and_time_jump_never_emit_catchup_target(self):
        before=self.p.position
        self.t.local_y_m+=.46
        with self.assertRaises(HandoffFault):self.p.step((1,0,0,0),self.t,.1)
        self.assertEqual(self.p.position,before)
        self.setUp()
        with self.assertRaises(HandoffFault):self.p.step((1,0,0,0),self.t,2.)
        self.assertEqual(self.p.position,before)

    def test_can_retreat_from_geofence_instead_of_sticking_at_edge(self):
        for i in range(1,121):self.follow();self.p.step((1,0,0,0),self.t,i*.05)
        before=self.p.position[0]
        for i in range(121,141):self.follow();self.p.step((-1,0,0,0),self.t,i*.05)
        self.assertLess(self.p.position[0],before-.05)

    def test_vertical_and_yaw_targets_stop_accumulating_if_not_followed(self):
        for i in range(1,301):self.p.step((0,0,1,1),self.t,i*.05)
        self.assertLess(abs(self.p.position[2]-self.t.local_z_m),.2)
        self.assertLess(abs(self.p.yaw),math.radians(15.))
        self.assertEqual(self.p.velocity[2],0.)
        self.assertEqual(self.p.yaw_rate,0.)


if __name__=='__main__':unittest.main()
