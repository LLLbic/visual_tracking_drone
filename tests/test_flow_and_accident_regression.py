"""Offline regression only. No real vehicle sockets or flight commands."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from pymavlink.dialects.v20 import common as mavlink2

from uav_preview.config import TelemetryConfig
from uav_preview.flow_health import FlowMonitor
from uav_preview.telemetry import PassiveMavlinkReceiver
from uav_preview.smooth_handoff import SmoothHandoff, HandoffFault, detailed_health_reason
from uav_preview.local_takeoff import LOCAL_TRAJECTORY_TYPE_MASK
from test_smooth_handoff import healthy, fresh, CLIENT, ZERO
import test_local_takeoff as local_fixtures
import test_position_stream as rate_fixtures


def flow(stamp, quality=245, sensor=0, kind='OPTICAL_FLOW_RAD', system=1):
    m=SimpleNamespace(time_usec=stamp,quality=quality,sensor_id=sensor,
                      integration_time_us=20000,distance=-1.,integrated_xgyro=math.nan,
                      integrated_ygyro=math.nan,integrated_zgyro=math.nan)
    m.get_type=lambda:kind
    m.get_srcSystem=lambda:system
    m.get_srcComponent=lambda:1
    return m


class FlowEvidenceTests(unittest.TestCase):
    def test_internal_sensor_0_158_is_diagnostic_not_autopilot_state(self):
        r=PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        r._snapshot.system_id=r._snapshot.component_id=1
        m=flow(1000000,system=0)
        m.get_srcComponent=lambda:158
        r._accept(m,'127.0.0.1',object())
        t=r.snapshot()
        self.assertEqual(t.system_id,1)
        self.assertIsNone(t.flow_quality)
        self.assertIsNone(t.flow_fusion_active)
        self.assertEqual(t.flow_sources['0/158/OPTICAL_FLOW_RAD/0']['quality_raw'],245)
        self.assertFalse(t.flow_sources['0/158/OPTICAL_FLOW_RAD/0']['selected'])

    def test_console_245_is_raw_uint8_not_percent_and_no_distance_not_zero(self):
        f=FlowMonitor(quality_authoritative=True)
        m=flow(1000000)
        sample,error=f.accept(m,100.)
        self.assertEqual(sample.quality,245)
        self.assertEqual(error,'')
        self.assertEqual(sample.integration_us,20000)
        self.assertIsNone(sample.ground_distance_m)
        self.assertEqual(f.diagnostics(100.)[sample.source]['quality_raw'],245)
        # Missing gyro integration is not reinterpreted as bad optical quality.
        self.assertEqual(f.good_samples,1)

    def test_actual_mavlink_encoding_decodes_quality_without_field_mixup(self):
        encoder=mavlink2.MAVLink(None,srcSystem=1,srcComponent=1)
        msg=encoder.optical_flow_rad_encode(13595537616,0,20000,-.0003,.0006,
                math.nan,math.nan,math.nan,0,245,33333,-1.)
        parsed=mavlink2.MAVLink(None).parse_char(msg.pack(encoder))
        sample,error=FlowMonitor(quality_authoritative=True).accept(parsed,100.)
        self.assertEqual(sample.quality,245)
        self.assertEqual(sample.sensor_id,0)
        self.assertEqual(error,'')

    def test_other_source_cannot_replace_bad_selected_source_by_higher_quality(self):
        f=FlowMonitor(quality_authoritative=True)
        f.accept(flow(1000,4),100.)
        self.assertIsNone(f.accept(flow(1000,245,sensor=1),100.)[0])
        self.assertIsNone(f.accept(flow(1000,245,kind='OPTICAL_FLOW'),100.)[0])
        self.assertEqual(f.samples[f.selected].quality,4)
        self.assertEqual(len(f.diagnostics(100.)),3)

    def test_duplicates_do_not_refresh_or_count_and_low_sample_resets_window(self):
        f=FlowMonitor(quality_authoritative=True)
        for i in range(16):f.accept(flow(1000000+i*200000),100.+i*.2)
        self.assertEqual(f.good_samples,16)
        self.assertEqual(f.good_since,100.)
        f.accept(flow(4000000),104.)
        self.assertEqual(f.good_samples,16)
        self.assertEqual(f.samples[f.selected].received,103.)
        f.accept(flow(4200000,0),104.2)
        self.assertEqual(f.good_samples,0)
        self.assertIsNone(f.good_since)
        f.accept(flow(4400000,245),104.4)
        self.assertEqual(f.good_samples,1)
        self.assertEqual(f.good_since,104.4)

    def test_gap_bad_data_and_clock_reversal_cannot_keep_good_history(self):
        for bad in ('gap','quality','clock'):
            with self.subTest(bad=bad):
                f=FlowMonitor(quality_authoritative=True)
                for i in range(20):f.accept(flow(1000000+i*200000),100.+i*.2)
                if bad=='gap':f.accept(flow(5100000),105.)
                elif bad=='quality':f.accept(flow(4900000,math.nan),104.)
                else:f.accept(flow(100),104.)
                self.assertLessEqual(f.good_samples,1)

    def test_failure_is_latched_while_armed_even_after_quality_recovers(self):
        r=PassiveMavlinkReceiver(TelemetryConfig(enabled=False, flow_quality_authoritative=True))
        r._snapshot.armed=True
        for stamp,q in ((1000000,245),(1200000,0),(1400000,245)):
            with patch('uav_preview.telemetry.monotonic',return_value=stamp/1e6):
                r._accept(flow(stamp,q),'127.0.0.1',object())
        t=r.snapshot()
        self.assertEqual(t.flow_quality,245)
        self.assertTrue(t.navigation_fault)
        self.assertEqual(t.flow_good_samples,1)
        self.assertIsNone(t.flow_fusion_active)
        self.assertIsNone(t.estimator_reset_signature)

    def test_untrusted_placeholder_quality_is_diagnostic_not_flight_fault(self):
        r=PassiveMavlinkReceiver(TelemetryConfig(enabled=False, flow_quality_authoritative=False))
        r._snapshot.armed=True
        with patch('uav_preview.telemetry.monotonic',return_value=1.):
            r._accept(flow(1000000,4),'127.0.0.1',object())
        t=r.snapshot()
        self.assertEqual(t.flow_quality,4)
        self.assertFalse(t.flow_quality_authoritative)
        self.assertEqual(t.flow_error,'')
        self.assertEqual(t.flow_good_samples,0)
        self.assertFalse(t.navigation_fault)


class RangeEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.r=PassiveMavlinkReceiver(TelemetryConfig(enabled=False))

    def receive(self,stamp=100,sensor=0,kind='DISTANCE_SENSOR',distance=120):
        m=SimpleNamespace(time_boot_ms=stamp,id=sensor,orientation=25,
                          current_distance=distance,min_distance=10,max_distance=500,distance=9.)
        m.get_type=lambda:kind
        m.get_srcSystem=lambda:1
        m.get_srcComponent=lambda:1
        self.r._accept(m,'127.0.0.1',object())

    def test_replay_and_unidentified_legacy_message_cannot_replace_height(self):
        self.receive()
        before=self.r.snapshot()
        self.receive(distance=240)
        self.receive(kind='RANGEFINDER')
        after=self.r.snapshot()
        self.assertEqual(after.laser_height_m,1.2)
        self.assertEqual(after.last_laser_monotonic,before.last_laser_monotonic)

    def test_missing_source_time_does_not_refresh(self):
        self.receive(stamp=None)
        self.assertIsNone(self.r.snapshot().last_laser_monotonic)

    def test_other_sensor_or_invalid_measurement_cannot_keep_trusted_height(self):
        self.receive()
        self.receive(stamp=200,sensor=1)
        self.assertTrue(self.r.snapshot().navigation_fault)
        self.assertEqual(self.r.snapshot().laser_sensor_id,0)
        self.receive(stamp=300,distance=0)
        self.assertIsNone(self.r.snapshot().last_laser_monotonic)


class AccidentRegressionTests(unittest.TestCase):
    def test_log_278_pre_handoff_error_and_speed_refuse_permission(self):
        # Rounded ULog samples at t=13 s. This is not a vehicle dynamics replay.
        t=healthy()
        t.local_x_m,t.local_y_m=-1.40748,1.24384
        t.vx_m_s,t.vy_m_s=.081,.054
        h=SmoothHandoff((-1.22478,1.19184,-1.25,0.),.25,(-1.22478,1.19184),1.,
                        frame_signature=t.estimator_reset_signature)
        for i in range(40):
            now=100.+i*.1;fresh(t,now)
            h.report(h.run_id,CLIENT,i,ZERO,True,True,None,now,True)
            h.observe(t,now)
        self.assertFalse(h.ready(now))
        self.assertIn('水平位置误差',h.reason)
        self.assertIn('水平速度',h.reason)
        with self.assertRaises(ValueError):h.authorize(CLIENT,t,now)

    def test_logged_later_flow_innovation_is_rejected_even_with_good_quality_and_fusion_bit(self):
        t=healthy()
        t.flow_quality=245
        t.flow_innovation_x_ratio=17.3
        self.assertIn('光流X',detailed_health_reason(t,100.))

    def test_slow_position_samples_never_pass_despite_fast_polling(self):
        t=healthy()
        h=SmoothHandoff((4.,-2.,-1.25,0.),.25,(4.,-2.),1.,frame_signature=t.estimator_reset_signature)
        source_stamp=100000
        received=100.
        for i in range(100):
            now=100.+i*.1;fresh(t,now)
            if i%4==0:
                source_stamp=int(now*1000);received=now
            t.local_position_sample_id=source_stamp
            t.last_distinct_position_monotonic=received
            t.last_local_position_monotonic=received
            h.report(h.run_id,CLIENT,i,ZERO,True,True,None,now,True)
            h.observe(t,now)
            self.assertFalse(h.ready(now))

    def test_realtime_arrival_of_backlogged_source_samples_does_not_fake_stability(self):
        t=healthy()
        h=SmoothHandoff((4.,-2.,-1.25,0.),.25,(4.,-2.),1.,frame_signature=t.estimator_reset_signature)
        for i in range(40):
            now=100.+i*.1;fresh(t,now)
            t.local_position_sample_id=100000+i*500
            h.report(h.run_id,CLIENT,i,ZERO,True,True,None,now,True)
            h.observe(t,now)
        self.assertFalse(h.ready(now))


class WholeTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.f=local_fixtures.LocalOffboardTakeoffTests()
        self.f.setUp()

    def test_same_position_mask_all_phases_and_packet_sequence_advances(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        msgs=[self.f._decode(p) for p,_ in self.f.sock.sent]
        self.assertGreater(len(msgs),30)
        for i,m in enumerate(msgs):
            self.assertEqual(m.type_mask,LOCAL_TRAJECTORY_TYPE_MASK)
            self.assertEqual(m.coordinate_frame,mavlink2.MAV_FRAME_LOCAL_NED)
            self.assertEqual(m.get_seq(),i%256)
            self.assertTrue(all(math.isfinite(getattr(m,n)) for n in ('x','y','z','yaw','vx','vy','vz')))
            self.assertAlmostEqual(m.x,4.)
            self.assertAlmostEqual(m.y,-2.)
        self.assertFalse(self.f.coordinator.snapshot()['handoff']['authorized'])

    def test_stall_from_ground_or_climb_never_catches_up_or_automatically_restarts(self):
        for climb in (False,True):
            self.setUp()
            if climb:self.f._reach_arming()
            else:
                self.f.coordinator.begin(1.5);self.f.coordinator.poll_once()
            count=len(self.f.sock.sent)
            self.f._advance(.26);self.f.coordinator.poll_once()
            self.assertFalse(self.f.coordinator.snapshot()['active'])
            self.assertEqual(len(self.f.sock.sent),count)
            self.f._advance(.1);self.f.coordinator.poll_once()
            self.assertEqual(len(self.f.sock.sent),count)

    def test_height_alone_no_longer_completes_hover(self):
        self.f.test_reaching_height_keeps_hold_stream_active()
        c=self.f.coordinator
        c._state.phase='HOVER_VERIFY'
        c._state.hover_stable_since_monotonic=self.f.clock.now
        self.f.telemetry.local_y_m+=.2
        self.f._advance(.1);c.poll_once()
        self.assertEqual(c.snapshot()['phase'],'CLIMBING')
        self.assertIsNone(c.snapshot()['hover_stable_since_age_seconds'])

    def test_nonfinite_goal_is_rejected_before_any_packet(self):
        self.f.coordinator.begin(1.5)
        c=self.f.coordinator
        self.assertFalse(c._send_setpoint(self.f.telemetry,1.5,self.f.clock.now,(4.,-2.,math.nan,0.,0.,0.,0.)))
        self.assertEqual(self.f.sock.sent,[])
        self.assertFalse(c.snapshot()['active'])

    def test_complete_health_is_needed_before_prestream_not_only_at_keyboard_authorization(self):
        self.f.telemetry.flow_fusion_active=None
        with self.assertRaises(ValueError):self.f.coordinator.begin(1.5)
        self.assertEqual(self.f.sock.sent,[])
        self.assertEqual(self.f.actions.arm_calls,0)


class OptionalRateTests(unittest.TestCase):
    def test_opt_in_flow_range_requests_are_ground_only_bounded_and_511_only(self):
        f=rate_fixtures.PositionStreamRateTests();f.setUp()
        f.requester.config.request_flow_range=True
        for now in (100,103,106,109,112):
            f.fresh(now);f.requester._step(now)
        messages=[mavlink2.MAVLink(None).parse_char(p) for p,_ in f.packets]
        for msgid in (106,132):
            chosen=[m for m in messages if m.param1==msgid]
            self.assertEqual(len(chosen),3)
            self.assertTrue(all(m.command==511 and m.param2==200000. for m in chosen))
        f=rate_fixtures.PositionStreamRateTests();f.setUp()
        f.requester.config.request_flow_range=True
        f.telemetry.armed=True
        f.requester._step(100.)
        self.assertEqual(f.packets,[])


if __name__=='__main__':unittest.main()
