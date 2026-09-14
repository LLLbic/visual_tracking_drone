"""No real socket, flight action or app lifespan is started in these tests."""
import unittest
from unittest.mock import Mock, patch

from fastapi.testclient import TestClient

import test_local_takeoff as fixtures
from test_smooth_handoff import fresh, healthy, CLIENT
from uav_preview.local_takeoff import LOCAL_TRAJECTORY_TYPE_MASK
from uav_preview.server import create_app
from uav_preview.telemetry import PassiveMavlinkReceiver
from uav_preview.config import TelemetryConfig
from types import SimpleNamespace


class CoordinatorHandoffTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.LocalOffboardTakeoffTests()
        self.f.setUp()
        self.f.test_reaching_height_keeps_hold_stream_active()
        self.c=self.f.coordinator
        self.f.telemetry=healthy(self.f.clock.now)
        self.f.telemetry.yaw_deg=30.0
        self.run=self.c.snapshot()['handoff']['run_id']
        self.seq=0
        self.token=None

    def tick(self,axes=(0.,)*4):
        self.f.clock.advance(.1);fresh(self.f.telemetry,self.f.clock.now)
        self.seq+=1
        self.c.report_keyboard(self.run,CLIENT,self.seq,axes,True,True,not any(axes),self.token)
        self.c.poll_once()

    def ready(self):
        for _ in range(33):self.tick()
        self.assertTrue(self.c.snapshot()['handoff']['ready'])

    def authorize(self):
        self.ready()
        self.token=self.c.authorize_keyboard(self.run,CLIENT)['token']
        self.tick()  # explicit neutral sample, same sender

    def last_packet(self):return self.f._decode(self.f.sock.sent[-1][0])

    def test_same_socket_same_mask_same_target_and_no_action_on_authorize(self):
        self.ready()
        previous=self.last_packet()
        sock=self.c._socket
        count=len(self.f.sock.sent)
        actions=(self.f.actions.arm_calls,self.f.actions.land_calls,self.f.actions.disarm_calls)
        result=self.c.authorize_keyboard(self.run,CLIENT)
        self.token=result['token']
        self.assertEqual(len(self.f.sock.sent),count)
        self.assertIsNone(self.c._thread) # no second worker started by handoff
        self.tick()
        packet=self.last_packet()
        self.assertIs(self.c._socket,sock)
        self.assertEqual(packet.type_mask,LOCAL_TRAJECTORY_TYPE_MASK)
        self.assertEqual(previous.type_mask,packet.type_mask)
        for name in ('x','y','z','yaw','vx','vy','vz','coordinate_frame'):
            self.assertAlmostEqual(getattr(packet,name),getattr(previous,name))
        for _ in range(3):self.tick((1,0,0,0))
        self.assertGreater(self.last_packet().vx,0)
        self.assertEqual(actions,(self.f.actions.arm_calls,self.f.actions.land_calls,self.f.actions.disarm_calls))
        self.assertTrue(all(self.f._decode(p).get_type()=='SET_POSITION_TARGET_LOCAL_NED' for p,_ in self.f.sock.sent))

    def test_rc_land_rtl_latch_reset_and_bad_health_stop_before_next_packet(self):
        cases=[('flight_mode','POSCTL'),('flight_mode','LAND'),('flight_mode','AUTO_RTL'),
               ('rc_channel_8_pwm',1000),('rc_channel_6_pwm',2000),
               ('flow_fusion_active',False),('estimator_reset_signature',(0,0,0,0,1,0)),
               ('navigation_fault','test reset')]
        for field,value in cases:
            with self.subTest(field=field,value=value):
                self.setUp();self.authorize()
                count=len(self.f.sock.sent)
                setattr(self.f.telemetry,field,value)
                self.c.poll_once()
                self.assertFalse(self.c.snapshot()['active'])
                self.assertTrue(self.c.snapshot()['handoff']['terminal'])
                self.assertEqual(len(self.f.sock.sent),count)
                self.assertEqual(self.f.actions.land_calls,0)
                self.assertEqual(self.f.actions.disarm_calls,0)
                setattr(self.f.telemetry,field,getattr(healthy(),field))
                self.c.poll_once()
                self.assertEqual(len(self.f.sock.sent),count)
        self.setUp();self.authorize();count=len(self.f.sock.sent)
        self.f.estop_latched=True;self.c.poll_once()
        self.assertEqual(len(self.f.sock.sent),count)
        self.assertFalse(self.c.snapshot()['active'])

    def test_reset_in_preauthorization_hover_never_sends_old_coordinate(self):
        count=len(self.f.sock.sent)
        self.f.telemetry.estimator_reset_signature=(1,0,0,0,0,0)
        self.c.poll_once()
        self.assertEqual(len(self.f.sock.sent),count)
        self.assertFalse(self.c.snapshot()['active'])

    def test_revoke_retains_sender_but_requires_explicit_new_authorization(self):
        self.authorize()
        for _ in range(4):self.tick((1,0,0,0))
        sock=self.c._socket;old=self.token
        self.c.revoke_keyboard(self.run,CLIENT,self.token)
        self.token=None
        for _ in range(40):self.tick()
        self.assertIs(self.c._socket,sock)
        self.assertTrue(self.c.snapshot()['active'])
        self.assertFalse(self.c.snapshot()['handoff']['authorized'])
        self.assertEqual((self.last_packet().vx,self.last_packet().vy,self.last_packet().vz),(0.,0.,0.))
        self.assertNotEqual(self.c.authorize_keyboard(self.run,CLIENT)['token'],old)

    def test_send_error_and_large_reference_error_are_terminal_without_actions(self):
        self.authorize()
        self.f.sock.sendto=Mock(side_effect=OSError('offline test socket failure'))
        self.tick()
        self.assertFalse(self.c.snapshot()['active'])
        self.assertIn('offline test socket failure',self.c.snapshot()['last_reason'])
        self.assertTrue(self.c.snapshot()['handoff']['terminal'])
        self.assertEqual(self.f.actions.land_calls,0)
        self.setUp();self.authorize();count=len(self.f.sock.sent)
        self.f.telemetry.local_x_m+=.46
        self.c.poll_once()
        self.assertEqual(len(self.f.sock.sent),count)
        self.assertFalse(self.c.snapshot()['active'])

    def test_missing_reset_evidence_blocks_ground_start_without_packets(self):
        self.f=fixtures.LocalOffboardTakeoffTests();self.f.setUp()
        self.f.telemetry.last_reset_evidence_monotonic=None
        with self.assertRaises(ValueError):self.f.coordinator.begin(1.5)
        self.assertEqual(self.f.sock.sent,[])
        self.assertEqual(self.f.actions.arm_calls,0)


class HandoffAPITests(unittest.TestCase):
    def setUp(self):
        self.runtime=Mock()
        self.runtime.report_handoff_input.return_value={'active':False}
        self.runtime.authorize_handoff.return_value={'token':'t'*24,'state':{'active':True}}
        self.runtime.revoke_handoff.return_value={'active':True}
        with patch('uav_preview.server.Runtime',return_value=self.runtime):
            app=create_app(Mock())
        self.client=TestClient(app) # intentionally no lifespan: no threads, sockets or camera
        self.identity={'run_id':'r'*24,'client_id':CLIENT}
        self.input={**self.identity,'pitch':0,'roll':0,'throttle':0,'yaw':0,
                    'sequence':1,'foreground':True,'confirmed':True,'keys_released':True}

    def tearDown(self):self.client.close()

    def test_input_is_permission_data_not_an_action_or_legacy_sender(self):
        response=self.client.post('/api/local-takeoff/keyboard/input',json=self.input)
        self.assertEqual(response.status_code,200)
        self.runtime.report_handoff_input.assert_called_once()
        self.runtime.start.assert_not_called()
        self.runtime.set_armed.assert_not_called()
        self.runtime.enable_keyboard_control.assert_not_called()

    def test_validation_requires_boolean_presence_neutral_integer_sequence_and_finite_axes(self):
        for field,value in [('sequence',True),('sequence',-1),('foreground','yes'),('confirmed',1),
                            ('keys_released','true'),('pitch',2),('pitch','NaN'),('token','short')]:
            with self.subTest(field=field):
                response=self.client.post('/api/local-takeoff/keyboard/input',json={**self.input,field:value})
                self.assertEqual(response.status_code,422)
        missing=self.input.copy();del missing['keys_released']
        self.assertEqual(self.client.post('/api/local-takeoff/keyboard/input',json=missing).status_code,422)
        self.runtime.report_handoff_input.assert_not_called()

    def test_explicit_authorization_and_backend_rejection(self):
        path='/api/local-takeoff/keyboard/authorize'
        self.assertEqual(self.client.post(path,json={**self.identity,'confirmation':'ENABLE'}).status_code,409)
        self.runtime.authorize_handoff.assert_not_called()
        self.assertEqual(self.client.post(path,json={**self.identity,'confirmation':'HANDOFF'}).status_code,200)
        self.runtime.authorize_handoff.side_effect=ValueError('fusion evidence missing')
        r=self.client.post(path,json={**self.identity,'confirmation':'HANDOFF'})
        self.assertEqual(r.status_code,409);self.assertIn('fusion evidence missing',r.text)


class PositionEvidenceTests(unittest.TestCase):
    def packet(self,stamp,x=1.):
        m=SimpleNamespace(time_boot_ms=stamp,x=x,y=0.,z=-1.,vx=0.,vy=0.,vz=0.)
        m.get_type=lambda:'LOCAL_POSITION_NED'
        m.get_srcSystem=lambda:1
        m.get_srcComponent=lambda:1
        return m

    def test_retransmission_never_refreshes_sample_age_or_pose(self):
        r=PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        with patch('uav_preview.telemetry.monotonic',return_value=100.):r._accept(self.packet(123),'127.0.0.1',object())
        with patch('uav_preview.telemetry.monotonic',return_value=101.):r._accept(self.packet(123,x=9.),'127.0.0.1',object())
        t=r.snapshot()
        self.assertEqual(t.local_position_sample_id,123)
        self.assertEqual(t.last_distinct_position_monotonic,100.)
        self.assertEqual(t.last_local_position_monotonic,100.)
        self.assertEqual(t.local_x_m,1.)

    def test_source_clock_regression_latches_but_uint32_wrap_is_valid(self):
        r=PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        r._accept(self.packet(123),'127.0.0.1',object())
        r._accept(self.packet(120),'127.0.0.1',object())
        self.assertTrue(r.snapshot().navigation_fault)
        r=PassiveMavlinkReceiver(TelemetryConfig(enabled=False))
        r._accept(self.packet(0xFFFFFFF0),'127.0.0.1',object())
        r._accept(self.packet(5),'127.0.0.1',object())
        self.assertFalse(r.snapshot().navigation_fault)
        self.assertEqual(r.snapshot().local_position_sample_id,5)


if __name__=='__main__':unittest.main()
