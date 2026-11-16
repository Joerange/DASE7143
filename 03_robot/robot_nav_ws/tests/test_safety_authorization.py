"""Runtime permission must be granted explicitly and never revive on reconnection."""
import ast
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from robot_safety.authorization import AuthorityConfig, MotionAuthority
from robot_safety.core import Config, Guard


class RuntimeAuthorityTests(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.a=MotionAuthority(AuthorityConfig(capable=True))
        self.healthy()

    def healthy(self, session='backend-1'):
        self.a.update_heartbeat(dict(schema=1,stamp=self.now,session_id=session),self.now)
        self.a.update_chassis(dict(schema=1,stamp=self.now,session_id=session,
            communication_ok=True,driver_running=True,can_up=True,can_interface='can0'),self.now)

    def grant(self, **overrides):
        inputs=dict(enabled=True,now=self.now,ros_now=self.now,sensor_ready=True,sensor_reason='healthy',
                    mission_fresh=True,navigation_ready=True,source_stopped=True,actual_stopped=True)
        inputs.update(overrides)
        return self.a.request(**inputs)

    def test_restart_is_always_disarmed_with_new_authority_session(self):
        self.assertFalse(self.a.authorized)
        self.assertTrue(self.grant()[0])
        next_boot=MotionAuthority(AuthorityConfig(capable=True))
        self.assertFalse(next_boot.authorized)
        self.assertNotEqual(next_boot.session,self.a.session)

    def test_preview_only_node_cannot_be_armed(self):
        self.a.config=AuthorityConfig(capable=False)
        self.assertEqual(self.grant(),(False,'runtime_control_disabled'))

    def test_explicit_grant_and_idempotent_repeat(self):
        self.assertEqual(self.grant(),(True,'operator_authorized'))
        session=self.a.session
        self.assertEqual(self.grant(source_stopped=False,actual_stopped=False),(True,'already_authorized'))
        self.assertEqual(self.a.session,session)

    def test_arm_requires_real_nav_sensor_and_standstill_readiness(self):
        for override,reason in [(dict(navigation_ready=False),'nav2_not_ready'),
                (dict(sensor_ready=False,sensor_reason='map_stale'),'sensor_health:map_stale'),
                (dict(mission_fresh=False),'mission_state_stale'),
                (dict(actual_stopped=False),'actual_robot_not_stopped'),
                (dict(source_stopped=False),'mission_must_be_stopped_before_authorization')]:
            self.assertEqual(self.grant(**override),(False,reason))
            self.assertFalse(self.a.authorized)

    def test_old_heartbeat_header_cannot_be_refreshed_by_new_receipt(self):
        self.a.heartbeat.stamp=1.
        self.assertEqual(self.grant(),(False,'control_heartbeat_stale'))

    def test_heartbeat_timeout_revokes_and_reconnect_does_not_rearm(self):
        self.grant()
        self.assertTrue(self.a.monitor(10.6,10.6,True,'healthy',True,True))
        self.assertEqual(self.a.reason,'control_heartbeat_stale')
        self.now=10.7;self.healthy()
        self.assertFalse(self.a.monitor(self.now,self.now,True,'healthy',True,True))
        self.assertFalse(self.a.authorized)

    def test_backend_session_change_revokes_and_retires_old_messages(self):
        self.grant();old=self.a.session
        self.healthy('backend-2')
        self.assertFalse(self.a.authorized)
        self.assertNotEqual(self.a.session,old)
        self.assertFalse(self.a.update_heartbeat(dict(schema=1,stamp=10.,session_id='backend-1'),10.))
        self.assertEqual(self.a.control_session,'backend-2')

    def test_chassis_communication_loss_revokes_immediately(self):
        self.grant()
        self.now += .01
        self.a.update_chassis(dict(self.a.chassis.value,stamp=self.now,communication_ok=False),self.now)
        self.assertFalse(self.a.authorized)
        self.assertEqual(self.a.reason,'chassis_communication_lost')
        self.healthy()
        self.assertFalse(self.a.authorized)

    def test_chassis_requires_same_control_session(self):
        self.a.update_chassis(dict(self.a.chassis.value,session_id='other'),self.now)
        self.assertEqual(self.grant(),(False,'chassis_session_mismatch'))

    def test_chassis_driver_and_can_health_are_independent(self):
        for change,reason in [(dict(can_up=False),'can_interface_down'),
                              (dict(driver_running=False),'chassis_driver_not_running')]:
            self.now += .02
            self.healthy()
            self.now += .01
            self.a.update_chassis(dict(self.a.chassis.value,stamp=self.now,**change),self.now)
            self.assertEqual(self.grant(),(False,reason))

    def test_replayed_heartbeat_and_chassis_do_not_refresh_receipt(self):
        heartbeat, chassis = dict(self.a.heartbeat.value), dict(self.a.chassis.value)
        self.assertFalse(self.a.update_heartbeat(heartbeat,10.4))
        self.assertFalse(self.a.update_chassis(chassis,10.4))
        self.assertEqual(self.a.heartbeat.receipt,10.)
        self.assertEqual(self.a.chassis.receipt,10.)
        self.now=10.6
        self.assertEqual(self.grant(),(False,'control_heartbeat_stale'))

    def test_sensor_and_nav_faults_latch_off(self):
        for inputs,reason in [((False,'scan_stale',True,True),'sensor_health:scan_stale'),
                             ((True,'healthy',True,False),'nav2_not_ready'),
                             ((True,'healthy',False,True),'mission_state_stale')]:
            self.healthy();self.grant()
            self.assertTrue(self.a.monitor(10.,10.,*inputs))
            self.assertFalse(self.a.authorized)
            self.assertEqual(self.a.reason,reason)
            self.a.monitor(10.,10.,True,'healthy',True,True)
            self.assertFalse(self.a.authorized)

    def test_clock_pause_still_expires_wall_lease(self):
        self.grant()
        self.assertTrue(self.a.monitor(11.,10.,True,'healthy',True,True))
        self.assertFalse(self.a.authorized)

    def test_malformed_health_revokes(self):
        self.grant()
        self.a.update_chassis(dict(self.a.chassis.value,communication_ok='true'),self.now)
        self.assertFalse(self.a.authorized)
        self.assertEqual(self.a.reason,'chassis_health_invalid')

    def test_explicit_false_never_needs_healthy_prerequisites(self):
        self.grant()
        self.assertEqual(self.grant(enabled=False,sensor_ready=False,mission_fresh=False,
                                   navigation_ready=False,actual_stopped=False),(True,'operator_disarmed'))
        self.assertFalse(self.a.authorized)


class RevokeServiceTests(unittest.TestCase):
    def test_false_service_immediately_zeroes_and_does_not_inspect_sensors(self):
        source=Path(__file__).resolve().parents[1]/'src/robot_safety/robot_safety/command_guard_node.py'
        cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='CommandGuardNode')
        names={'set_motion_enabled','force_zero_and_stop','request_mission_stop'}
        methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in names]
        context=dict(replace=replace,time=NS(monotonic=lambda:10.),Twist=lambda:NS(linear=NS(x=0.),angular=NS(z=0.)))
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),context)
        guard=Guard(Config(motion_enabled=True));guard.output=(.1,.2);guard.commands['search']='old'
        authority=MotionAuthority(AuthorityConfig(capable=True));authority.authorized=True
        outputs=[]
        def forbidden(*args):raise AssertionError('Disarm must not inspect unavailable sensors')
        node=NS(guard=guard,authority=authority,output_pub=NS(publish=lambda m:outputs.append(m)),
                stop_pending=False,stop_future=None,stop_requested=0.,
                stop_client=NS(service_is_ready=lambda:False),resolve_slam_pose=forbidden,check_tf=forbidden,
                authority_inputs=forbidden,get_clock=forbidden)
        node.request_mission_stop=lambda now:context['request_mission_stop'](node,now)
        node.force_zero_and_stop=lambda:context['force_zero_and_stop'](node)
        response=context['set_motion_enabled'](node,NS(data=False),NS())
        self.assertTrue(response.success)
        self.assertFalse(authority.authorized)
        self.assertFalse(guard.config.motion_enabled)
        self.assertEqual(guard.output,(0.,0.))
        self.assertFalse(guard.commands)
        self.assertTrue(node.stop_pending)
        self.assertEqual((outputs[0].linear.x,outputs[0].angular.z),(0.,0.))


if __name__=='__main__':unittest.main()
