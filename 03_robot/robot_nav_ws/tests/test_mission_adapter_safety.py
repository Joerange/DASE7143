"""Execute production ROS-adapter methods with fake transports and a real mission.

Extracting just methods avoids importing unavailable ROS libraries on a laptop;
the actual tick/cancellation code is executed unchanged, not reproduced here.
"""
import ast
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
import unittest
from unittest.mock import Mock

from robot_mission.core import Mission, Goal, Grid, fresh, finite, approach_limit, wrap
from robot_mission.mapping import CachedPlan, classify_update, revalidate_cache, plan_valid


class MissionCancellationTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.0
        source = Path(__file__).resolve().parents[1] / 'src/robot_mission/robot_mission/node.py'
        module = ast.parse(source.read_text())
        cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('tick', 'nav_result', 'revoke_geometry', 'publish_status', 'check_map_source', 'map_cb', 'lifecycle_ready', 'runtime_authorized')]
        code = ast.Module(body=methods, type_ignores=[])
        self.context = dict(math=math, json=json, time=NS(monotonic=lambda: self.clock),
                            fresh=fresh, finite=finite, approach_limit=approach_limit, wrap=wrap,
                            Time=lambda: None, yaw_of=lambda q: 0., String=lambda **kw: NS(**kw),
                            GoalStatus=NS(STATUS_SUCCEEDED=4), Grid=Grid, classify_update=classify_update,
                            revalidate_cache=revalidate_cache, plan_valid=plan_valid)
        exec(compile(code, str(source), 'exec'), self.context)
        task = Mission()
        task.start(1.)
        task.target = 'bin'
        task.confirmed_at = 1.
        task.nav_goal = Goal('bin', 0., 0., 0., (1., 0., .4), self.clock)
        task.transition('VERIFY_STOP', 'arrival latched')
        nav_handle = object()
        self.node = NS(
            task=task, now=lambda: self.clock, tf=NS(lookup_transform=lambda *a: self.transform()),
            safety_rx=self.clock, safety=dict(ready=True, stamp=self.clock),
            detector_rx=self.clock, detector=dict(ready=True, stamp=self.clock),
            odom_rx=self.clock, odom=self.odom(), grid=object(), pose=(0.,0.,0.),
            nav=NS(server_is_ready=lambda: True), planner=NS(server_is_ready=lambda: True),
            targets={'bin': dict(valid=True, stamp=self.clock, map_point=[1., 0., .4], gap=.65, bearing=0.)},
            plans={}, nav_handle=nav_handle, nav_pending=False, cancel_pending=True,
            cancel_since=self.clock, plan_job=None, nav_last_feedback=None,
            poll_lifecycle=Mock(), lifecycle_ready=lambda n: True, nav_alignment=None, map_rx=self.clock, map_stamp=self.clock, map_version=1, map_load_time=0., map_fingerprint=None,
            map_sources_checked=self.clock, map_sources=None,
            get_publishers_info_by_topic=lambda t: [NS(endpoint_gid=[1]*24)],
            plan_token=0, plan_handle=None, plan_backoff={}, active_plan=None,
            nomotion_requested=self.clock, nomotion=NS(service_is_ready=lambda: False),
            nomotion_future=None, enabled=True, last_state=None,
            capable=True, authorization_rx=self.clock,
            authorization=dict(authorized=True,runtime_control_capable=True,stamp=self.clock),
            state_pub=NS(publish=Mock()), log=NS(put=Mock()),
            get_parameter=lambda name: NS(value='trash can'), get_logger=lambda: NS(info=Mock()),
            cancel_navigation=Mock(), nav_token=4)
        for name in ('revoke_geometry', 'publish_status', 'check_map_source', 'runtime_authorized'):
            setattr(self.node, name, MethodType(self.context[name], self.node))

    def transform(self):
        return NS(header=NS(stamp=NS(sec=int(self.clock), nanosec=int((self.clock % 1)*1e9))),
                  transform=NS(translation=NS(x=0., y=0.), rotation=None))

    def odom(self):
        return NS(header=NS(stamp=NS(sec=int(self.clock), nanosec=int((self.clock % 1)*1e9))),
                  twist=NS(twist=NS(linear=NS(x=0.), angular=NS(z=0.))))

    def tick(self, advance=0.):
        self.clock += advance
        n = self.node
        n.safety_rx = n.detector_rx = n.odom_rx = n.map_rx = n.map_stamp = self.clock
        n.safety['stamp'] = n.detector['stamp'] = self.clock
        n.authorization_rx = self.clock
        n.authorization['stamp'] = self.clock
        n.targets['bin']['stamp'] = self.clock
        n.odom = self.odom()
        self.context['tick'](n)

    def test_cancel_wait_preserves_verify_without_premature_done(self):
        self.tick()
        self.tick(.8)
        self.assertEqual(self.node.task.state, 'VERIFY_STOP')
        self.assertIsNone(self.node.task.stopped_since)

    def test_terminal_cancellation_then_stable_actual_stop_completes(self):
        self.tick(.1)
        handle = self.node.nav_handle
        future = NS(result=lambda: NS(status=5))  # CANCELED is expected after arrival
        self.context['nav_result'](self.node, future, 4, self.node.task.generation, handle)
        self.assertIsNone(self.node.nav_handle)
        self.assertFalse(self.node.cancel_pending)
        self.tick(.1)
        self.assertEqual(self.node.task.state, 'VERIFY_STOP')
        self.tick(.71)
        self.assertEqual(self.node.task.state, 'DONE')

    def test_goal_acceptance_pending_prevents_done(self):
        self.node.nav_handle = None
        self.node.cancel_pending = False
        self.node.nav_pending = True
        self.tick()
        self.tick(.8)
        self.assertEqual(self.node.task.state, 'VERIFY_STOP')

    def test_localization_jump_revokes_confirmed_target(self):
        self.tick()
        self.node._map_odom = (1., 0., 0.)
        self.tick(.1)
        self.assertEqual(self.node.task.state, 'REOBSERVE_RECOVERY')
        self.assertIn('localization jump', self.node.task.reason)
        self.assertEqual(self.node.task.source, 'none')
        self.assertTrue(self.node.task.reconfirm_required)
        self.node.cancel_navigation.assert_called()

    def online_map_setup(self):
        n = self.node
        n.grid = Grid(160,160,.05,(-4.,-4.,0.),[0]*25600)
        goal = Goal('bin',1.,0.,0.,(1.96,0.,.4),self.clock)
        n.task.nav_goal = goal
        n.task.confirmed = {'bin':1.}
        n.task.transition('NAVIGATE')
        n.active_plan = CachedPlan(goal,1.,((0.,0.),(.5,0.),(1.,0.)),1,0)
        n.plans = {'bin':n.active_plan}
        return n

    def map_message(self, width=160, height=160, origin=(-4.,-4.), obstacle=None):
        data = [0]*(width*height)
        if obstacle is not None: data[obstacle] = 100
        return NS(header=NS(frame_id='map',stamp=NS(sec=int(self.clock),nanosec=0)),
                  info=NS(width=width,height=height,resolution=.05,
                          origin=NS(position=NS(x=origin[0],y=origin[1]),orientation=NS(x=0.,y=0.,z=0.,w=1.)),
                          map_load_time=NS(sec=0,nanosec=0)),data=data)

    def test_production_map_growth_does_not_interrupt_navigation(self):
        n = self.online_map_setup()
        self.context['map_cb'](n,self.map_message(240,200,(-6.,-5.)))
        self.assertEqual(n.task.state,'NAVIGATE')
        self.assertEqual(n.task.source,'nav')
        n.cancel_navigation.assert_not_called()
        self.assertIn('bin',n.plans)

    def test_production_blocked_map_revokes_and_cancels_immediately(self):
        n = self.online_map_setup()
        self.context['map_cb'](n,self.map_message(obstacle=80*160+90))
        self.assertEqual((n.task.state,n.task.source),('REOBSERVE_RECOVERY','none'))
        n.cancel_navigation.assert_called_once()
        self.assertEqual(json.loads(n.state_pub.publish.call_args.args[0].data)['source'],'none')
        self.assertIsNone(n.active_plan)

    def test_inactive_lifecycle_cannot_be_mistaken_for_ready_action_server(self):
        n = self.node
        n.lifecycle_clients = {'controller_server':object(),'planner_server':object()}
        n.lifecycle_states = {'controller_server':(2,self.clock),'planner_server':(3,self.clock)}
        self.assertFalse(self.context['lifecycle_ready'](n,self.clock))
        n.lifecycle_states['controller_server'] = (3,self.clock)
        self.assertTrue(self.context['lifecycle_ready'](n,self.clock))
        self.assertFalse(self.context['lifecycle_ready'](n,self.clock+3.))

    def test_cancel_terminal_timeout_holds(self):
        self.tick(3.1)
        self.assertEqual(self.node.task.state, 'HOLD')
        self.assertIn('cancellation terminal result timeout', self.node.task.reason)

    def test_map_endpoint_identity_preserves_same_publisher(self):
        n = self.online_map_setup()
        n.check_map_source(self.clock+1.)
        self.assertEqual(n.map_sources, frozenset([bytes([1]*24)]))
        n.check_map_source(self.clock+2.)
        self.assertEqual(n.task.state, 'NAVIGATE')
        n.cancel_navigation.assert_not_called()

    def test_map_endpoint_replacement_revokes_old_coordinates(self):
        n = self.online_map_setup()
        n.check_map_source(self.clock+1.)
        old_generation = n.task.generation
        n.get_publishers_info_by_topic = lambda topic: [NS(endpoint_gid=[2]*24)]
        n.check_map_source(self.clock+2.)
        self.assertEqual((n.task.state,n.task.source), ('REOBSERVE_RECOVERY','none'))
        self.assertNotEqual(n.task.generation, old_generation)
        self.assertTrue(n.task.reconfirm_required)
        n.cancel_navigation.assert_called_once()

    def test_multiple_map_endpoints_hold_and_cancel(self):
        n = self.online_map_setup()
        n.get_publishers_info_by_topic = lambda topic: [NS(endpoint_gid=[1]*24),NS(endpoint_gid=[2]*24)]
        n.check_map_source(self.clock+1.)
        self.assertEqual((n.task.state,n.task.source,n.task.active), ('HOLD','none',False))
        n.cancel_navigation.assert_called_once()

    @unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 Python is unavailable')
    def test_actual_humble_topic_endpoint_info_with_production_source_check(self):
        # Use the installed ROS class, not a permissive mock. Humble exposes
        # endpoint_gid; publisher_gid does not exist and previously crashed tick.
        from rclpy.topic_endpoint_info import TopicEndpointInfo, TopicEndpointTypeEnum
        def endpoint(gid):
            return TopicEndpointInfo(node_name='slam_toolbox', node_namespace='/',
                                     topic_type='nav_msgs/msg/OccupancyGrid',
                                     endpoint_type=TopicEndpointTypeEnum.PUBLISHER,
                                     endpoint_gid=[gid]*24)
        first, replacement = endpoint(1), endpoint(2)
        n = self.online_map_setup()
        n.get_publishers_info_by_topic = lambda topic: [first]
        n.check_map_source(self.clock+1.)
        n.check_map_source(self.clock+2.)
        self.assertEqual(n.task.state, 'NAVIGATE')
        n.cancel_navigation.assert_not_called()
        n.get_publishers_info_by_topic = lambda topic: [replacement]
        n.check_map_source(self.clock+3.)
        self.assertEqual((n.task.state,n.task.source), ('REOBSERVE_RECOVERY','none'))
        self.assertTrue(n.task.reconfirm_required)
        n.cancel_navigation.assert_called_once()
        n.get_publishers_info_by_topic = lambda topic: [first,replacement]
        n.check_map_source(self.clock+4.)
        self.assertEqual((n.task.state,n.task.active), ('HOLD',False))


if __name__ == '__main__':
    unittest.main()
