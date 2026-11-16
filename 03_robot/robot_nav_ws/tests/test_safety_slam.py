"""SLAM localization and TF fault tests; no ROS commands or hardware required."""
import ast
import json
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from robot_safety.core import Config, Guard, Sample, dynamic_tf_valid, map_valid, scan_points, fresh
from robot_safety.authorization import MotionAuthority


class SlamHealthTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.guard = Guard(Config(motion_enabled=True, localization_mode='slam'))
        self.health()
        self.mission()
        self.guard.tick(self.now, self.now)

    def health(self):
        g, t = self.guard, self.now
        g.scan = Sample(t, t, scan_points([3.] * 720, -math.pi, math.pi/360, .1, 10., g.config))
        g.depth = Sample(t, t, [])
        g.odom = Sample(t, t, (0., 0.))
        g.odom_pose = Sample(t, t, (0., 0., 0.))
        g.map = Sample(t, t, True)
        g.tf = Sample(t, t, True)
        if g.slam_pose is None:
            g.slam_pose = Sample(t, t, (0., 0., 0.))

    def mission(self):
        self.guard.update_mission(dict(schema=1, stamp=self.now, task_generation='slam',
            state='SEARCH_CONTINUOUS', source='search', control_epoch=1, motion_enabled=True), self.now)

    def run_command(self):
        self.guard.command('search', 0., .15, self.now)
        return self.guard.tick(self.now+.02, self.now+.02)

    def test_slam_needs_no_amcl_or_fabricated_covariance(self):
        self.assertIsNone(self.guard.localization)
        velocity, status = self.run_command()
        self.assertGreater(velocity[1], 0.)
        self.assertTrue(status['ready'])

    def test_valid_map_is_required(self):
        for value in (None, False):
            self.guard.map = Sample(self.now, self.now, value)
            velocity, status = self.run_command()
            self.assertEqual(velocity, (0., 0.))
            self.assertEqual(status['reason'], 'map_invalid')

    def test_stale_map_header_not_renewed_by_receipt(self):
        self.guard.map.stamp = 1.
        velocity, status = self.run_command()
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'map_stale')

    def test_stale_map_receipt_not_renewed_by_future_header(self):
        self.guard.map.receipt = 1.
        velocity, status = self.run_command()
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'map_stale')

    def test_radar_and_tf_timeouts_stop_immediately(self):
        for name in ('scan', 'odom', 'tf'):
            with self.subTest(name=name):
                self.health()
                getattr(self.guard, name).stamp = 1.
                self.guard.output = (0., .15)
                velocity, status = self.run_command()
                self.assertEqual(velocity, (0., 0.))
                self.assertEqual(status['reason'], name+'_stale')

    def test_rebroadcast_current_tf_without_successful_slam_pose_rejected(self):
        self.guard.slam_pose = None
        velocity, status = self.run_command()
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'slam_pose_missing')

    def test_stationary_pose_can_persist_while_live_scan_map_tf_continue(self):
        self.now += 30.
        self.health()
        self.mission()
        velocity, status = self.run_command()
        self.assertGreater(velocity[1], 0.)
        self.assertTrue(status['ready'])

    def test_pose_must_progress_when_robot_moves(self):
        self.now += 2.
        self.health()
        self.mission()
        for pose in ((.1, 0., 0.), (0., 0., .12)):
            self.guard.odom_pose.value = pose
            velocity, status = self.run_command()
            self.assertEqual(velocity, (0., 0.))
            self.assertEqual(status['reason'], 'slam_pose_not_progressing')

    def test_paused_ros_clock_does_not_prevent_wall_watchdog(self):
        velocity, status = self.guard.tick(self.now+1., self.now)
        self.assertEqual(velocity, (0., 0.))
        self.assertFalse(status['ready'])

    def test_unknown_or_bad_map_not_ready(self):
        self.assertFalse(map_valid(10, 10, .05, (0., 0., 0.), [-1]*100))
        self.assertFalse(map_valid(10, 10, .05, (0., 0., 0.), [0]*99))
        self.assertFalse(map_valid(10, 10, 0., (0., 0., 0.), [0]*100))
        self.assertFalse(map_valid(10, 10, .05, (0., 0., 0.), [101]*100))
        self.assertFalse(map_valid(10, 10, .05, (0., 0., 0.), [0]*100, 'odom'))
        self.assertTrue(map_valid(10, 10, .05, (-2., 3., 0.), [-1]*90+[0]*10))

    def test_compatibility_amcl_mode_still_requires_covariance(self):
        self.guard.config = Config(motion_enabled=True, localization_mode='amcl')
        velocity, status = self.run_command()
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'localization_stale')

    def test_new_wait_state_never_grants_a_source(self):
        self.guard.update_mission(dict(schema=1, stamp=self.now, task_generation='slam',
            state='WAIT_PATH', source='none', control_epoch=2, motion_enabled=True), self.now)
        self.assertFalse(self.guard.command('search', 0., .15, self.now))
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'mission_stop')

    def test_status_echoes_current_geometry_epoch_after_slam_reset(self):
        self.guard.mission.value['geometry_epoch'] = 4
        file = Path(__file__).resolve().parents[1]/'src/robot_safety/robot_safety/command_guard_node.py'
        cls = next(x for x in ast.parse(file.read_text()).body if isinstance(x, ast.ClassDef) and x.name=='CommandGuardNode')
        method = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name=='tick')
        context = dict(time=NS(monotonic=lambda: self.now), fresh=fresh, json=json, replace=replace,
                       Twist=lambda: NS(linear=NS(x=0.), angular=NS(z=0.)), String=NS)
        exec(compile(ast.Module(body=[method], type_ignores=[]),str(file),'exec'),context)
        captured=[]
        node = NS(guard=self.guard, resolve_slam_pose=lambda: None, check_tf=lambda *a: None,
                  authority=MotionAuthority(), authority_inputs=lambda *a:(True,'healthy',True,True),
                  request_mission_stop=lambda *a: None,
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(self.now*1e9))),
                  output_pub=NS(publish=lambda m: None), status_pub=NS(publish=lambda m:captured.append(m)),
                  output_topic='/cmd_vel_guard_preview', last_status=0., last_reason=None)
        context['tick'](node)
        self.assertEqual(json.loads(captured[0].data)['geometry_epoch'], 4)


class DynamicTransformTests(unittest.TestCase):
    def setUp(self):
        self.config = Config(localization_mode='slam')
        self.now = 10.
        self.map = Sample(10.2, 10., True)
        self.odom = Sample(10., 10., True)

    def test_slam_future_tolerance_is_bounded(self):
        self.assertTrue(dynamic_tf_valid(self.map, self.odom, 10., 10., self.config))
        self.map.stamp = 11.
        self.assertFalse(dynamic_tf_valid(self.map, self.odom, 10., 10., self.config))

    def test_static_lookup_without_dynamic_receipt_rejected(self):
        self.assertFalse(dynamic_tf_valid(None, self.odom, 10., 10., self.config))
        self.assertFalse(dynamic_tf_valid(self.map, None, 10., 10., self.config))

    def test_old_header_rebroadcast_with_fresh_receipt_rejected(self):
        self.map.stamp = 2.
        self.assertFalse(dynamic_tf_valid(self.map, self.odom, 10., 10., self.config))

    def test_map_transform_publisher_silence_rejected(self):
        self.map.receipt = 9.
        self.assertFalse(dynamic_tf_valid(self.map, self.odom, 10., 10., self.config))

    def test_successful_stale_buffer_lookup_still_rejected(self):
        # Execute the real adapter method with a TF buffer that successfully
        # returns an old transform while newer receipts exist separately.
        file = Path(__file__).resolve().parents[1]/'src/robot_safety/robot_safety/command_guard_node.py'
        cls = next(x for x in ast.parse(file.read_text()).body if isinstance(x, ast.ClassDef) and x.name=='CommandGuardNode')
        method = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name=='check_tf')
        context = dict(Sample=Sample, dynamic_tf_valid=dynamic_tf_valid, Time=lambda: None,
                       TransformException=RuntimeError, transform_matrix=lambda x: None,
                       stamp_seconds=lambda s: s.sec)
        exec(compile(ast.Module(body=[method], type_ignores=[]),str(file),'exec'),context)
        stale = NS(header=NS(stamp=NS(sec=2.)), transform=None)
        node = NS(tf_buffer=NS(lookup_transform=lambda *a: stale),
                  dynamic_edges={('map','odom'):self.map,('odom','base_link'):self.odom},
                  guard=Guard(self.config))
        context['check_tf'](node,10.,10.)
        self.assertFalse(node.guard.tf.value)


if __name__ == '__main__':
    unittest.main()
