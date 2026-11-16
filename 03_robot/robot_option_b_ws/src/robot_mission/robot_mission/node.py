"""ROS 2 Humble adapter; all action calls are asynchronous and generation checked."""
import json
import copy
from dataclasses import fields
import math
import os
from pathlib import Path
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener
from .core import Mission, Timing, Docking, Grid, fresh, docking_goals, path_length, approach_limit, wrap, finite, locked_target_geometry
from .mapping import CachedPlan, classify_update, plan_valid, plan_issue, remaining_path, revalidate_cache
from .refinement import TargetRefiner
from .option_b import OptionBMission
from .exploration import choose_view
from .core import Goal


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class EventLog:
    def __init__(self, path):
        path = Path(os.path.expanduser(path))
        path.mkdir(parents=True, exist_ok=True)
        self.path = path / ('mission_' + time.strftime('%Y%m%d_%H%M%S') + '.jsonl')
        self.queue = queue.Queue(maxsize=1024)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def put(self, record):
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            pass  # Logging must never block control; bounded event-only logging.

    def run(self):
        with self.path.open('a', buffering=1) as f:
            while True:
                record = self.queue.get()
                if record is None:
                    return
                f.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        for name, default in {'motion_enabled': False, 'allow_runtime_control': False, 'autostart': False, 'target_label': 'trash can',
                              'mission_timeout': 180., 'log_dir': '~/.ros/robot_mission'}.items():
            self.declare_parameter(name, default)
        self.capable = bool(self.get_parameter('allow_runtime_control').value)
        self.enabled = False
        self.authorization = {}
        self.authorization_rx = -math.inf
        self.authority_boot = ''
        self.authority_revision = -1
        self.retired_authority_boots = set()
        timeout = float(self.get_parameter('mission_timeout').value)
        if not finite(timeout) or not 10 <= timeout <= 1800:
            raise ValueError('mission_timeout must be 10..1800 seconds')
        defaults = Timing()
        timing = Timing(**{f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                           for f in fields(Timing)})
        lock_target = bool(self.declare_parameter('lock_target_on_navigation', False).value)
        visual_refinement = bool(self.declare_parameter('visual_target_refinement', False).value)
        docking_defaults = Docking()
        docking = Docking(**{f.name:self.declare_parameter('docking_'+f.name, getattr(docking_defaults,f.name)).value
                             for f in fields(Docking)})
        self.target_update_interval = float(self.declare_parameter('target_update_interval', 1.).value)
        if not finite(self.target_update_interval) or self.target_update_interval < .5:
            raise ValueError('target_update_interval must be at least 0.5 seconds')
        self.navigation_speed_scale = float(self.declare_parameter('navigation_speed_scale', 1.).value)
        self.navigation_max_linear = float(self.declare_parameter('navigation_max_linear', .10).value)
        if not finite(self.navigation_speed_scale) or not 0 < self.navigation_speed_scale <= 2.5:
            raise ValueError('navigation_speed_scale must be within (0, 2.5]')
        if not finite(self.navigation_max_linear) or not 0 < self.navigation_max_linear <= .25:
            raise ValueError('navigation_max_linear must be within (0, 0.25] m/s')
        self.exploration_enabled = bool(self.declare_parameter('exploration_enabled', True).value)
        task_class = OptionBMission if self.exploration_enabled else Mission
        self.task = task_class(timeout=timeout, timing=timing, lock_target_on_navigation=lock_target,
                            docking=docking, visual_refinement=visual_refinement)
        self.refiner = TargetRefiner()
        self.refine_pending = None
        self.refine_last_sent = -math.inf
        self.refinement_status = 'waiting_for_target'
        self.target_revision = 0
        self.log = EventLog(self.get_parameter('log_dir').value)
        self.view_pub = self.create_publisher(String, '/exploration/view_request', 1)
        self.sent_view = None
        self.selection_worker = None
        self.selection_results = queue.SimpleQueue()
        self.selection_retry = 0.
        self.create_subscription(String, '/exploration/view_result', self.view_cb, 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 1)
        self.control_pub = self.create_publisher(String, '/mission/control_state', 1)
        self.control_snapshot = None
        self.last_telemetry = -math.inf
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.control_timer = self.create_timer(.05, self.publish_control,
            callback_group=self.control_group, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.search_pub = self.create_publisher(Twist, '/cmd_vel_search', 1)
        self.limit_pub = self.create_publisher(String, '/approach/speed_limit', 1)
        self.nav_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 1)
        self.targets, self.plans = {}, {}
        self.safety, self.detector = {}, {}
        self.safety_rx = self.detector_rx = self.odom_rx = -math.inf
        self.odom = None
        self.grid = None
        self.map_version = 0
        self.map_fingerprint = None
        self.map_load_time = 0.
        self.map_stamp = -math.inf
        self.map_rx = -math.inf
        self.map_sources = None
        self.map_sources_checked = -math.inf
        self.active_plan = None
        self.nav_alignment = None
        self.plan_job = None
        self.plan_handle = None
        self.plan_token = 0
        self.plan_backoff = {}
        self.nav_handle = None
        self.nav_token = 0
        self.nav_pending = False
        self.cancel_pending = False
        self.cancel_since = None
        self.nav_last_feedback = None
        self.pose = None
        self.pose_stamp = -math.inf
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.lifecycle_clients = {name: self.create_client(GetState, '/'+name+'/get_state') for name in
                                  ('planner_server', 'controller_server', 'bt_navigator', 'behavior_server', 'velocity_smoother')}
        self.lifecycle_states, self.lifecycle_pending = {}, {}
        self.lifecycle_polled = -math.inf
        self.create_subscription(String, '/perception/candidate_event', self.event_cb, 10)
        self.create_subscription(String, '/perception/confirmation', self.confirm_cb, 10)
        self.create_subscription(String, '/perception/candidates', self.candidates_cb, 1)
        self.create_subscription(String, '/target/estimates', self.targets_cb, 1)
        self.create_subscription(String, '/safety/status', self.safety_cb, 1)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(NavPath, '/plan', self.path_cb, 1)
        self.create_service(Trigger, '/mission/start', self.start_cb)
        self.create_service(Trigger, '/mission/stop', self.stop_cb)
        self.create_service(Trigger, '/mission/safety_stop', self.safety_stop_cb)
        self.last_state = None
        self.create_timer(.05, self.tick)
        # A runtime grant never starts a task; explicit /mission/start is required.
        self.get_logger().info('Mission ready; motion_enabled=%s; log=%s' % (self.enabled, self.log.path))

    def view_cb(self, msg):
        if not self.exploration_enabled:
            return
        try:
            result = json.loads(msg.data)
            if self.task.accept_view(result):
                self.log.put(dict(event='view_result', **result))
        except (ValueError, TypeError):
            pass

    def poll_exploration(self, now):
        while not self.selection_results.empty():
            generation, geometry, selection, error = self.selection_results.get_nowait()
            self.selection_worker = None
            self.selection_retry = now+2.
            if error:
                self.get_logger().warning('View selection failed: '+error)
            if (generation, geometry) == (self.task.generation, self.task.geometry_epoch) and self.task.state == 'SELECT_VIEW':
                self.task.selected = selection
        if (self.task.state != 'SELECT_VIEW' or self.selection_worker or now < self.selection_retry
                or not self.pose or not self.grid):
            return
        generation, geometry = self.task.generation, self.task.geometry_epoch
        grid, memory, pose = self.grid, copy.deepcopy(self.task.memory), self.pose
        def run():
            try:
                selection = choose_view(grid, memory, pose, now)
                self.selection_results.put((generation, geometry, selection, ''))
            except Exception as exc:
                self.selection_results.put((generation, geometry, None, type(exc).__name__))
        self.selection_worker = threading.Thread(target=run, daemon=True, name='view_selector')
        self.selection_worker.start()

    def start_exploration(self):
        task = self.task
        if self.nav_pending or self.nav_handle or self.cancel_pending or not self.runtime_authorized(time.monotonic(), self.now()):
            return
        goal = task.nav_goal
        if not self.grid or not task.exploration_plan or not self.grid.path_free(task.exploration_plan, self.pose[2], goal.yaw):
            task.memory.reject((goal.x, goal.y, goal.yaw), time.monotonic(), 'path_changed')
            task.transition('SELECT_VIEW', 'map changed; selecting another viewpoint')
            return
        self.active_plan = CachedPlan(goal, path_length(task.exploration_plan), task.exploration_plan,
                                      self.map_version, task.geometry_epoch)
        self.nav_alignment = getattr(self, '_map_odom', None)
        self.nav_token += 1
        token, generation = self.nav_token, task.generation
        self.nav_pending = True
        request = NavigateToPose.Goal()
        request.pose = self.pose_msg(goal)
        self.nav.send_goal_async(request, feedback_callback=self.feedback_cb).add_done_callback(
            lambda f: self.nav_accepted(f, token, generation))

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def decode(self, msg, age=2.):
        try:
            value = json.loads(msg.data, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
            if not isinstance(value, dict) or value.get('schema') != 1:
                return None
            if value.get('task_generation') != self.task.generation or value.get('geometry_epoch', 0) != self.task.geometry_epoch or not fresh(value.get('stamp'), self.now(), age):
                return None
            return value
        except (ValueError, TypeError):
            return None

    def candidates_cb(self, msg):
        value = self.decode(msg, self.task.timing.detector_timeout)
        if value is not None:
            self.detector, self.detector_rx = value, time.monotonic()

    def safety_cb(self, msg):
        try:
            authority = json.loads(msg.data, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
            if (not isinstance(authority, dict) or authority.get('schema') != 1 or
                    type(authority.get('authorized')) is not bool or type(authority.get('ready')) is not bool or
                    type(authority.get('runtime_control_capable')) is not bool or
                    not fresh(authority.get('stamp'), self.now(), self.task.timing.status_timeout)):
                return
            session = authority.get('authority_session', '')
            if not isinstance(session, str):
                return
            boot, revision, control = session.split(':', 2)
            revision = int(revision)
            if not boot or revision < 0 or boot in self.retired_authority_boots:
                return
            if boot == self.authority_boot:
                if revision < self.authority_revision or authority['stamp'] <= self.authorization.get('stamp', -math.inf):
                    return
            elif self.authority_boot:
                self.retired_authority_boots.add(self.authority_boot)
            self.authority_boot, self.authority_revision = boot, revision
            self.authorization, self.authorization_rx = authority, time.monotonic()
            self.enabled = self.runtime_authorized(self.authorization_rx, self.now())
            if not self.enabled and self.task.active:
                self.task.hold('safety stop: '+authority.get('authorization_reason','runtime permission revoked'))
                self.cancel_navigation()
                self.publish_status()
            elif (not self.enabled and self.task.state=='HOLD' and
                  self.task.reason.startswith('safety stop:')):
                reason='safety stop: '+authority.get('authorization_reason','runtime permission revoked')
                if reason!=self.task.reason:
                    self.task.hold(reason)
                    self.publish_status()
        except (ValueError, TypeError):
            return
        value = self.decode(msg, self.task.timing.status_timeout)
        if value is not None:
            self.safety, self.safety_rx = value, time.monotonic()

    def runtime_authorized(self, now, ros_now):
        # Runtime permission is a guard lease, independent of mission UUID.
        # Safety/geometry readiness below still requires the current task UUID.
        return bool(self.capable and self.authorization.get('authorized') is True and
                    self.authorization.get('runtime_control_capable') is True and
                    0 <= now-self.authorization_rx <= self.task.timing.status_timeout and
                    fresh(self.authorization.get('stamp'), ros_now, self.task.timing.status_timeout))

    def odom_cb(self, msg):
        self.odom, self.odom_rx = msg, time.monotonic()

    def targets_cb(self, msg):
        value = self.decode(msg, self.task.timing.target_timeout)
        if value is None or value['stamp'] <= self.task.observation_after:
            return
        targets = {}
        for t in value.get('targets', []):
            try:
                if not isinstance(t.get('track_id'), str):
                    continue
                if t.get('valid') and (len(t['map_point']) != 3 or len(t['base_point']) != 3 or
                    not finite(*t['map_point'], *t['base_point'], t['gap'], t['bearing'], t['stamp'])):
                    continue
                targets[t['track_id']] = t
            except (KeyError, TypeError):
                continue
        self.targets = targets  # absent tracks are absent now, never silently refreshed

    def event_cb(self, msg):
        value = self.decode(msg, 1.)
        if value and self.task.candidate_event(value, time.monotonic()):
            self.plans.clear()
            self.plan_backoff.clear()
            self.log.put({'event': 'candidate_event', 'stamp': self.now(), 'request_id': value['request_id'],
                          'observation_id': value.get('observation_id'), 'candidates': value['candidates']})

    def confirm_cb(self, msg):
        value = self.decode(msg, 9.)
        if value:
            accepted = self.task.confirmation(value, time.monotonic())
            self.log.put({'event': 'confirmation', 'stamp': self.now(), 'accepted_context': accepted,
                          'request_id': value.get('request_id'), 'valid': value.get('valid'),
                          'candidates': value.get('candidates', []), 'error': value.get('error', '')})

    def revoke_geometry(self, reason, reconfirm=False):
        if not self.task.active:
            return
        if self.task.lock_target_on_navigation and self.task.nav_goal is not None and self.task.state != 'EXPLORE':
            # A frame reset cannot silently select a second target in the same
            # operator-started run. Keep the original anchor for display and stop.
            self.task.hold('locked target coordinates unavailable: '+reason)
        else:
            self.task.recover(reason, time.monotonic(), self.now(), reconfirm=reconfirm)
        self.plans.clear()
        self.plan_backoff.clear()
        self.active_plan = None
        self.nav_alignment = None
        self.targets.clear()
        self.plan_token += 1
        if self.plan_handle:
            self.plan_handle.cancel_goal_async()
        self.plan_job, self.plan_handle = None, None
        self.cancel_navigation()
        self.publish_status()  # Revoke before another timer/controller iteration.

    def map_cb(self, msg):
        try:
            if msg.header.frame_id != 'map':
                raise ValueError('map frame must be map')
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if not fresh(stamp, self.now(), self.task.timing.map_timeout):
                return  # Delayed/replayed maps never replace the current revision.
            if stamp < self.map_stamp-.05:
                return
            o = msg.info.origin
            if not finite(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w):
                raise ValueError('invalid map quaternion')
            new_grid = Grid(msg.info.width, msg.info.height, msg.info.resolution,
                            (o.position.x, o.position.y, yaw_of(o.orientation)), list(msg.data))
            fingerprint = (new_grid.width, new_grid.height, new_grid.resolution, new_grid.origin, hash(tuple(msg.data)))
            load_time = msg.info.map_load_time.sec+msg.info.map_load_time.nanosec/1e9
            change = classify_update(self.grid, new_grid, self.map_load_time, load_time)
            self.map_rx, self.map_stamp = time.monotonic(), stamp
            if fingerprint == self.map_fingerprint and change != 'reset':
                return
            self.map_fingerprint, self.map_load_time = fingerprint, load_time
            self.grid = new_grid
            self.map_version += 1
            if change == 'reset':
                self.revoke_geometry('SLAM map reset or coordinate system replacement', reconfirm=True)
                return
            # An accepted Nav2 action owns live route updates. Ordinary SLAM
            # revisions must not cancel it using a second, cached whole-route
            # check. Nav2 keeps its unknown-space rule and obstacle costmaps.
            # Resets and localization jumps still invalidate the coordinate frame.
            if self.task.source == 'nav':
                return
            self.plans = revalidate_cache(self.plans, self.grid, self.map_version, self.task.geometry_epoch)
            if self.active_plan and self.task.state == 'READY_TO_NAV':
                issue = plan_issue(self.active_plan, self.grid, self.task.geometry_epoch, self.pose)
                if issue:
                    self.get_logger().warning('PATH_INVALID_BEFORE_DISPATCH '+json.dumps(issue))
                    self.revoke_geometry('updated map invalidated pending docking path')
            elif self.task.state == 'READY_TO_NAV' and self.task.target not in self.plans:
                self.revoke_geometry('updated map invalidated pending docking path')
        except (ValueError, TypeError, OverflowError):
            self.grid = None
            self.revoke_geometry('invalid SLAM map; waiting for fresh geometry', reconfirm=True)

    def path_cb(self, msg):
        # Follow the actual path of the accepted Nav2 action. Its global planner
        # rejects unknown space; its controller and the guard handle obstacles.
        if self.task.source != 'nav' or not self.task.nav_goal or msg.header.frame_id != 'map':
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not fresh(stamp, self.now(), 3.) or stamp <= self.task.observation_after:
            return
        points = tuple((p.pose.position.x, p.pose.position.y) for p in msg.poses)
        g = self.task.nav_goal
        if not points or any(not finite(*pt) for pt in points) or math.dist(points[-1], (g.x, g.y)) > .08:
            return
        plan = CachedPlan(g, path_length(points), points, self.map_version, self.task.geometry_epoch)
        self.active_plan = plan

    def start_cb(self, request, response):
        now, ros_now = time.monotonic(), self.now()
        self.enabled = self.runtime_authorized(now, ros_now)
        if not self.enabled:
            response.success, response.message = False, 'independent guard motion authorization is absent'
        elif (self.authorization.get('ready') is not True or
                self.authorization.get('task_generation') != self.task.generation or
                self.authorization.get('geometry_epoch', 0) != self.task.geometry_epoch or
                not self.lifecycle_ready(now)):
            response.success, response.message = False, 'sensors, localization or Nav2 not ready'
        elif self.task.active or self.nav_pending or self.nav_handle or self.cancel_pending:
            response.success, response.message = False, 'mission or navigation cancellation still active'
        else:
            self.task.start(now)
            self.target_revision = 0
            self.refinement_status = 'waiting_for_target'
            self.refine_last_sent = -math.inf
            self.targets.clear()
            self.plans.clear()
            self.detector, self.safety = {}, {}
            self.publish_status()
            response.success, response.message = True, 'new mission '+self.task.generation
        return response

    def stop_cb(self, request, response):
        self.task.hold('operator stop')
        self.cancel_navigation()
        self.publish_status()
        response.success, response.message = True, 'mission movement revoked; navigation cancellation requested'
        return response

    def safety_stop_cb(self, request, response):
        # Trigger has no payload; the following safety status supplies the exact
        # reason. Keep this distinct from an operator's /mission/stop request.
        reason=(self.authorization.get('authorization_reason') if not self.enabled else None)
        self.task.hold('safety stop: '+(reason or 'guard cancellation requested'))
        self.cancel_navigation()
        self.publish_status()
        response.success,response.message=True,'safety stop accepted; navigation cancellation requested'
        return response

    def pose_msg(self, goal):
        p = PoseStamped()
        p.header.frame_id, p.header.stamp = 'map', self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = goal.x, goal.y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(goal.yaw/2), math.cos(goal.yaw/2)
        return p

    def poll_plan(self, now, ros_now):
        if self.plan_job:
            if now-self.plan_job[0] > self.task.timing.plan_timeout:
                self.plan_token += 1
                if self.plan_handle:
                    self.plan_handle.cancel_goal_async()
                self.plan_job, self.plan_handle = None, None
            return
        tracks = self.task.planning_tracks()
        if not tracks or not self.grid or not self.pose or not self.planner.server_is_ready():
            return
        for track in tracks:
            target = self.targets.get(track)
            if not target or not target.get('valid') or not fresh(target.get('stamp'), ros_now, self.task.timing.target_timeout) or target['stamp'] <= self.task.observation_after:
                continue
            old = self.plans.get(track)
            if old and fresh(old.goal.stamp, ros_now, 1.) and math.dist(old.goal.surface, target['map_point']) < .12:
                continue
            if now < self.plan_backoff.get(track, 0):
                continue
            dock = self.task.docking
            goals = docking_goals(target, self.pose[:2], self.grid, gap=dock.target_gap,
                                  margin=dock.goal_margin, min_gap=dock.min_gap)
            self.plan_backoff[track] = now+.5
            if not goals:
                self.plans.pop(track, None)
                continue
            goal = goals[0]
            self.plan_token += 1
            token, generation, version = self.plan_token, self.task.generation, self.map_version
            self.plan_job = (now, track)
            req = ComputePathToPose.Goal()
            req.goal, req.planner_id, req.use_start = self.pose_msg(goal), 'GridBased', False
            future = self.planner.send_goal_async(req)
            future.add_done_callback(lambda f, g=goal, tok=token, gen=generation, ver=version:
                                     self.plan_accepted(f, g, tok, gen, ver))
            return

    def plan_accepted(self, future, goal, token, generation, version):
        try:
            handle = future.result()
            if token != self.plan_token or generation != self.task.generation or not self.task.planning_tracks():
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.plan_job = None
                return
            self.plan_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.plan_result(f, goal, token, generation, version))
        except Exception as exc:
            self.plan_job = None
            self.get_logger().warning('planner request failed: '+type(exc).__name__)

    def plan_result(self, future, goal, token, generation, version):
        if token != self.plan_token or generation != self.task.generation:
            return
        self.plan_job, self.plan_handle = None, None
        try:
            result = future.result()
            points = [(p.pose.position.x, p.pose.position.y) for p in result.result.path.poses]
            if result.result.path.header.frame_id != 'map' or not self.grid or not points or not self.task.planning_tracks():
                return
            if result.status == GoalStatus.STATUS_SUCCEEDED and math.dist(points[-1], (goal.x, goal.y)) < .08:
                plan = CachedPlan(goal, path_length(points), tuple(points), self.map_version, self.task.geometry_epoch)
                issue = plan_issue(plan, self.grid, self.task.geometry_epoch, self.pose)
                if issue:
                    self.log.put({'event': 'path_invalid', 'stamp': self.now(), 'track_id': goal.track_id,
                                  'map_revision': self.map_version, 'detail': issue})
                    return
                self.plans[goal.track_id] = plan
                self.log.put({'event': 'path_valid', 'stamp': self.now(), 'track_id': goal.track_id,
                              'goal': [goal.x, goal.y, goal.yaw], 'length': path_length(points)})
        except Exception as exc:
            self.get_logger().warning('path validation failed: '+type(exc).__name__)

    def start_navigation(self):
        if self.nav_pending or self.nav_handle or self.cancel_pending or not self.runtime_authorized(time.monotonic(), self.now()):
            return
        if not self.nav.server_is_ready() or not self.lifecycle_ready(time.monotonic()):
            self.task.hold('Nav2 unavailable')
            return
        plan = self.plans.get(self.task.target)
        target = self.targets.get(self.task.target, {})
        if (not self.grid or not plan_valid(plan, self.grid, self.task.geometry_epoch, self.pose) or
                not target.get('valid') or not fresh(target.get('stamp'), self.now(), self.task.timing.target_timeout) or
                math.dist(target['map_point'], plan.goal.surface) >= .12):
            self.revoke_geometry('current geometry/path changed before Nav2 dispatch')
            return
        self.active_plan = plan
        self.nav_alignment = getattr(self, '_map_odom', None)
        self.nav_token += 1
        token, generation = self.nav_token, self.task.generation
        self.nav_pending = True
        request = NavigateToPose.Goal()
        request.pose = self.pose_msg(self.task.nav_goal)
        self.nav.send_goal_async(request, feedback_callback=self.feedback_cb).add_done_callback(
            lambda f: self.nav_accepted(f, token, generation))

    def feedback_cb(self, msg):
        self.nav_last_feedback = time.monotonic()

    def nav_accepted(self, future, token, generation):
        self.nav_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.task.memory.reject((self.task.nav_goal.x,self.task.nav_goal.y,self.task.nav_goal.yaw),time.monotonic(),'goal_rejected') if self.task.state == 'EXPLORE_READY' else None
                self.task.transition('SELECT_VIEW','try another viewpoint') if self.task.state == 'EXPLORE_READY' else self.task.hold('Nav2 rejected goal')
                return
            self.nav_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.nav_result(f, token, generation, handle))
            if (token != self.nav_token or generation != self.task.generation or self.task.state not in ('READY_TO_NAV', 'EXPLORE_READY')
                    or not self.runtime_authorized(time.monotonic(), self.now())):
                self.cancel_navigation()
                return
            if not self.active_plan or not self.grid or not plan_valid(self.active_plan, self.grid, self.task.geometry_epoch, self.pose):
                self.revoke_geometry('path invalidated before navigation acceptance')
                return
            self.nav_last_feedback = time.monotonic()
            self.task.navigation_accepted()
        except Exception as exc:
            self.task.hold('Nav2 request failed: '+type(exc).__name__)

    def nav_result(self, future, token, generation, handle):
        pending = self.refine_pending
        if pending is not None and pending['old_handle'] == handle and pending['old_token'] == token:
            try:
                pending['old_status'] = future.result().status
            except Exception:
                pending['old_status'] = GoalStatus.STATUS_ABORTED
            if self.nav_handle == handle:
                self.nav_handle = None
                self.cancel_pending, self.cancel_since = False, None
            return  # Preemption of the previous goal is not mission failure.
        if self.nav_handle == handle:
            self.nav_handle = None
            self.cancel_pending, self.cancel_since = False, None
        if token != self.nav_token or generation != self.task.generation:
            return
        try:
            self.task.navigation_result(future.result().status == GoalStatus.STATUS_SUCCEEDED)
        except Exception:
            self.task.hold('Nav2 result transport failure')

    def poll_refinement(self, now, ros_now):
        if self.refine_pending is not None:
            if now-self.refine_pending['sent'] > self.task.timing.goal_timeout and self.task.active:
                self.task.hold('visual goal update response timeout')
                self.cancel_navigation()
            return
        if (not self.task.visual_refinement or not self.task.active or self.task.source != 'nav'
                or self.task.state not in ('NAVIGATE','APPROACH') or self.nav_pending or self.cancel_pending
                or now-self.refine_last_sent < self.target_update_interval or self.pose is None
                or not self.runtime_authorized(now,ros_now) or not self.nav.server_is_ready()):
            return
        proposal = self.refiner.propose(self.task, self.targets, self.detector, self.pose, self.grid, ros_now)
        self.refinement_status = self.refiner.reason
        if proposal is None:
            return
        goal, observed_track = proposal
        context = dict(goal=goal, observed_track=observed_track, generation=self.task.generation,
                       old_handle=self.nav_handle, old_token=self.nav_token, old_status=None, sent=now)
        self.refine_pending, self.nav_pending, self.refine_last_sent = context, True, now
        self.refinement_status = 'update_requested'
        request = NavigateToPose.Goal()
        request.pose = self.pose_msg(goal)
        try:
            self.nav.send_goal_async(request,feedback_callback=self.feedback_cb).add_done_callback(
                lambda f,c=context:self.refinement_accepted(f,c))
            self.log.put(dict(event='target_update_requested',stamp=ros_now,task_generation=self.task.generation,
                              target_track_id=self.task.target,observed_track_id=observed_track,
                              observation_stamp=goal.stamp,goal=[goal.x,goal.y,goal.yaw]))
        except Exception:
            self.refine_pending, self.nav_pending = None, False
            self.refinement_status = 'update_transport_error'

    def refinement_accepted(self, future, context):
        if self.refine_pending is not context:
            try:
                handle = future.result()
                if handle.accepted:handle.cancel_goal_async()
            except Exception:
                pass
            return
        self.refine_pending, self.nav_pending = None, False
        accepted = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.refinement_status = 'update_rejected'
                if context['old_status'] is not None:
                    self.task.navigation_result(context['old_status'] == GoalStatus.STATUS_SUCCEEDED)
                return
            accepted = True
            self.nav_token += 1
            token = self.nav_token
            self.nav_handle, self.cancel_pending, self.cancel_since = handle, False, None
            result_future = handle.get_result_async()
            if (context['generation'] != self.task.generation or not self.task.active or self.task.source != 'nav'
                    or not self.runtime_authorized(time.monotonic(),self.now())):
                result_future.add_done_callback(lambda f:self.nav_result(f,token,context['generation'],handle))
                self.cancel_navigation()
                return
            self.task.nav_goal = context['goal']
            self.task.observed_target = context['observed_track']
            self.task.nav_finished = False
            self.task.reason = 'Nav2 accepted refined RGB-D docking goal'
            self.active_plan = None
            self.target_revision += 1
            self.nav_last_feedback = time.monotonic()
            self.refinement_status = 'update_accepted'
            self.publish_status()
            self.log.put(dict(event='target_update_accepted',stamp=self.now(),target_revision=self.target_revision,
                              task_generation=self.task.generation,observation_stamp=self.task.nav_goal.stamp,
                              goal=[self.task.nav_goal.x,self.task.nav_goal.y,self.task.nav_goal.yaw]))
            result_future.add_done_callback(lambda f:self.nav_result(f,token,context['generation'],handle))
        except Exception as error:
            self.refinement_status = 'update_transport_error'
            if accepted or context['old_status'] is not None or self.nav_handle is None:
                self.task.hold('visual goal update transport failure: '+type(error).__name__)
                self.cancel_navigation()
                self.publish_status()

    def cancel_navigation(self):
        if self.nav_handle and not self.cancel_pending:
            self.cancel_pending, self.cancel_since = True, time.monotonic()
            # Keep handle until terminal result, not merely receipt of cancel request.
            self.nav_handle.cancel_goal_async()

    def tick(self):
        now, ros_now = time.monotonic(), self.now()
        self.poll_lifecycle(now)
        self.enabled = self.runtime_authorized(now, ros_now)
        if self.task.active and not self.enabled:
            self.task.hold('runtime authorization absent or expired')
            self.cancel_navigation()
        try:
            map_tf = self.tf.lookup_transform('map', 'odom', Time())
            odom_tf = self.tf.lookup_transform('odom', 'base_link', Time())
            map_odom = map_tf.transform
            alignment = (map_odom.translation.x, map_odom.translation.y, yaw_of(map_odom.rotation))
            map_stamp = map_tf.header.stamp.sec+map_tf.header.stamp.nanosec/1e9
            stamp = odom_tf.header.stamp.sec+odom_tf.header.stamp.nanosec/1e9
            tr = odom_tf.transform.translation
            c, s = math.cos(alignment[2]), math.sin(alignment[2])
            pose = (alignment[0]+c*tr.x-s*tr.y, alignment[1]+s*tr.x+c*tr.y,
                    wrap(alignment[2]+yaw_of(odom_tf.transform.rotation)))
            # Propagate the last valid SLAM alignment with current odometry;
            # latest-common-time lookup otherwise freezes pose during scan gaps.
            map_recent = -.35 <= ros_now-map_stamp <= self.task.timing.tf_timeout
            self.pose = pose if finite(*pose) and map_recent and fresh(stamp, ros_now, self.task.timing.odom_timeout) else None
            self.pose_stamp = stamp
            previous = getattr(self, '_map_odom', None)
            if previous and self.task.active and (self.task.pending or self.task.confirmed or self.task.nav_goal):
                if math.dist(previous[:2], alignment[:2]) > .25 or abs(wrap(previous[2]-alignment[2])) > .25:
                    self.revoke_geometry('significant SLAM localization jump', reconfirm=True)
            anchor = self.nav_alignment
            if anchor and self.task.source == 'nav' and (math.dist(anchor[:2], alignment[:2]) > .20 or abs(wrap(anchor[2]-alignment[2])) > .20):
                self.revoke_geometry('accumulated SLAM correction invalidated target coordinates', reconfirm=True)
            self._map_odom = alignment
        except Exception:
            self.pose = None
        safety_ready = (now-self.safety_rx < self.task.timing.status_timeout and self.safety.get('ready', False)
                        and fresh(self.safety.get('stamp'), ros_now, self.task.timing.status_timeout))
        detected = (now-self.detector_rx < self.task.timing.detector_timeout and self.detector.get('ready', False)
                    and fresh(self.detector.get('stamp'), ros_now, self.task.timing.detector_timeout))
        stopped = False
        if self.odom is not None and now-self.odom_rx < self.task.timing.odom_timeout:
            v = self.odom.twist.twist
            os = self.odom.header.stamp.sec+self.odom.header.stamp.nanosec/1e9
            stopped = fresh(os, ros_now, self.task.timing.odom_timeout) and abs(v.linear.x)<.015 and abs(v.angular.z)<.025
        ready = safety_ready and self.pose is not None and self.grid is not None and now-self.map_rx <= self.task.timing.map_timeout and fresh(self.map_stamp, ros_now, self.task.timing.map_timeout) and self.nav.server_is_ready() and self.planner.server_is_ready() and self.lifecycle_ready(now)
        if self.pose:
            self.task.update_yaw(self.pose[2], detected)
        # A static target can briefly leave the image far away, but never support
        # blind travel into the near-target zone.
        if not self.task.lock_target_on_navigation and self.task.state == 'NAVIGATE' and self.task.nav_goal and self.pose:
            t = self.targets.get(self.task.target, {})
            map_gap = math.dist(self.pose[:2], self.task.nav_goal.surface[:2])-.345
            if map_gap <= 1.5 and not (t.get('valid') and fresh(t.get('stamp'), ros_now, self.task.timing.target_timeout)):
                self.revoke_geometry('entering near zone without current target geometry')
        if self.task.state == 'VERIFY_STOP':
            stopped = stopped and self.nav_handle is None and not self.nav_pending and not self.cancel_pending
        navigation_idle = self.nav_handle is None and not self.nav_pending and not self.cancel_pending
        before = (self.task.generation, self.task.state)
        geometry = locked_target_geometry(self.task.nav_goal, self.pose, self.pose_stamp)
        if geometry is not None:
            geometry['goal_update_pending'] = self.refine_pending is not None
        if self.exploration_enabled:
            self.task.world, self.task.pose = self.grid, self.pose
            self.poll_exploration(now)
        self.task.tick(now, ros_now, ready, stopped, self.targets, self.plans, detected, navigation_idle, geometry)
        if self.task.state == 'REOBSERVE_RECOVERY' and before != (self.task.generation, self.task.state):
            self.plans.clear()
            self.targets.clear()
            self.active_plan = None
            self.plan_token += 1
            if self.plan_handle:
                self.plan_handle.cancel_goal_async()
            self.plan_job, self.plan_handle = None, None
            self.cancel_navigation()
        self.check_map_source(now)
        self.poll_refinement(now, ros_now)
        if self.task.state in ('ACQUIRE', 'CONFIRM_PENDING', 'WAIT_PATH', 'REOBSERVE_RECOVERY') and navigation_idle:
            self.poll_plan(now, ros_now)
        elif self.plan_job:
            self.plan_token += 1
            if self.plan_handle:
                self.plan_handle.cancel_goal_async()
            self.plan_job, self.plan_handle = None, None
        if self.task.state == 'READY_TO_NAV':
            self.start_navigation()
        elif self.task.state == 'EXPLORE_READY':
            self.start_exploration()
        if self.task.source != 'nav' and self.task.state not in ('READY_TO_NAV', 'EXPLORE_READY'):
            self.cancel_navigation()
        if self.cancel_pending and now-self.cancel_since > self.task.timing.cancel_timeout:
            self.task.hold('Nav2 cancellation terminal result timeout; motion remains revoked')
        if self.task.source == 'nav' and self.nav_last_feedback and now-self.nav_last_feedback > self.task.timing.feedback_timeout:
            self.task.hold('Nav2 feedback timeout')
        if self.exploration_enabled:
            request = self.task.view_request
            if self.task.state == 'VIEW_OBSERVE' and request and request['request_id'] != self.sent_view:
                self.view_pub.publish(String(data=json.dumps(request)))
                self.sent_view = request['request_id']
                self.log.put(dict(event='view_request', **request))
        self.publish_status()
        if self.task.source == 'search':
            cmd = Twist()
            hz = self.detector.get('processing_hz', 0.)
            if self.task.state == 'SEARCH_CONTINUOUS' and detected and finite(hz):
                cmd.angular.z = min(.15, max(0., hz)*.03)
            if self.task.state == 'VIEW_TURN' and self.pose:
                cmd.angular.z = max(-.20, min(.20, wrap(self.task.turn_yaw-self.pose[2])*.8))
            self.search_pub.publish(cmd)
        if self.task.source == 'nav':
            t = self.targets.get(self.task.target, {})
            current = t.get('valid', False) and fresh(t.get('stamp'), ros_now, self.task.timing.target_timeout)
            if self.task.lock_target_on_navigation:
                geometry = locked_target_geometry(self.task.nav_goal, self.pose, self.pose_stamp)
                valid = bool(geometry and fresh(geometry['stamp'], ros_now, self.task.timing.odom_timeout))
                cap = approach_limit(geometry['gap'], latency=self.task.timing.odom_timeout+.30,
                                     speed_scale=self.navigation_speed_scale, max_linear=self.navigation_max_linear,
                                     stop_gap=self.task.docking.target_gap-self.task.docking.gap_tolerance,
                                     terminal_zone=self.task.docking.terminal_zone,
                                     terminal_speed=self.task.docking.terminal_speed) if valid else 0.
            else:
                cap = approach_limit(t['gap'], latency=self.task.timing.target_timeout + .30,
                                     speed_scale=self.navigation_speed_scale, max_linear=self.navigation_max_linear,
                                     stop_gap=self.task.docking.target_gap-self.task.docking.gap_tolerance,
                                     terminal_zone=self.task.docking.terminal_zone,
                                     terminal_speed=self.task.docking.terminal_speed) if current else self.navigation_max_linear
                valid = bool(current or self.task.state == 'NAVIGATE')
            if self.task.state == 'EXPLORE':
                cap, valid = min(.15, self.navigation_max_linear), True
            # Far navigation follows the route; target bearing alone must not
            # pin translation to zero while Nav2 is trying to align with it.
            if not self.task.lock_target_on_navigation and self.task.state == 'APPROACH' and current and abs(t['bearing']) > .35:
                cap = 0.
            limit = dict(schema=1, stamp=ros_now, task_generation=self.task.generation,
                         control_epoch=self.task.epoch, max_linear=cap, valid=valid)
            self.limit_pub.publish(String(data=json.dumps(limit)))
            # A linear-only stop belongs to the final guard. Keep a nonzero Nav2
            # planning envelope so it can still select an alignment turn.
            # Invalid geometry still receives the near-zero upstream limit.
            speed = SpeedLimit()
            speed.header.stamp = self.get_clock().now().to_msg()
            speed.percentage, speed.speed_limit = False, max(.05*self.navigation_speed_scale, cap) if valid else .0001
            self.nav_limit_pub.publish(speed)

    def lifecycle_ready(self, now):
        return all(name in self.lifecycle_states and self.lifecycle_states[name][0] == 3
                   and now-self.lifecycle_states[name][1] <= self.task.timing.lifecycle_timeout for name in self.lifecycle_clients)

    def poll_lifecycle(self, now):
        for name, (future, sent) in list(self.lifecycle_pending.items()):
            if now-sent > self.task.timing.lifecycle_request_timeout:
                future.cancel()
                self.lifecycle_pending.pop(name, None)
                self.lifecycle_states.pop(name, None)
        if now-self.lifecycle_polled < 1.:
            return
        self.lifecycle_polled = now
        for name, client in self.lifecycle_clients.items():
            if name not in self.lifecycle_pending and client.service_is_ready():
                future = client.call_async(GetState.Request())
                self.lifecycle_pending[name] = (future, now)
                future.add_done_callback(lambda f, n=name: self.lifecycle_result(n, f))

    def lifecycle_result(self, name, future):
        current = self.lifecycle_pending.get(name)
        if not current or current[0] is not future:
            return
        self.lifecycle_pending.pop(name, None)
        try:
            self.lifecycle_states[name] = (future.result().current_state.id, time.monotonic())
        except Exception:
            self.lifecycle_states.pop(name, None)

    def check_map_source(self, now):
        if now-self.map_sources_checked < 1.:
            return
        self.map_sources_checked = now
        sources = frozenset(bytes(p.endpoint_gid) for p in self.get_publishers_info_by_topic('/map'))
        if len(sources) > 1:
            self.task.hold('multiple map publishers; coordinate ownership ambiguous')
            self.cancel_navigation()
            self.publish_status()
        elif sources:
            if self.map_sources is not None and sources != self.map_sources:
                self.revoke_geometry('SLAM map publisher restarted/replaced', reconfirm=True)
            self.map_sources = sources

    def publish_control(self):
        # Publish only the last committed decision. A heartbeat must never
        # refresh decision_stamp: the guard still detects a frozen state machine.
        decision = self.control_snapshot
        if decision is not None:
            self.control_pub.publish(String(data=json.dumps(dict(decision, stamp=self.now()))))

    def publish_status(self):
        goal = self.task.nav_goal
        locked_target = (dict(track_id=goal.track_id, map_point=list(goal.surface), target_revision=self.target_revision,
                              goal=[goal.x, goal.y, goal.yaw], observation_stamp=goal.stamp)
                         if self.task.lock_target_on_navigation and goal else None)
        envelope = {'schema': 1, 'stamp': self.now(), 'task_generation': self.task.generation,
                    'state': self.task.state, 'active': self.task.active, 'reason': self.task.reason, 'source': self.task.source,
                    'control_epoch': self.task.epoch, 'target_track_id': self.task.target,
                    'request_id': self.task.pending.get('request_id') if self.task.pending else None,
                    'observation_id': self.task.pending.get('observation_id') if self.task.pending else None,
                    'exploration': self.task.memory.snapshot() if getattr(self, 'exploration_enabled', False) else None,
                    'view_selection': self.task.selection if getattr(self, 'exploration_enabled', False) else None,
                    'target_label': self.get_parameter('target_label').value,
                    'motion_enabled': self.enabled, 'coverage_radians': self.task.coverage,
                    'geometry_epoch': self.task.geometry_epoch, 'reconfirm_required': self.task.reconfirm_required,
                    'navigation_mode': 'adaptive_map' if self.task.visual_refinement else 'locked_map' if self.task.lock_target_on_navigation else 'visual',
                    'observed_target_track_id': self.task.observed_target,
                    'target_revision': self.target_revision, 'refinement_status': self.refinement_status,
                    'docking_gap': self.task.docking.target_gap,
                    'locked_target': locked_target,
                    'locked_geometry': locked_target_geometry(goal, self.pose, self.pose_stamp) if locked_target else None,
                    'completion_evidence': self.task.completion_evidence,
                    'map_revision': self.map_version, 'localization_mode': 'slam',
                    'nav_ready': self.lifecycle_ready(time.monotonic()),
                    'navigation_idle': self.nav_handle is None and not self.nav_pending and not self.cancel_pending,
                    'pose': list(self.pose) if self.pose else None,
                    'goal': [self.task.nav_goal.x, self.task.nav_goal.y, self.task.nav_goal.yaw] if self.task.nav_goal else None,
                    'path': list(self.active_plan.path) if self.active_plan else [],
                    'target': self.targets.get(self.task.target),
                    'planning_tracks': self.task.planning_tracks(),
                    'validated_paths': list(self.plans),
                    'waiting_for': ('obstacle_clearance' if self.task.active and self.safety.get('motion_blocked') else
                                    'depth' if self.task.state == 'WAIT_PATH' and not any(t.get('valid') and fresh(t.get('stamp'), self.now(), self.task.timing.target_timeout) for k,t in self.targets.items() if k in self.task.confirmed)
                                    else 'known_free_path' if self.task.state == 'WAIT_PATH' else '')}
        committed = {k:v for k,v in envelope.items() if k not in ('path', 'target', 'pose', 'goal', 'locked_geometry', 'exploration', 'view_selection')}
        committed['decision_stamp'] = envelope['stamp']
        self.control_snapshot = committed
        self.publish_control()
        key = (self.task.state, self.task.reason)
        # Paths and display geometry are telemetry, not part of the control lease.
        if time.monotonic()-self.last_telemetry >= .2 or key != self.last_state:
            self.last_telemetry = time.monotonic()
            self.state_pub.publish(String(data=json.dumps(envelope)))
        if key != self.last_state:
            self.last_state = key
            self.get_logger().info('%s: %s' % key)
            self.log.put(dict(envelope, event='state'))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.task.hold('process shutdown')
        node.cancel_navigation()
        try:
            node.log.queue.put_nowait(None)
        except queue.Full:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
