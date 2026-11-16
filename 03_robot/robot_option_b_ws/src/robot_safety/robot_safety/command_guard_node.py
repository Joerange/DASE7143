"""50 Hz independent ROS command guard. No network or inference runs here."""
from dataclasses import fields, replace
import json
import math
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_msgs.msg import TFMessage

from .core import Config, Guard, Sample, scan_points, fresh, map_valid, dynamic_tf_valid
from .authorization import AuthorityConfig, MotionAuthority


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_xyyaw(pose):
    q = pose.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('invalid pose quaternion')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    value = (pose.position.x, pose.position.y, math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    if not all(map(math.isfinite, value)):
        raise ValueError('nonfinite pose')
    return value


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-8:
        raise ValueError('invalid quaternion')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    translation = np.array([t.x, t.y, t.z])
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError('nonfinite transform')
    return rotation, translation


class CommandGuardNode(Node):
    def __init__(self):
        super().__init__('command_guard_node')
        defaults = Config()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                  for f in fields(Config)}
        configured = Config(**values)
        runtime_capable = bool(self.declare_parameter('allow_runtime_control', False).value)
        heartbeat_timeout = float(self.declare_parameter('control_heartbeat_timeout', .5).value)
        chassis_timeout = float(self.declare_parameter('chassis_health_timeout', .5).value)
        # The old parameter grants neither capability nor authorization. Every
        # process starts disarmed; only this explicit console capability opts in.
        self.authority = MotionAuthority(AuthorityConfig(
            capable=runtime_capable,
            heartbeat_timeout=heartbeat_timeout, chassis_timeout=chassis_timeout))
        self.guard = Guard(replace(configured, motion_enabled=False))
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('depth_scale', 0.001)
        self.camera_info = None
        self.depth_lock = threading.Lock()
        self.depth_pending = None
        self.depth_stopping = threading.Event()
        self.dynamic_edges = {}
        self.pending_slam_pose = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Runtime-capable console sessions keep one final publisher, emitting
        # zeros until the operator explicitly grants a fresh runtime lease.
        self.output_topic = '/cmd_vel' if self.authority.config.capable else '/cmd_vel_guard_preview'
        self.output_pub = self.create_publisher(Twist, self.output_topic, 1)
        self.status_pub = self.create_publisher(String, '/safety/status', 1)
        self.stop_client = self.create_client(Trigger, '/mission/safety_stop')
        self.last_stop_logged = None
        self.stop_pending, self.stop_future, self.stop_requested = False, None, -math.inf
        self.create_service(SetBool, '/safety/set_motion_enabled', self.set_motion_enabled)
        self.create_subscription(String, '/console/control_heartbeat', self.on_control_heartbeat, 1)
        self.create_subscription(String, '/chassis/health', self.on_chassis_health, 1)
        self.create_subscription(String, '/mission/control_state', self.on_mission, 1)
        self.create_subscription(String, '/approach/speed_limit', self.on_limit, 1)
        self.create_subscription(String, '/target/estimates', self.on_targets, 1)
        self.create_subscription(Twist, '/cmd_vel_search', lambda m: self.on_command('search', m), 1)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.on_command('nav', m), 1)
        self.create_subscription(Twist, '/cmd_vel_nav_smoothed', lambda m: self.on_command('nav_smoothed', m), 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_dynamic_tf, qos_profile_sensor_data)
        if self.guard.config.localization_mode == 'amcl':
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_localization, 1)
        else:
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
            self.create_subscription(PoseWithCovarianceStamped, '/slam_toolbox/pose', self.on_slam_pose, 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self.on_depth, qos_profile_sensor_data)
        self.last_status = 0.0
        self.last_reason = None
        self.last_blocked_log = -math.inf
        # Wall/steady clock continues publishing zero when /clock pauses.
        self.timer = self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.depth_worker = threading.Thread(target=self.depth_loop, daemon=True)
        self.depth_worker.start()
        self.get_logger().info('Guard output: ' + self.output_topic)

    def authority_inputs(self, now, ros_now):
        ready, reason = self.guard.health(now, ros_now)
        mission_fresh = fresh(self.guard.mission, now, ros_now, self.guard.config.mission_timeout)
        mission = self.guard.mission.value if self.guard.mission else {}
        nav_ready = mission_fresh and mission.get('nav_ready') is True
        return ready, reason, mission_fresh, nav_ready

    def force_zero_and_stop(self):
        self.guard.config = replace(self.guard.config, motion_enabled=False)
        self.guard.commands.clear()
        self.guard.output, self.guard.blocked = (0., 0.), True
        self.output_pub.publish(Twist())
        # Persist the FIRST stop cause in the task log, before asynchronous
        # mission cancellation can replace the visible state with generic HOLD.
        if self.last_stop_logged != self.authority.session:
            self.last_stop_logged = self.authority.session
            try:
                now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9
                detail={'reason':self.authority.reason,'authority_session':self.authority.session,
                        'source':self.guard.identity[2] if self.guard.identity else None}
                for name in ('mission','scan','depth','odom','tf'):
                    sample=getattr(self.guard,name)
                    detail[name+'_age']=None if sample is None else round(ros_now-sample.stamp,4)
                    detail[name+'_receipt_age']=None if sample is None else round(now-sample.receipt,4)
                scan=self.guard.scan.value if self.guard.scan else None
                detail['scan_min_sector_fraction']=min(scan.get('sectors',[]),default=0.) if isinstance(scan,dict) else None
                self.get_logger().warning('MOTION_REVOKED '+json.dumps(detail,allow_nan=False))
            except Exception:
                self.get_logger().warning('MOTION_REVOKED '+self.authority.reason)
        self.stop_pending = True
        self.request_mission_stop(time.monotonic())

    def request_mission_stop(self, now):
        # Async and retried if the mission process is temporarily unavailable.
        # Final zero output never depends on its cancellation response.
        if self.stop_future is not None:
            if self.stop_future.done():
                try:
                    if self.stop_future.result().success:
                        self.stop_pending = False
                except Exception:
                    pass
                self.stop_future = None
            elif now-self.stop_requested > 1.:
                self.stop_future.cancel()
                self.stop_future = None
        if (self.stop_pending and self.stop_future is None and now-self.stop_requested >= .5
                and self.stop_client.service_is_ready()):
            self.stop_requested = now
            try:
                self.stop_future = self.stop_client.call_async(Trigger.Request())
            except Exception:
                self.stop_future = None

    def set_motion_enabled(self, request, response):
        if not request.data:
            # Emergency revoke does not inspect TF, camera, map, networking or
            # mission readiness. Publish zero before attempting async cancel.
            self.authority.revoke('operator_disarmed')
            self.force_zero_and_stop()
            response.success, response.message = True, self.authority.reason
            return response
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        try:
            self.resolve_slam_pose()
            self.check_tf(now, ros_now)
            inputs = self.authority_inputs(now, ros_now)
        except Exception as error:
            self.authority.revoke('authorization_check_failed:' + type(error).__name__)
            self.force_zero_and_stop()
            response.success, response.message = False, self.authority.reason
            return response
        mission = self.guard.mission.value if self.guard.mission else {}
        actual = self.guard.odom.value if self.guard.odom and self.guard.odom.value else (math.inf, math.inf)
        source_stopped = mission.get('source') == 'none' and not self.stop_pending and self.stop_future is None
        actual_stopped = abs(actual[0]) < .015 and abs(actual[1]) < .025
        was_authorized = self.authority.authorized
        accepted, reason = self.authority.request(bool(request.data), now, ros_now, *inputs,
                                                  source_stopped, actual_stopped)
        if not self.authority.authorized:
            self.force_zero_and_stop()
        elif not was_authorized:
            # A grant cannot resurrect a cached velocity from the previous lease.
            self.guard.commands.clear()
            self.guard.output, self.guard.blocked = (0., 0.), True
            self.guard.config = replace(self.guard.config, motion_enabled=True)
        response.success, response.message = accepted, reason
        return response

    def on_control_heartbeat(self, message):
        was_authorized = self.authority.authorized
        try:
            self.authority.update_heartbeat(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError):
            self.authority.heartbeat = None
            self.authority.revoke('control_heartbeat_invalid')
        if was_authorized and not self.authority.authorized:
            self.force_zero_and_stop()

    def on_chassis_health(self, message):
        was_authorized = self.authority.authorized
        try:
            self.authority.update_chassis(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError):
            self.authority.chassis = None
            self.authority.revoke('chassis_health_invalid')
        if was_authorized and not self.authority.authorized:
            self.force_zero_and_stop()

    def on_camera_info(self, message):
        self.camera_info = (message, time.monotonic())

    def on_mission(self, message):
        try:
            self.guard.update_mission(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError, AttributeError):
            self.guard.mission = None

    def on_limit(self, message):
        try:
            self.guard.speed_limit(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError, AttributeError):
            self.guard.limit = None

    def on_targets(self, message):
        try:
            self.guard.target_update(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError, AttributeError):
            self.guard.targets = None

    def on_command(self, source, message):
        components = [message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z]
        if not all(map(math.isfinite, components)) or any(abs(v) > 1e-6 for v in components[1:5]):
            self.guard.commands.pop(source, None)
            return
        self.guard.command(source, message.linear.x, message.angular.z, time.monotonic())

    def on_scan(self, message):
        value = None
        if message.header.frame_id == 'base_link':
            value = scan_points(message.ranges, message.angle_min, message.angle_increment,
                                message.range_min, message.range_max, self.guard.config)
        self.guard.scan = Sample(stamp_seconds(message.header.stamp), time.monotonic(), value)

    def on_odom(self, message):
        v, w = message.twist.twist.linear.x, message.twist.twist.angular.z
        good = (message.header.frame_id == 'odom' and message.child_frame_id == 'base_link'
                and all(map(math.isfinite, (v, w))))
        self.guard.odom = Sample(stamp_seconds(message.header.stamp), time.monotonic(), (v, w) if good else None)
        pose = None
        if good:
            try:
                pose = pose_xyyaw(message.pose.pose)
            except ValueError:
                pass
        self.guard.odom_pose = Sample(stamp_seconds(message.header.stamp), time.monotonic(), pose)

    def on_map(self, message):
        valid = False
        try:
            origin = pose_xyyaw(message.info.origin)
            valid = map_valid(message.info.width, message.info.height, message.info.resolution,
                              origin, message.data, message.header.frame_id)
        except (ValueError, TypeError):
            pass
        # Header is the most recent scan observed by SLAM Toolbox, even for an
        # unchanged stationary graph. Receipt alone cannot renew an old map.
        self.guard.map = Sample(stamp_seconds(message.header.stamp), time.monotonic(), valid)

    def on_slam_pose(self, message):
        stamp, receipt = stamp_seconds(message.header.stamp), time.monotonic()
        if self.guard.slam_pose is not None and stamp <= self.guard.slam_pose.stamp:
            return
        try:
            if message.header.frame_id != 'map':
                raise ValueError('wrong SLAM pose frame')
            pose_xyyaw(message.pose.pose)
        except (ValueError, TypeError):
            self.guard.slam_pose = None
            self.pending_slam_pose = None
            return
        # Retain the original acquisition and receipt times while waiting for
        # DDS to deliver the scan-time odometry TF to this independent node.
        self.pending_slam_pose = (message.header.stamp, stamp, receipt)
        self.resolve_slam_pose()

    def resolve_slam_pose(self):
        if self.pending_slam_pose is None:
            return
        message_stamp, stamp, receipt = self.pending_slam_pose
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', Time.from_msg(message_stamp)).transform
            rotation, translation = transform_matrix(transform)
            anchor = (float(translation[0]), float(translation[1]), math.atan2(rotation[1, 0], rotation[0, 0]))
            self.guard.slam_pose = Sample(stamp, receipt, anchor)
            self.pending_slam_pose = None
        except (TransformException, ValueError):
            pass

    def on_dynamic_tf(self, message):
        receipt = time.monotonic()
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            if key not in (('map', 'odom'), ('odom', 'base_link')):
                continue
            good = True
            try:
                transform_matrix(transform.transform)
            except ValueError:
                good = False
            self.dynamic_edges[key] = Sample(stamp_seconds(transform.header.stamp), receipt, good)

    def on_localization(self, message):
        covariance = message.pose.covariance
        variances = (covariance[0], covariance[7], covariance[35])
        valid = message.header.frame_id == 'map' and all(math.isfinite(v) and v >= 0 for v in variances)
        value = (max(variances[:2]), variances[2]) if valid else None
        self.guard.localization = Sample(stamp_seconds(message.header.stamp), time.monotonic(), value)

    def on_depth(self, message):
        # Keep only the newest frame; point projection cannot block the 50Hz guard.
        with self.depth_lock:
            self.depth_pending = (message, time.monotonic(), self.camera_info)

    def depth_loop(self):
        while not self.depth_stopping.wait(.01):
            with self.depth_lock:
                pending, self.depth_pending = self.depth_pending, None
            if pending is None:
                continue
            message, receipt, camera_info = pending
            try:
                value = self.project_depth(message, receipt, camera_info)
            except Exception:
                value = None
            # One atomic result replacement, preserving acquisition AND receipt.
            self.guard.depth = Sample(stamp_seconds(message.header.stamp), receipt, value)

    def project_depth(self, message, receipt, camera_info):
        stamp = stamp_seconds(message.header.stamp)
        if not camera_info:
            return
        info, info_receipt = camera_info
        if (receipt - info_receipt > 1.0 or info.header.frame_id != message.header.frame_id or
                (info.width, info.height) != (message.width, message.height) or
                abs(stamp_seconds(info.header.stamp) - stamp) > 0.1 or
                info.k[0] <= 0 or info.k[4] <= 0):
            return
        try:
            if message.encoding == '16UC1':
                dtype, scale = np.dtype('>u2' if message.is_bigendian else '<u2'), float(self.get_parameter('depth_scale').value)
                if not 0.0001 <= scale <= 0.01:
                    return
            elif message.encoding == '32FC1':
                dtype, scale = np.dtype('>f4' if message.is_bigendian else '<f4'), 1.0
            else:
                return
            # step includes row padding; do not reshape assuming a packed buffer.
            depth = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                               strides=(message.step, dtype.itemsize))[::8, ::8].astype(np.float32) * scale
            u, v = np.meshgrid(np.arange(0, message.width, 8), np.arange(0, message.height, 8))
            valid = np.isfinite(depth) & (depth >= 0.15) & (depth <= 8.0)
            if valid.mean() < 0.20:
                return
            pixels = np.column_stack((u[valid], v[valid])).astype(np.float32).reshape(-1, 1, 2)
            k, d = np.asarray(info.k).reshape(3, 3), np.asarray(info.d)
            if info.distortion_model in ('plumb_bob', 'rational_polynomial', ''):
                rays = cv2.undistortPoints(pixels, k, d if d.size else None).reshape(-1, 2)
            elif info.distortion_model == 'equidistant':
                rays = cv2.fisheye.undistortPoints(pixels, k, d).reshape(-1, 2)
            else:
                return
            points = np.column_stack((rays * depth[valid, None], depth[valid]))
            transform = self.tf_buffer.lookup_transform('base_link', message.header.frame_id,
                                                        Time.from_msg(message.header.stamp))
            rotation, translation = transform_matrix(transform.transform)
            points = points @ rotation.T + translation
            c = self.guard.config
            # Same measured chassis volume as the LiDAR self-filter. Do not
            # erase surrounding clearance or unmodelled objects above the body.
            body = ((np.abs(points[:, 0]) <= c.front) &
                    (np.abs(points[:, 1]) <= c.half_width) &
                    (points[:, 2] >= 0.) & (points[:, 2] <= c.body_height))
            # Covers low obstacles missed by the horizontal projected lidar.
            keep = (~body & (points[:, 2] > 0.06) & (points[:, 2] < 1.30) &
                    (np.linalg.norm(points[:, :2], axis=1) < 2.0))
            points = points[keep]
            return [(float(x), float(y)) for x, y, z in points]
        except (ValueError, TypeError, BufferError, TransformException, cv2.error):
            return

    def check_tf(self, now, ros_now):
        good = False
        self.guard.map_pose = None
        try:
            odom_base = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
            map_odom = self.tf_buffer.lookup_transform('map', 'odom', Time())
            self.tf_buffer.lookup_transform('map', 'base_link', Time())
            odom_rotation, odom_translation = transform_matrix(odom_base.transform)
            map_rotation, map_translation = transform_matrix(map_odom.transform)
            map_edge = self.dynamic_edges.get(('map', 'odom'))
            odom_edge = self.dynamic_edges.get(('odom', 'base_link'))
            good = dynamic_tf_valid(map_edge, odom_edge, now, ros_now, self.guard.config)
            if good:
                # Also check the exact transforms returned from the buffer;
                # direct receipt must not excuse a stale successful lookup.
                good = dynamic_tf_valid(
                    Sample(stamp_seconds(map_odom.header.stamp), map_edge.receipt, True),
                    Sample(stamp_seconds(odom_base.header.stamp), odom_edge.receipt, True),
                    now, ros_now, self.guard.config)
                if good:
                    position = map_rotation @ odom_translation + map_translation
                    rotation = map_rotation @ odom_rotation
                    pose = (float(position[0]), float(position[1]), math.atan2(rotation[1,0], rotation[0,0]))
                    self.guard.map_pose = Sample(stamp_seconds(odom_base.header.stamp), odom_edge.receipt, pose)
        except (TransformException, ValueError):
            pass
        self.guard.tf = Sample(ros_now, now, good)

    def tick(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        self.resolve_slam_pose()
        self.check_tf(now, ros_now)
        try:
            if self.authority.monitor(now, ros_now, *self.authority_inputs(now, ros_now)):
                self.force_zero_and_stop()
        except Exception:
            self.authority.revoke('authorization_health_error')
            self.force_zero_and_stop()
        self.guard.config = replace(self.guard.config, motion_enabled=self.authority.authorized)
        try:
            velocity, status = self.guard.tick(now, ros_now)
        except Exception as error:
            # An unexpected malformed input must not leave the last velocity active.
            velocity, status = self.guard._zero('guard_internal_error', False)
            if self.last_reason != 'guard_internal_error':
                self.get_logger().error('Guard failed closed: ' + type(error).__name__)
        if self.authority.authorized and status['reason'] in {
                'guard_internal_error', 'measured_overspeed', 'reverse_disabled',
                'search_translation_forbidden', 'scan_sector_unknown',
                'target_stale', 'target_identity_invalid', 'target_geometry_invalid'}:
            self.authority.revoke('guard:' + status['reason'])
            self.force_zero_and_stop()
            velocity = (0., 0.)
        # When sweep checking is enabled for the current motion, a collision
        # blocks velocity without losing operator authorization. Re-arm still
        # requires actual rest and a new command after the sweep becomes clear.
        status['motion_blocked'] = status['reason'] == 'obstacle_in_braking_sweep'
        status['collision'] = self.guard.collision_detail
        if status['motion_blocked'] and now-self.last_blocked_log >= 2.:
            self.last_blocked_log = now
            self.get_logger().warning('MOTION_BLOCKED '+json.dumps(self.guard.collision_detail,allow_nan=False))
        self.request_mission_stop(now)
        output = Twist()
        output.linear.x, output.angular.z = velocity
        self.output_pub.publish(output)
        if now - self.last_status >= 0.05 or status['reason'] != self.last_reason:
            status.update(self.authority.status())
            status['search_sweep_enabled'] = self.guard.config.search_sweep_enabled
            status['navigation_sweep_enabled'] = self.guard.config.navigation_sweep_enabled
            status['locked_target_navigation'] = self.guard.config.locked_target_navigation
            status['allow_target_refinement'] = self.guard.config.allow_target_refinement
            status['minimum_gap'] = self.guard.config.min_gap
            status['depth_required'] = self.guard.config.require_depth
            status['freshness_limits'] = {
                name: getattr(self.guard.config, name) for name in
                ('scan_timeout', 'depth_timeout', 'odom_timeout', 'tf_timeout',
                 'map_tf_timeout', 'map_timeout', 'target_timeout', 'command_timeout', 'mission_timeout')}
            status['output_zero'] = abs(velocity[0]) < 1e-9 and abs(velocity[1]) < 1e-9
            sample = self.guard.mission
            status['mission_health'] = {
                'decision_age': ros_now-sample.stamp if sample else None,
                'receipt_age': now-sample.receipt if sample else None,
                'topic': '/mission/control_state'}
            status['sensor_health'] = {
                name: bool(fresh(sample, now, ros_now, timeout) and sample.value is not None
                           and (sample.value.get('valid', False) if name == 'scan' else True))
                for name, sample, timeout in (
                    ('scan', self.guard.scan, self.guard.config.scan_timeout),
                    ('depth', self.guard.depth, self.guard.config.depth_timeout),
                    ('odom', self.guard.odom, self.guard.config.odom_timeout))}
            status.update(schema=1, stamp=ros_now,
                          task_generation=self.guard.identity[0] if self.guard.identity else '',
                          geometry_epoch=self.guard.mission.value.get('geometry_epoch', 0) if self.guard.mission else 0,
                          output_topic=self.output_topic,
                          localization_mode=self.guard.config.localization_mode)
            if self.guard.config.localization_mode == 'slam':
                status['sensor_health']['map'] = bool(fresh(self.guard.map, now, ros_now,
                    self.guard.config.map_timeout) and self.guard.map.value)
                status['sensor_health']['slam_pose_seen'] = bool(self.guard.slam_pose is not None)
                status['sensor_health']['tf'] = bool(self.guard.tf and self.guard.tf.value)
            message = String()
            message.data = json.dumps(status, allow_nan=False)
            self.status_pub.publish(message)
            self.last_status = now
        self.last_reason = status['reason']


def main(args=None):
    rclpy.init(args=args)
    node = CommandGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.depth_stopping.set()
        node.depth_worker.join(timeout=1.)
        # Best effort only; chassis firmware must handle process death.
        node.authority.revoke('process_shutdown')
        if rclpy.ok():
            node.output_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
