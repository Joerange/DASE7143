"""ROS-independent guard. All receipt times are monotonic; stamps are ROS time.

This is a software interlock, not a substitute for the chassis communication
watchdog. Killing this process must be covered by the chassis watchdog.
"""
from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class Config:
    motion_enabled: bool = False
    require_depth: bool = True
    search_sweep_enabled: bool = True
    navigation_sweep_enabled: bool = True
    locked_target_navigation: bool = False
    allow_target_refinement: bool = False
    max_target_refinement_step: float = .20
    max_goal_refinement_step: float = .30
    max_target_refinement_yaw: float = .25
    localization_mode: str = 'amcl'
    command_timeout: float = 0.20
    mission_timeout: float = 0.30
    scan_timeout: float = 0.40
    depth_timeout: float = 0.30
    odom_timeout: float = 0.30
    localization_timeout: float = 30.0
    map_timeout: float = 5.0
    slam_pose_timeout: float = 1.0
    slam_stationary_distance: float = 0.08
    slam_stationary_angle: float = 0.10
    map_tf_timeout: float = 0.60
    map_tf_future_tolerance: float = 0.35
    tf_timeout: float = 0.30
    target_timeout: float = 0.30
    max_linear: float = 0.10
    max_angular: float = 0.20
    linear_accel: float = 0.15
    angular_accel: float = 0.30
    braking_decel: float = 0.25
    angular_decel: float = 0.50
    reaction_time: float = 0.25
    front: float = 0.345
    half_width: float = 0.285
    body_height: float = 0.335
    clearance: float = 0.080
    min_gap: float = 0.55
    docking_stop_gap: float = .60
    max_xy_variance: float = 0.10
    max_yaw_variance: float = 0.15
    scan_min_fraction: float = 0.25
    scan_sector_fraction: float = 0.15

    def __post_init__(self):
        if self.localization_mode not in ('amcl', 'slam'):
            raise ValueError('localization_mode must be amcl or slam')
        for k, v in self.__dict__.items():
            if isinstance(v, (bool, str)):
                continue
            if not math.isfinite(v) or v <= 0:
                raise ValueError('Safety parameter must be positive and finite: ' + k)
        if self.max_linear > 0.25 or self.max_angular > 0.5:
            raise ValueError('Initial guard speeds exceed reviewed low-speed limits')


@dataclass
class Sample:
    stamp: float
    receipt: float
    value: object


def fresh(sample, now, ros_now, timeout):
    if sample is None:
        return False
    return (0 <= now - sample.receipt <= timeout and
            -0.05 <= ros_now - sample.stamp <= timeout)


def map_valid(width, height, resolution, origin, data, frame='map'):
    """A received empty/unknown/malformed grid is not localization evidence."""
    if (frame != 'map' or width <= 0 or height <= 0 or
            not math.isfinite(resolution) or resolution <= 0 or
            len(data) != width * height or len(origin) != 3 or
            not all(map(math.isfinite, origin))):
        return False
    return all(-1 <= v <= 100 for v in data) and any(0 <= v < 50 for v in data)


def dynamic_tf_valid(map_edge, odom_edge, now, ros_now, config):
    """Require actual dynamic /tf receipts AND transform acquisition times.

    TF lookup itself does not refresh these Samples. Static transforms and an
    indefinitely rebroadcast old header cannot make a dynamic chain healthy.
    SLAM Toolbox future-dates map->odom by its transform_timeout (.2 s).
    """
    if not fresh(odom_edge, now, ros_now, config.tf_timeout) or not odom_edge.value:
        return False
    if map_edge is None or not map_edge.value:
        return False
    future = config.map_tf_future_tolerance if config.localization_mode == 'slam' else 1.2
    maximum = config.map_tf_timeout if config.localization_mode == 'slam' else 1.5
    return (0 <= now-map_edge.receipt <= config.tf_timeout and
            -future <= ros_now-map_edge.stamp <= maximum)


def swept_collision(points, velocity, actual, config, detail=None):
    """Test full inflated footprint along commanded AND measured braking arcs.

    Sampling error is covered by extra padding computed from the fastest point
    on the rectangle. An obstacle in the starting footprint always stops.
    """
    radius = math.hypot(config.front, config.half_width)
    for v, w in (velocity, actual):
        horizon = config.reaction_time + max(abs(v) / config.braking_decel,
                                              abs(w) / config.angular_decel)
        reach = radius + config.clearance * 2 + abs(v) * horizon
        nearby = [(px, py) for px, py in points if px * px + py * py <= reach * reach]
        steps = max(2, int(math.ceil(horizon / 0.025)))
        step = horizon / steps
        extra = (abs(v) + abs(w) * radius) * step
        lx = config.front + config.clearance + extra
        ly = config.half_width + config.clearance + extra
        for i in range(steps + 1):
            t = i * step
            angle = w * t
            if abs(w) < 1e-8:
                ox, oy = v * t, 0.0
            else:
                ox, oy = v / w * math.sin(angle), v / w * (1 - math.cos(angle))
            ca, sa = math.cos(angle), math.sin(angle)
            for px, py in nearby:
                dx, dy = px - ox, py - oy
                if abs(ca * dx + sa * dy) <= lx and abs(-sa * dx + ca * dy) <= ly:
                    if detail is not None:
                        detail.update(point=[px, py], time_ahead=t,
                                      velocity=[v, w], footprint=[lx, ly])
                    return True
    return False


class Guard:
    STATE_SOURCES = {
        'SEARCH_CONTINUOUS': {'search'}, 'ACQUIRE': {'search', 'none'},
        'CONFIRM_PENDING': {'search', 'none'},
        'NAVIGATE': {'nav'}, 'APPROACH': {'nav'}, 'EXPLORE': {'nav'},
        'INIT': {'none'}, 'READY_TO_NAV': {'none'}, 'REOBSERVE_RECOVERY': {'none'},
        'VERIFY_STOP': {'none'}, 'WAIT_PATH': {'none'},
        'DONE': {'none'}, 'HOLD': {'none'}}

    def __init__(self, config=None):
        self.config = config or Config()
        self.mission = self.scan = self.depth = self.odom = self.localization = self.tf = None
        self.map = self.slam_pose = self.odom_pose = None
        self.map_pose = None
        self.locked_context = self.locked_anchor = None
        self.locked_revision = -1
        self.limit = self.targets = None
        self.commands = {}
        self.identity = None
        self.epoch = -1
        self.retired_generations = set()
        self.output = (0.0, 0.0)
        self.blocked = True
        self.last_tick = None
        self.collision_detail = None

    def update_mission(self, data, now):
        try:
            generation = data['task_generation']
            epoch = data['control_epoch']
            state, source = data['state'], data['source']
            if (data.get('schema') != 1 or not isinstance(generation, str) or not generation or
                    type(epoch) is not int or epoch < 0 or source not in self.STATE_SOURCES.get(state, ()) or
                    type(data.get('motion_enabled')) is not bool or
                    not math.isfinite(float(data['stamp']))):
                raise ValueError('mission schema')
            if generation in self.retired_generations:
                return False
            if self.identity and generation == self.identity[0]:
                if epoch < self.epoch or (epoch == self.epoch and source != self.identity[2]):
                    return False
            identity = (generation, epoch, source)
            decision_stamp = float(data.get('decision_stamp', data['stamp']))
            if not math.isfinite(decision_stamp):
                raise ValueError('mission decision stamp')
            if self.mission is not None and identity == self.identity and decision_stamp < self.mission.stamp:
                return False
            if identity != self.identity:
                if self.identity and generation != self.identity[0]:
                    self.retired_generations.add(self.identity[0])
                self.identity, self.epoch = identity, epoch
                self.commands.clear()
                self.limit = self.targets = None
                self.output = (0.0, 0.0)
                self.blocked = True
            # The compact control heartbeat carries the age of the actual
            # decision separately; repeated transport packets cannot renew it.
            self.mission = Sample(decision_stamp, now, dict(data))
            return True
        except (ValueError, KeyError, TypeError):
            self.mission = None
            return False

    def command(self, source, v, w, now):
        expected = 'nav' if source.startswith('nav') else source
        if not self.identity or expected != self.identity[2] or not all(map(math.isfinite, (v, w))):
            return False
        self.commands[source] = (now, self.identity, (v, w))
        return True

    def speed_limit(self, data, now):
        try:
            if (data.get('schema') != 1 or not self.identity or
                    data['task_generation'] != self.identity[0] or
                    data['control_epoch'] != self.identity[1] or
                    type(data['valid']) is not bool or not math.isfinite(float(data['max_linear']))):
                return
            self.limit = Sample(float(data['stamp']), now, dict(data))
        except (KeyError, TypeError, ValueError):
            self.limit = None

    def target_update(self, data, now):
        if data.get('schema') == 1 and self.identity and data.get('task_generation') == self.identity[0]:
            self.targets = Sample(float(data['stamp']), now, data.get('targets', []))

    def health(self, now, ros_now):
        c = self.config
        for name, sample, timeout in (
            ('scan', self.scan, c.scan_timeout), ('odom', self.odom, c.odom_timeout),
            ('tf', self.tf, c.tf_timeout)):
            if not fresh(sample, now, ros_now, timeout):
                return False, name + '_stale'
            if sample.value is None:
                return False, name + '_invalid'
        if not self.scan.value['valid']:
            return False, 'scan_invalid'
        if not self.tf.value:
            return False, 'tf_invalid'
        if c.localization_mode == 'amcl':
            if not fresh(self.localization, now, ros_now, c.localization_timeout):
                return False, 'localization_stale'
            if self.localization.value is None:
                return False, 'localization_invalid'
            xy, yaw = self.localization.value
            if not all(map(math.isfinite, (xy, yaw))) or min(xy, yaw) < 0 or xy > c.max_xy_variance or yaw > c.max_yaw_variance:
                return False, 'localization_uncertain'
        else:
            if not fresh(self.map, now, ros_now, c.map_timeout):
                return False, 'map_stale'
            if not self.map.value:
                return False, 'map_invalid'
            # Real accepted SLAM poses are distinct from TF rebroadcasts. A
            # stationary robot may legitimately have no new graph pose; only
            # allow this if current odometry remains at that accepted pose.
            if self.slam_pose is None or self.slam_pose.value is None:
                return False, 'slam_pose_missing'
            if not fresh(self.odom_pose, now, ros_now, c.odom_timeout) or self.odom_pose.value is None:
                return False, 'odom_pose_invalid'
            if self.slam_pose.stamp > ros_now + .05 or self.slam_pose.receipt > now:
                return False, 'slam_pose_invalid'
            anchor, current = self.slam_pose.value, self.odom_pose.value
            if (len(anchor) != 3 or len(current) != 3 or
                    not all(map(math.isfinite, (*anchor, *current)))):
                return False, 'slam_pose_invalid'
            distance = math.hypot(current[0]-anchor[0], current[1]-anchor[1])
            angle = abs(math.atan2(math.sin(current[2]-anchor[2]), math.cos(current[2]-anchor[2])))
            if (not fresh(self.slam_pose, now, ros_now, c.slam_pose_timeout) and
                    (distance > c.slam_stationary_distance or angle > c.slam_stationary_angle)):
                return False, 'slam_pose_not_progressing'
        if c.require_depth:
            if not fresh(self.depth, now, ros_now, c.depth_timeout):
                return False, 'depth_stale'
            if self.depth.value is None:
                return False, 'depth_invalid'
        return True, 'healthy'

    def _zero(self, reason, ready):
        self.commands.clear()
        self.output = (0.0, 0.0)
        self.blocked = True
        actual = self.odom.value if self.odom and self.odom.value else (float('inf'), float('inf'))
        return self.output, {'ready': ready, 'stopped': ready and abs(actual[0]) < 0.015 and abs(actual[1]) < 0.025,
                             'reason': reason}

    def locked_geometry(self, now, ros_now):
        """Derive range from the accepted map target and bounded revision history."""
        m = self.mission.value
        target = m.get('locked_target')
        if (not isinstance(target, dict) or target.get('track_id') != m.get('target_track_id')
                or not target.get('track_id') or type(m.get('geometry_epoch')) is not int
                or not fresh(self.map_pose, now, ros_now, self.config.odom_timeout)):
            return None
        try:
            point, goal = tuple(target['map_point']), tuple(target['goal'])
            stamp = float(target['observation_stamp'])
            if len(point) != 3 or len(goal) != 3 or not all(map(math.isfinite, (*point, *goal, stamp))) or stamp > ros_now+.05:
                return None
            context = (m['task_generation'], m['geometry_epoch'], target['track_id'])
            anchor = (point, goal, stamp)
            revision = target.get('target_revision', 0)
            if type(revision) is not int or revision < 0:
                return None
            if context != self.locked_context:
                self.locked_context, self.locked_anchor = context, anchor
                self.locked_revision = revision
            elif revision < self.locked_revision:
                return None
            elif revision == self.locked_revision:
                if anchor != self.locked_anchor:
                    return None
            else:
                old_point, old_goal, old_stamp = self.locked_anchor
                yaw_change = abs(math.atan2(math.sin(goal[2]-old_goal[2]), math.cos(goal[2]-old_goal[2])))
                if (not self.config.allow_target_refinement or m.get('navigation_mode') != 'adaptive_map'
                        or stamp <= old_stamp or math.dist(point[:2],old_point[:2]) > self.config.max_target_refinement_step
                        or math.dist(goal[:2],old_goal[:2]) > self.config.max_goal_refinement_step
                        or yaw_change > self.config.max_target_refinement_yaw):
                    return None
                self.locked_anchor, self.locked_revision = anchor, revision
            x, y, yaw = self.map_pose.value
            dx, dy = point[0]-x, point[1]-y
            return dict(gap=math.hypot(dx, dy)-self.config.front,
                        bearing=math.atan2(math.sin(math.atan2(dy, dx)-yaw), math.cos(math.atan2(dy, dx)-yaw)))
        except (KeyError, TypeError, ValueError):
            return None

    def tick(self, now, ros_now):
        c = self.config
        self.collision_detail = None
        dt = min(0.04, max(0.0, now - self.last_tick)) if self.last_tick is not None else 0.02
        self.last_tick = now
        ready, reason = self.health(now, ros_now)
        if not ready:
            return self._zero(reason, False)
        if not c.motion_enabled:
            return self._zero('disarmed', True)
        if not fresh(self.mission, now, ros_now, c.mission_timeout):
            return self._zero('mission_stale', True)
        m = self.mission.value
        if not m['motion_enabled'] or m['source'] == 'none':
            return self._zero('mission_stop', True)
        actual = self.odom.value
        if abs(actual[0]) > c.max_linear + 0.05 or abs(actual[1]) > c.max_angular + 0.10:
            return self._zero('measured_overspeed', True)
        if self.blocked:
            self.commands.clear()
            self.output = (0.0, 0.0)
            if abs(actual[0]) >= 0.015 or abs(actual[1]) >= 0.025:
                return self._zero('waiting_for_actual_stop', True)
            self.blocked = False
            return self.output, {'ready': True, 'stopped': True, 'reason': 'rearming'}
        source = m['source']
        names = ('nav', 'nav_smoothed') if source == 'nav' else ('search',)
        for name in names:
            command = self.commands.get(name)
            if not command or command[1] != self.identity or not 0 <= now - command[0] <= c.command_timeout:
                return self._zero(name + '_command_stale', True)
        v, w = self.commands[names[-1]][2]
        if v < -1e-6:
            return self._zero('reverse_disabled', True)
        if source == 'search' and abs(v) > 1e-6:
            return self._zero('search_translation_forbidden', True)
        maximum = c.max_linear
        if source == 'nav':
            if not fresh(self.limit, now, ros_now, c.target_timeout) or not self.limit.value['valid']:
                return self._zero('speed_limit_invalid', True)
            maximum = min(maximum, max(0.0, self.limit.value['max_linear']))
        locked = c.locked_target_navigation and m.get('navigation_mode') in ('locked_map','adaptive_map') and source == 'nav'
        if locked:
            geometry = self.locked_geometry(now, ros_now)
            if geometry is None:
                return self._zero('locked_target_invalid', True)
            if geometry['gap'] <= c.min_gap:
                return self._zero('target_minimum_gap', True)
            # Guard keeps its own range cap even when the camera no longer sees
            # the bin. This is explicitly a map estimate, never a depth sample.
            if m.get('navigation_mode') == 'adaptive_map' and fresh(self.targets, now, ros_now, c.target_timeout):
                track = m.get('observed_target_track_id') or m.get('target_track_id')
                for target in self.targets.value:
                    if (target.get('track_id') == track and target.get('valid') is True
                            and isinstance(target.get('gap'), (int,float)) and math.isfinite(target['gap'])
                            and isinstance(target.get('stamp'), (int,float)) and -.05 <= ros_now-target['stamp'] <= c.target_timeout
                            and target['gap'] <= c.min_gap):
                        return self._zero('target_minimum_gap', True)
            remaining = max(0., geometry['gap']-c.docking_stop_gap)
            latency = c.odom_timeout+c.command_timeout
            braking = math.sqrt((c.braking_decel*latency)**2+2*c.braking_decel*remaining)-c.braking_decel*latency
            maximum = min(maximum, max(0., braking))
        elif m['state'] == 'APPROACH':
            if not fresh(self.targets, now, ros_now, c.target_timeout):
                return self._zero('target_stale', True)
            candidates = [t for t in self.targets.value if t.get('track_id') == m.get('target_track_id')]
            if len(candidates) != 1:
                return self._zero('target_identity_invalid', True)
            target = candidates[0]
            try:
                ts, gap, bearing = float(target['stamp']), float(target['gap']), float(target['bearing'])
                if (target.get('valid') is not True or not all(map(math.isfinite, (ts, gap, bearing))) or
                        not -0.05 <= ros_now - ts <= c.target_timeout or abs(bearing) > 0.50):
                    return self._zero('target_geometry_invalid', True)
                if gap <= c.min_gap:
                    return self._zero('target_minimum_gap', True)
            except (KeyError, ValueError, TypeError):
                return self._zero('target_geometry_invalid', True)
        v = min(maximum, max(0.0, v))
        w = max(-c.max_angular, min(c.max_angular, w))
        turning_requested = abs(w) > 0.01
        # Check the velocity this cycle will actually issue, not the final
        # requested speed that acceleration limiting has not reached yet.
        pv, pw = self.output
        v = pv + max(-c.braking_decel * dt, min(c.linear_accel * dt, v - pv))
        w = pw + max(-c.angular_decel * dt, min(c.angular_accel * dt, w - pw))
        v = min(maximum, max(0.0, v))
        w = max(-c.max_angular, min(c.max_angular, w))
        sectors = self.scan.value['sectors']
        required = range(len(sectors)) if turning_requested or abs(actual[1]) > 0.01 else [
            i for i in range(len(sectors)) if abs(-math.pi + (i + 0.5) * 2 * math.pi / len(sectors)) <= math.radians(80)]
        if any(sectors[i] < c.scan_sector_fraction for i in required):
            return self._zero('scan_sector_unknown', True)
        # Operator-configured profiles can delegate navigation collision
        # checking to Nav2 while retaining this independent velocity watchdog.
        skip_search_sweep = (not c.search_sweep_enabled and source == 'search'
                             and abs(v) <= 1e-6 and abs(actual[0]) < 0.015)
        skip_navigation_sweep = not c.navigation_sweep_enabled and source == 'nav'
        if not (skip_search_sweep or skip_navigation_sweep):
            # A larger permitted scan gap must still be included in braking
            # distance, without assuming every fresh scan is 2.5 seconds old.
            sweep_config = replace(c, reaction_time=max(c.reaction_time,
                max(0., ros_now-self.scan.stamp)+c.command_timeout))
            sources = [('lidar', self.scan.value['points'])]
            if fresh(self.depth, now, ros_now, c.depth_timeout) and self.depth.value is not None:
                sources.append(('depth', self.depth.value))
            for sensor, points in sources:
                detail = {'sensor': sensor}
                if swept_collision(points, (v, w), actual, sweep_config, detail):
                    self.collision_detail = detail
                    return self._zero('obstacle_in_braking_sweep', True)
        self.output = (v, w)
        return self.output, {'ready': True, 'stopped': abs(v) < 1e-5 and abs(w) < 1e-5 and
                             abs(actual[0]) < 0.015 and abs(actual[1]) < 0.025, 'reason': 'authorized'}


def scan_points(ranges, angle_min, increment, range_min, range_max, config):
    """No return (Inf/NaN/zero) is unknown, never fabricated free space."""
    points, counts, totals = [], [0] * 36, [0] * 36
    if not ranges or not all(map(math.isfinite, (angle_min, increment, range_min, range_max))) or increment <= 0:
        return {'valid': False, 'points': [], 'sectors': [0.0] * 36}
    for i, distance in enumerate(ranges):
        angle = (angle_min + i * increment + math.pi) % (2 * math.pi) - math.pi
        sector = min(35, int((angle + math.pi) / (2 * math.pi) * 36))
        totals[sector] += 1
        if math.isfinite(distance) and max(0.01, range_min) <= distance <= range_max:
            counts[sector] += 1
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
    sectors = [n / t if t else 0.0 for n, t in zip(counts, totals)]
    return {'valid': len(points) / len(ranges) >= config.scan_min_fraction,
            'points': points, 'sectors': sectors}
