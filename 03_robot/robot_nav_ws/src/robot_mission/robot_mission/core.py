"""ROS-independent task, geometry and freshness rules. All times supplied by caller."""
from dataclasses import dataclass
import math
import uuid


def finite(*values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def fresh(stamp, now, limit):
    return finite(stamp, now) and -.05 <= now - stamp <= limit


@dataclass(frozen=True)
class Timing:
    """Acquisition/receipt budgets; task launch supplies the Jetson profile."""
    status_timeout: float = .3
    odom_timeout: float = .3
    tf_timeout: float = .5
    map_timeout: float = 5.
    detector_timeout: float = .8
    target_timeout: float = .3
    init_timeout: float = 30.
    recovery_timeout: float = 30.
    plan_timeout: float = 2.5
    plan_validity: float = 2.
    goal_timeout: float = 5.
    feedback_timeout: float = 3.
    cancel_timeout: float = 3.
    lifecycle_timeout: float = 2.5
    lifecycle_request_timeout: float = 2.

    def __post_init__(self):
        if any(not finite(v) or v <= 0 for v in self.__dict__.values()):
            raise ValueError('Mission timing budgets must be positive and finite')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Goal:
    track_id: str
    x: float
    y: float
    yaw: float
    surface: tuple
    stamp: float


@dataclass(frozen=True)
class Docking:
    target_gap: float = .60
    goal_margin: float = .015
    min_gap: float = .55
    gap_tolerance: float = .05
    position_tolerance: float = .04
    yaw_tolerance: float = .12
    terminal_zone: float = .45
    terminal_speed: float = .025

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if not finite(value) or value < 0 or (name != 'goal_margin' and value == 0):
                raise ValueError('invalid docking parameter: '+name)
        if self.min_gap > self.target_gap-self.gap_tolerance+1e-9:
            raise ValueError('minimum gap must be below the docking acceptance band')


class Grid:
    """Unknown/outside/occupied are blocked. Includes origin rotation and swept body."""
    def __init__(self, width, height, resolution, origin, data):
        if width <= 0 or height <= 0 or not finite(resolution) or resolution <= 0 or len(data) != width * height:
            raise ValueError('invalid occupancy grid')
        if not finite(*origin) or len(origin) != 3 or any(type(v) is not int or v < -1 or v > 100 for v in data):
            raise ValueError('invalid map origin or cells')
        self.width, self.height, self.resolution = width, height, resolution
        self.origin, self.data = origin, data

    def value_at(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        gx = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / self.resolution)
        gy = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / self.resolution)
        return self.data[gy*self.width+gx] if 0 <= gx < self.width and 0 <= gy < self.height else -1

    def free(self, x, y):
        return 0 <= self.value_at(x, y) < 50

    def disk_free(self, x, y, radius=math.hypot(.395, .335)):
        # Circumscribed swept footprint plus half a cell prevents sampling gaps.
        r = radius + self.resolution * math.sqrt(2) / 2
        n = math.ceil(r / self.resolution)
        for ix in range(-n, n+1):
            for iy in range(-n, n+1):
                dx, dy = ix*self.resolution, iy*self.resolution
                if math.hypot(dx, dy) <= r and not self.free(x+dx, y+dy):
                    return False
        return True

    def footprint_issue(self, x, y, yaw, padding=0.):
        """Intersect map cells with the oriented Nav2 body plus 5 cm padding.

        Check the cells actually touched by the rectangle, rather than its
        circumscribed disk. Cell intersection also covers map-origin rotation.
        """
        ox, oy, origin_yaw = self.origin
        co, so = math.cos(origin_yaw), math.sin(origin_yaw)
        gx, gy = co*(x-ox)+so*(y-oy), -so*(x-ox)+co*(y-oy)
        c, s = math.cos(yaw-origin_yaw), math.sin(yaw-origin_yaw)
        length, width = .395+padding, .335+padding
        ex, ey = abs(c)*length+abs(s)*width, abs(s)*length+abs(c)*width
        half_cell = self.resolution/2
        cell_projection = half_cell*(abs(c)+abs(s))
        for ix in range(math.floor((gx-ex)/self.resolution), math.floor((gx+ex)/self.resolution)+1):
            for iy in range(math.floor((gy-ey)/self.resolution), math.floor((gy+ey)/self.resolution)+1):
                cx, cy = (ix+.5)*self.resolution, (iy+.5)*self.resolution
                dx, dy = cx-gx, cy-gy
                if (abs(c*dx+s*dy) > length+cell_projection or
                        abs(-s*dx+c*dy) > width+cell_projection):
                    continue
                value = self.data[iy*self.width+ix] if 0 <= ix < self.width and 0 <= iy < self.height else -1
                if not 0 <= value < 50:
                    return dict(reason='unknown_cell' if value < 0 else 'occupied_cell',
                                cell=[ix, iy], occupancy=value,
                                point=[ox+co*cx-so*cy, oy+so*cx+co*cy], pose=[x, y, yaw])
        return None

    def footprint_free(self, x, y, yaw):
        return self.footprint_issue(x, y, yaw) is None

    def path_issue(self, points, start_yaw=None, goal_yaw=None):
        if not points:
            return {'reason': 'empty_path'}
        route = []
        for point in points:
            if not route or math.dist(route[-1], point[:2]) > 1e-8:
                route.append(tuple(point[:2]))
        headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a, b in zip(route, route[1:])]
        yaw = start_yaw if start_yaw is not None else headings[0] if headings else goal_yaw or 0.
        radius = math.hypot(.395, .335)

        def turn_issue(point, before, after):
            angle = wrap(after-before)
            count = max(1, math.ceil(abs(angle)/.04))
            padding = radius*abs(angle)/count/2
            for i in range(count+1):
                issue = self.footprint_issue(*point, before+angle*i/count, padding)
                if issue:
                    return dict(issue, phase='turn')
            return None

        for start, end, heading in zip(route, route[1:], headings):
            issue = turn_issue(start, yaw, heading)
            if issue:
                return issue
            distance = math.dist(start, end)
            count = max(1, math.ceil(distance/(self.resolution/2)))
            for i in range(count+1):
                f = i/count
                issue = self.footprint_issue(start[0]+f*(end[0]-start[0]),
                                             start[1]+f*(end[1]-start[1]), heading, distance/count/2)
                if issue:
                    return dict(issue, phase='translation')
            yaw = heading
        return turn_issue(route[-1], yaw, goal_yaw if goal_yaw is not None else yaw)

    def path_free(self, points, start_yaw=None, goal_yaw=None):
        return self.path_issue(points, start_yaw, goal_yaw) is None


def docking_goals(target, robot_xy, grid, gap=.60, margin=.015, min_gap=.55, diagnostics=None):
    """Use only the observed face; never infer an unseen rear surface."""
    p = target.get('map_point', [])
    if not target.get('valid') or len(p) != 3 or not finite(*p, *robot_xy):
        if diagnostics is not None:
            diagnostics.update(reason='invalid_target_geometry')
        return []
    dx, dy = p[0]-robot_xy[0], p[1]-robot_xy[1]
    if math.hypot(dx, dy) < .345 + min_gap:
        if diagnostics is not None:
            diagnostics.update(reason='target_too_close')
        return []  # Never automatically back away from a target already too close.
    yaw = math.atan2(dy, dx)
    normal = target.get('surface_normal_map')
    if isinstance(normal, (list, tuple)) and len(normal) == 3 and finite(*normal):
        face_yaw = math.atan2(-normal[1], -normal[0])
        if abs(wrap(face_yaw-yaw)) < math.pi/3:
            yaw = face_yaw
    offset = .345 + gap + margin
    # Surface is observed from current direction; lateral alternatives require
    # their own observations. Planner remains free to route around obstacles.
    x, y = p[0]-offset*math.cos(yaw), p[1]-offset*math.sin(yaw)
    issue = grid.footprint_issue(x, y, yaw)
    if issue:
        if diagnostics is not None:
            diagnostics.update(reason='docking_'+issue['reason'], goal=[x,y,yaw], issue=issue)
        return []
    return [Goal(target['track_id'], x, y, yaw, tuple(p), target['stamp'])]


def path_length(points):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))


def approach_limit(gap, latency=.30, deceleration=.15, speed_scale=1., max_linear=None,
                   stop_gap=.60, terminal_zone=0., terminal_speed=.025):
    """Continuous braking envelope above the configured stop gap."""
    if not finite(gap) or gap <= stop_gap:
        return 0.
    available = max(0., gap - stop_gap)
    braking = max(0., math.sqrt((deceleration*latency)**2 + 2*deceleration*available) - deceleration*latency)
    cruise_cap = .10*speed_scale if max_linear is None else max_linear
    # Keep the approach cap at least the configured terminal cap so entering
    # the final zone cannot unexpectedly raise the permitted speed.
    near_cap = max(.05*speed_scale, terminal_speed if terminal_zone > 0 else 0.)
    zone_cap = min(cruise_cap, near_cap) if gap <= 1.4 else cruise_cap
    if gap <= terminal_zone:
        zone_cap = min(zone_cap, terminal_speed)
    return min(zone_cap, braking)


def locked_target_geometry(goal, pose, stamp):
    """Map estimate from a frozen observation and current pose, not new RGB-D."""
    if goal is None or pose is None or not finite(*pose, stamp, *goal.surface):
        return None
    dx, dy = goal.surface[0]-pose[0], goal.surface[1]-pose[1]
    return dict(source='locked_map', stamp=stamp, observation_stamp=goal.stamp,
                gap=math.hypot(dx, dy)-.345, bearing=wrap(math.atan2(dy, dx)-pose[2]),
                goal_distance=math.dist(pose[:2], (goal.x, goal.y)),
                heading_error=wrap(goal.yaw-pose[2]))


class Mission:
    def __init__(self, timeout=180., confirmation_timeout=8., timing=None, lock_target_on_navigation=False,
                 docking=None, visual_refinement=False):
        self.timeout, self.confirmation_timeout = timeout, confirmation_timeout
        self.timing = timing or Timing()
        self.lock_target_on_navigation = lock_target_on_navigation
        self.docking = docking or Docking()
        self.visual_refinement = visual_refinement
        self.observed_target = ''
        self.completion_evidence = ''
        self.generation = str(uuid.uuid4())
        self.state, self.reason, self.source, self.epoch = 'INIT', 'waiting for /mission/start', 'none', 0
        self.active = False
        self.deadline = math.inf
        self.pending = None
        self.confirmed = {}
        self.target = ''
        self.confirmed_at = -math.inf
        self.stopped_since = None
        self.coverage = 0.
        self.last_yaw = None
        self.recovery_deadline = math.inf
        self.nav_finished = False
        self.nav_goal = None
        self.geometry_epoch = 0
        self.reconfirm_required = False
        self.observation_after = -math.inf
        self.ready_since = 0.
        self.recovering = False

    def transition(self, state, reason=''):
        source = 'search' if state in ('SEARCH_CONTINUOUS', 'ACQUIRE') else 'nav' if state in ('NAVIGATE', 'APPROACH') else 'none'
        if source != self.source:
            self.epoch += 1
        self.state, self.reason, self.source = state, reason, source

    def start(self, now):
        self.generation = str(uuid.uuid4())
        self.pending, self.confirmed, self.target = None, {}, ''
        self.stopped_since, self.last_yaw, self.nav_goal = None, None, None
        self.coverage, self.nav_finished = 0., False
        self.confirmed_at = -math.inf
        self.reconfirm_required, self.recovering = False, False
        self.completion_evidence = ''
        self.observed_target = ''
        self.observation_after = -math.inf
        self.active, self.deadline = True, now+self.timeout
        self.init_deadline = now+self.timing.init_timeout
        self.transition('INIT', 'waiting for current sensors, localization and Nav2')

    def hold(self, reason):
        self.pending = None
        self.active = False
        self.transition('HOLD', reason)

    def candidate_event(self, event, now):
        if (not self.active or self.state not in ('SEARCH_CONTINUOUS', 'REOBSERVE_RECOVERY') or
                (self.state == 'REOBSERVE_RECOVERY' and not self.reconfirm_required) or
                event.get('task_generation') != self.generation):
            return False
        cs = event.get('candidates', [])
        if not cs or not event.get('request_id') or len(cs) > 6:
            return False
        ids = [c.get('id') for c in cs]
        tracks = [c.get('track_id') for c in cs]
        if len(set(ids)) != len(ids) or len(set(tracks)) != len(tracks) or not all(tracks):
            return False
        self.pending = {'request_id': event['request_id'], 'observation_id': event.get('observation_id'),
                        'candidates': {c['id']: c['track_id'] for c in cs}, 'deadline': now+self.confirmation_timeout,
                        'response': None}
        if self.state == 'REOBSERVE_RECOVERY':
            self.transition('CONFIRM_PENDING', 'fresh recovery candidate; confirmation and planning while stopped')
        else:
            self.transition('ACQUIRE', 'candidate snapshot frozen; stopping while confirmation and planning run')
        return True

    def confirmation(self, response, now):
        p = self.pending
        if not p or now > p['deadline'] or response.get('task_generation') != self.generation:
            return False
        if response.get('request_id') != p['request_id'] or response.get('observation_id') != p['observation_id']:
            return False
        cs = response.get('candidates', [])
        if not response.get('valid') or len(cs) != len(p['candidates']):
            p['response'] = {}
            return False
        out = {}
        for c in cs:
            i, track, status = c.get('id'), c.get('track_id'), c.get('status')
            if i in out or i not in p['candidates'] or track != p['candidates'][i] or status not in ('match', 'no_match', 'uncertain'):
                p['response'] = {}
                return False
            out[i] = (track, status)
        p['response'] = out
        return True

    def update_yaw(self, yaw, valid_frame):
        if self.state == 'SEARCH_CONTINUOUS' and valid_frame and self.last_yaw is not None:
            delta = abs(wrap(yaw-self.last_yaw))
            if delta < .15:  # localization jumps do not count as observed coverage
                self.coverage += delta
        self.last_yaw = yaw

    def recover(self, reason, now, ros_now, reconfirm=False):
        """Revoke immediately; adapter cancels action and clears geometry/path caches."""
        if not self.active:
            return
        self.recovering = True
        self.pending, self.nav_goal, self.nav_finished = None, None, False
        self.stopped_since = None
        self.observation_after = ros_now
        self.recovery_deadline = min(self.deadline, now+self.timing.recovery_timeout)
        self.reconfirm_required = reconfirm
        if reconfirm:
            self.geometry_epoch += 1
            self.generation = str(uuid.uuid4())
            self.confirmed, self.target = {}, ''
            self.confirmed_at = -math.inf
        self.transition('REOBSERVE_RECOVERY', reason)

    def planning_tracks(self):
        if self.pending:
            return list(self.pending['candidates'].values())
        if self.state in ('WAIT_PATH', 'REOBSERVE_RECOVERY') and not self.reconfirm_required:
            return list(self.confirmed)
        return []

    def choose_plan(self, now, ros_now, targets, plans):
        eligible = []
        for track in self.confirmed:
            t, plan = targets.get(track), plans.get(track)
            if t and t.get('valid') and fresh(t.get('stamp'), ros_now, self.timing.target_timeout) and t['stamp'] > self.observation_after and plan:
                goal, cost = plan
                if fresh(goal.stamp, ros_now, self.timing.plan_validity) and goal.stamp > self.observation_after and math.dist(goal.surface, t['map_point']) < .12:
                    eligible.append((cost, track, goal))
        if eligible:
            _, self.target, self.nav_goal = min(eligible)
            self.observed_target = self.target
            self.nav_finished = False
            self.ready_since = now
            self.transition('READY_TO_NAV', 'identity, standstill, current geometry and known-free path accepted')
            return True
        return False

    def tick(self, now, ros_now, ready, stopped, targets, plans, detection_fresh=True, navigation_idle=True,
             locked_geometry=None):
        if not self.active:
            return
        if now >= self.deadline:
            self.hold('mission deadline exceeded')
            return
        if self.state == 'INIT':
            if ready and detection_fresh and navigation_idle:
                self.transition('SEARCH_CONTINUOUS', 'initial local map ready; continuous local search')
            elif now > self.init_deadline:
                self.hold('initialization timeout')
            return
        if self.state == 'REOBSERVE_RECOVERY':
            if now > self.recovery_deadline:
                self.hold('bounded reobservation/relocalization failed')
            elif ready and stopped and navigation_idle and not self.reconfirm_required:
                if any(t.get('valid') and fresh(t.get('stamp'), ros_now, self.timing.target_timeout) and t['stamp'] > self.observation_after
                       for k, t in targets.items() if k in self.confirmed):
                    self.transition('WAIT_PATH', 'fresh target recovered; waiting for a newly validated path')
                    self.choose_plan(now, ros_now, targets, plans)
            return
        if not ready:
            self.hold('sensor, localization or safety readiness lost')
            return
        if self.state == 'SEARCH_CONTINUOUS':
            if not detection_fresh:
                self.hold('detector stale')
            elif self.coverage >= 2*math.pi:
                self.hold('full observed search completed without an accepted reachable target')
            return
        if self.state in ('ACQUIRE', 'CONFIRM_PENDING'):
            if self.state == 'ACQUIRE' and stopped:
                self.transition('CONFIRM_PENDING', 'waiting for confirmed identity, current geometry and standstill')
            p = self.pending
            if not p or now > p['deadline']:
                self.pending = None
                if self.recovering:
                    self.transition('REOBSERVE_RECOVERY', 'confirmation expired; remain stopped for fresh evidence')
                else:
                    self.transition('SEARCH_CONTINUOUS', 'candidate event expired')
                return
            if p['response'] is None or not stopped or not navigation_idle:
                return
            matches = [track for track, status in p['response'].values() if status == 'match']
            if not matches:
                self.pending = None
                if self.recovering:
                    self.transition('REOBSERVE_RECOVERY', 'confirmation unsuccessful; remain stopped')
                else:
                    self.transition('SEARCH_CONTINUOUS', 'no valid confirmed candidate; resume remaining coverage')
                return
            # Preserve confirmed identity independently of the API request deadline.
            # Seeing a bin with an unconfirmed path MUST NOT resume rotation/movement.
            self.confirmed = {t: now for t in matches}
            self.confirmed_at = now
            self.pending = None
            self.reconfirm_required = False
            self.transition('WAIT_PATH', 'confirmed target; parked until current geometry and known-free path exist')
            self.choose_plan(now, ros_now, targets, plans)
            return
        if self.state == 'WAIT_PATH':
            if now-self.confirmed_at > 90.:
                self.hold('semantic target expired while waiting for known-free path')
            elif stopped and navigation_idle:
                self.choose_plan(now, ros_now, targets, plans)
            return
        if self.state == 'READY_TO_NAV':
            if now-self.ready_since > self.timing.goal_timeout:
                self.hold('navigation goal acceptance timeout')
            return
        if self.state in ('NAVIGATE', 'APPROACH', 'VERIFY_STOP'):
            if self.lock_target_on_navigation and self.nav_goal is not None:
                self.tick_locked(now, ros_now, stopped, navigation_idle, locked_geometry, targets)
                return
            if now-self.confirmed_at > 90.:
                self.hold('semantic target record expired')
                return
            t = targets.get(self.target)
            current = bool(t and t.get('valid') and fresh(t.get('stamp'), ros_now, self.timing.target_timeout))
            if current and self.nav_goal and math.dist(t['map_point'], self.nav_goal.surface) > .25:
                self.recover('target geometry conflicts with previous map record', now, ros_now, reconfirm=True)
                return
            if self.state == 'NAVIGATE':
                if current and t['gap'] <= 1.4:
                    self.transition('APPROACH', 'continuous Nav2 with current range supervision')
                elif self.nav_finished:
                    self.hold('Nav2 ended without current valid docking range')
                    return
            if self.state in ('APPROACH', 'VERIFY_STOP'):
                if not current:
                    self.recover('current target or depth lost; movement revoked', now, ros_now)
                    return
                if t['gap'] < self.docking.min_gap:
                    self.hold('target inside minimum docking gap; no automatic reversing')
                    return
                arrived = (abs(t['gap']-self.docking.target_gap) <= self.docking.gap_tolerance
                           and abs(t['bearing']) <= self.docking.yaw_tolerance)
                if self.state == 'APPROACH' and arrived:
                    self.transition('VERIFY_STOP', 'arrival latched; cancel remaining navigation')
                if self.state == 'VERIFY_STOP':
                    if not arrived:
                        self.hold('latched arrival failed geometric verification')
                    elif stopped and navigation_idle:
                        self.stopped_since = now if self.stopped_since is None else self.stopped_since
                        if now-self.stopped_since >= .7:
                            self.active = False
                            self.transition('DONE', 'current identity, configured docking gap and measured standstill verified')
                    else:
                        self.stopped_since = None
                elif self.nav_finished:
                    self.hold('Nav2 succeeded but physical gap/bearing acceptance failed')

    def tick_locked(self, now, ros_now, stopped, navigation_idle, geometry, targets):
        if not geometry or not fresh(geometry.get('stamp'), ros_now, self.timing.odom_timeout):
            self.hold('current pose unavailable for locked target navigation')
            return
        gap = geometry['gap']
        dock = self.docking
        if gap < dock.min_gap:
            self.hold('locked target minimum gap reached; no automatic reversing')
            return
        if self.state == 'NAVIGATE' and gap <= 1.4:
            self.transition('APPROACH', 'approaching docking goal; visual corrections enabled' if self.visual_refinement else
                            'approaching locked map goal; visual reacquisition is optional')
        live = targets.get(self.observed_target or self.target, {})
        current = bool(live.get('valid') and not live.get('image_clipped',False)
                       and fresh(live.get('stamp'), ros_now, self.timing.target_timeout)
                       and math.dist(live.get('map_point', (math.inf,)*3)[:2], self.nav_goal.surface[:2]) <= .35)
        low, high = dock.target_gap-dock.gap_tolerance, dock.target_gap+dock.gap_tolerance
        measured = (current and low <= live.get('gap', math.inf) <= high
                    and abs(live.get('bearing', math.inf)) <= dock.yaw_tolerance
                    and abs(live.get('base_point', (math.inf,)*3)[1]) <= .03
                    and math.dist(live.get('map_point', (math.inf,)*3)[:2], self.nav_goal.surface[:2]) <= .35)
        normal = live.get('surface_normal_base')
        if measured and isinstance(normal,(list,tuple)) and len(normal)==3 and finite(*normal):
            measured = abs(math.atan2(-normal[1],-normal[0])) <= dock.yaw_tolerance
        arrived = (low <= gap <= high and geometry['goal_distance'] <= dock.position_tolerance
                   and abs(geometry['heading_error']) <= dock.yaw_tolerance
                   and not geometry.get('goal_update_pending', False))
        if self.visual_refinement and current and self.state != 'VERIFY_STOP':
            arrived = arrived and measured
        if self.state in ('NAVIGATE', 'APPROACH') and arrived:
            self.transition('VERIFY_STOP', 'locked docking pose reached; cancel navigation and verify standstill')
        if self.state == 'VERIFY_STOP':
            if not arrived:
                self.hold('locked docking pose moved outside arrival tolerance')
            elif stopped and navigation_idle:
                self.stopped_since = now if self.stopped_since is None else self.stopped_since
                if now-self.stopped_since >= .7:
                    self.completion_evidence = 'rgbd_and_map' if measured else 'locked_map_only'
                    self.active = False
                    self.transition('DONE', 'locked docking goal reached and measured standstill verified; '+
                                    ('live RGB-D gap verified' if measured else 'physical gap not remeasured'))
            else:
                self.stopped_since = None
        elif self.nav_finished:
            if self.visual_refinement:
                self.reason = 'at previous goal; waiting for visual docking correction'
            else:
                self.hold('Nav2 ended outside locked docking pose tolerance')

    def navigation_accepted(self):
        if self.active and self.state == 'READY_TO_NAV':
            self.transition('NAVIGATE', 'Nav2 goal accepted; RGB-D target refinement enabled' if self.visual_refinement else
                            'Nav2 goal accepted; target fixed in map' if self.lock_target_on_navigation else 'Nav2 goal accepted')

    def navigation_result(self, succeeded):
        if self.state in ('NAVIGATE', 'APPROACH'):
            if succeeded:
                self.nav_finished = True
            else:
                self.hold('Nav2 aborted or canceled before verified arrival')
