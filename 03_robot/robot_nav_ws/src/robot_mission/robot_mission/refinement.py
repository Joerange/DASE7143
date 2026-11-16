"""Bounded updates of one confirmed bin from matched YOLO/RGB-D observations."""
import math
from .core import Goal, finite, fresh, wrap


class TargetRefiner:
    def __init__(self):
        self.context = None
        self.last_observation = None
        self.reason = 'waiting_for_depth'

    def propose(self, task, targets, detector, pose, grid, ros_now):
        goal = task.nav_goal
        context = (task.generation, task.geometry_epoch, task.target)
        if context != self.context:
            self.context, self.last_observation = context, None
        if goal is None or pose is None or grid is None:
            return None
        rows = {row['track_id']: row for row in detector.get('candidates', []) if row.get('stable')}
        options = []
        for track, target in targets.items():
            point = target.get('map_point')
            if (not target.get('valid') or target.get('image_clipped',False)
                    or not fresh(target.get('stamp'), ros_now, task.timing.target_timeout)
                    or not isinstance(point, (list, tuple)) or len(point) != 3 or not finite(*point)):
                continue
            distance = math.dist(point[:2], goal.surface[:2])
            if distance > .35:
                continue
            row = rows.get(track)
            # A new tracker ID is only associated using a stable, same-frame
            # detection and a unique spatial match to the confirmed object.
            if track not in (task.target, task.observed_target):
                if not row or abs(detector.get('stamp', -math.inf)-target['stamp']) > .02:
                    continue
            options.append((distance, track, target))
        options.sort(key=lambda item: item[0])
        if not options:
            self.reason = 'waiting_for_depth'
            return None
        preferred = next((item for item in options if item[1] == task.observed_target), None)
        if preferred is None and len(options) > 1 and options[1][0]-options[0][0] < .12:
            self.reason = 'ambiguous_target'
            return None
        _, observed_track, target = preferred or options[0]
        point, stamp = tuple(target['map_point']), target['stamp']
        previous = self.last_observation
        if previous and stamp <= previous[2]:
            return None
        self.last_observation = (observed_track, point, stamp)
        if (previous is None or previous[0] != observed_track or stamp-previous[2] > 1.5
                or math.dist(point[:2], previous[1][:2]) > .12):
            self.reason = 'waiting_for_second_observation'
            return None
        if stamp <= goal.stamp:
            return None
        filtered = tuple(old+.35*(new-old) for old, new in zip(goal.surface, point))
        shift = math.dist(filtered[:2], goal.surface[:2])
        if shift > .08:
            ratio = .08/shift
            filtered = tuple(old+ratio*(new-old) for old, new in zip(goal.surface, filtered))
        desired_yaw = math.atan2(filtered[1]-pose[1], filtered[0]-pose[0])
        normal = target.get('surface_normal_map')
        if isinstance(normal, (list, tuple)) and len(normal) == 3 and finite(*normal):
            face_yaw = math.atan2(-normal[1], -normal[0])
            if abs(wrap(face_yaw-desired_yaw)) < math.pi/3:
                desired_yaw = face_yaw
        yaw = goal.yaw+max(-.10, min(.10, .5*wrap(desired_yaw-goal.yaw)))
        offset = .345+task.docking.target_gap+task.docking.goal_margin
        x, y = filtered[0]-offset*math.cos(yaw), filtered[1]-offset*math.sin(yaw)
        if math.hypot(x-goal.x, y-goal.y) < .01 and abs(wrap(yaw-goal.yaw)) < .025:
            self.reason = 'correction_below_deadband'
            return None
        if not grid.footprint_free(x, y, yaw):
            self.reason = 'updated_goal_not_known_free'
            return None
        self.reason = 'ready'
        return Goal(task.target, x, y, yaw, filtered, stamp), observed_track
