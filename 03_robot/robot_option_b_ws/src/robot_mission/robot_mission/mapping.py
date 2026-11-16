"""Online-map revisions and path ownership, independent of ROS transport."""
from dataclasses import dataclass, replace
import math
from .core import Goal, Grid, wrap


@dataclass(frozen=True)
class CachedPlan:
    goal: Goal
    cost: float
    path: tuple
    map_revision: int
    geometry_epoch: int

    def __iter__(self):
        yield self.goal
        yield self.cost


def classify_update(old, new, old_load_time=0., new_load_time=0.):
    """Origin translation/dimensions/content can change in an ordinary SLAM map.

    Reset evidence is coordinate-system change, a new nonzero load epoch, or
    overwhelming disappearance of previously known world-space support. A path
    becoming blocked is handled separately, without declaring a map reset.
    """
    if old is None:
        return 'initial'
    if (abs(old.resolution-new.resolution) > 1e-8 or
            abs(wrap(old.origin[2]-new.origin[2])) > 1e-4):
        return 'reset'
    if old_load_time > 0 and new_load_time > 0 and abs(old_load_time-new_load_time) > .001:
        return 'reset'
    known = [i for i, v in enumerate(old.data) if v >= 0]
    if len(known) >= 100:
        samples = known[::max(1, len(known)//2000)]
        lost = 0
        ox, oy, yaw = old.origin
        c, s = math.cos(yaw), math.sin(yaw)
        for i in samples:
            x, y = (i % old.width+.5)*old.resolution, (i//old.width+.5)*old.resolution
            if new.value_at(ox+c*x-s*y, oy+s*x+c*y) < 0:
                lost += 1
        if lost/len(samples) > .8 and sum(v >= 0 for v in new.data) < len(known)*.4:
            return 'reset'
    return 'update'


def remaining_path(path, robot_xy):
    if not path:
        return ()
    if robot_xy is None:
        return tuple(path)
    robot_xy = tuple(robot_xy[:2])
    if len(path) == 1:
        return (robot_xy, tuple(path[0][:2]))
    nearest = None
    for i, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        fraction = max(0., min(1., ((robot_xy[0]-a[0])*dx+(robot_xy[1]-a[1])*dy)/length2)) if length2 else 1.
        projection = (a[0]+fraction*dx, a[1]+fraction*dy)
        candidate = (math.dist(robot_xy, projection), -i, projection)
        if nearest is None or candidate[:2] < nearest[:2]:
            nearest = candidate
    _, negative_index, projection = nearest
    return (robot_xy, projection) + tuple(path[-negative_index+1:])


def plan_issue(plan, grid, geometry_epoch, robot_xy=None):
    if not isinstance(plan, CachedPlan) or plan.geometry_epoch != geometry_epoch:
        return {'reason': 'geometry_epoch_mismatch'}
    issue = grid.footprint_issue(plan.goal.x, plan.goal.y, plan.goal.yaw)
    if issue:
        return dict(issue, phase='docking')
    yaw = robot_xy[2] if robot_xy is not None and len(robot_xy) >= 3 else None
    return grid.path_issue(remaining_path(plan.path, robot_xy), yaw, plan.goal.yaw)


def plan_valid(plan, grid, geometry_epoch, robot_xy=None):
    return plan_issue(plan, grid, geometry_epoch, robot_xy) is None


def revalidate_cache(plans, grid, revision, geometry_epoch):
    return {key: replace(plan, map_revision=revision) for key, plan in plans.items()
            if plan_valid(plan, grid, geometry_epoch)}
