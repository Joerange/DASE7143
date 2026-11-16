"""Inspectable world-coordinate view memory and reachable viewpoint selection."""
from dataclasses import dataclass, asdict
import heapq
import math
from .core import Goal, wrap

@dataclass
class View:
    x: float
    y: float
    yaw: float
    stamp: float
    outcome: str
    request_id: str

class ViewMemory:
    def __init__(self, cell=.5, sectors=12, cooldown=60.):
        self.cell, self.sectors, self.cooldown = cell, sectors, cooldown
        self.views = {}
        self.excluded = {}
        self.history = []

    def key(self, pose):
        return (math.floor(pose[0]/self.cell), math.floor(pose[1]/self.cell),
                round((pose[2] % (2*math.pi))*self.sectors/(2*math.pi)) % self.sectors)

    def record(self, pose, stamp, outcome, request_id):
        view = View(*pose, stamp, outcome, request_id)
        self.history.append(view)
        if outcome in ('not_found', 'found'):
            self.views[self.key(pose)] = view

    def reject(self, pose, now, reason):
        self.excluded[self.key(pose)] = (now, reason)

    def blocked(self, pose, now, revisit=False):
        key = self.key(pose)
        failed = self.excluded.get(key)
        if failed and now-failed[0] < self.cooldown:
            return True
        view = self.views.get(key)
        return bool(view and (not revisit or now-view.stamp < self.cooldown))

    def snapshot(self):
        return dict(views=[asdict(v) for v in self.history],
                    excluded=[dict(key=list(k), stamp=v[0], reason=v[1]) for k,v in self.excluded.items()])


def visible_gain(grid, memory, pose, radius=3., fov=math.radians(65)):
    """Ray-cast camera fan; occupied cells occlude and unknown cells end rays."""
    unknown, visual = set(), set()
    for ray in range(15):
        a = pose[2]-fov/2+fov*ray/14
        for step in range(1, int(radius/.15)+1):
            x, y = pose[0]+step*.15*math.cos(a), pose[1]+step*.15*math.sin(a)
            v = grid.value_at(x,y)
            if v >= 50:
                break
            key = (math.floor(x/memory.cell), math.floor(y/memory.cell))
            if v < 0:
                unknown.add(key)
                break
            # Coverage is directional: a known LiDAR cell can still need a new view.
            if not any(math.dist((x,y),(p.x,p.y)) < radius and
                       abs(wrap(math.atan2(y-p.y,x-p.x)-p.yaw)) <= fov/2 and
                       line_visible(grid,(p.x,p.y),(x,y)) for p in memory.views.values()):
                visual.add(key)
    return len(unknown), len(visual)


def line_visible(grid, a, b):
    n = max(1,math.ceil(math.dist(a,b)/max(.05,grid.resolution)))
    return all(grid.free(a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n) for i in range(1,n+1))


def reachable_paths(grid, pose, spacing=.5, max_distance=4.):
    """Bounded Dijkstra on a world lattice, with full swept-body checks."""
    start=(0,0); distances={start:0.}; parents={}; heap=[(0.,start)]
    points={start:pose[:2]}; headings={start:pose[2]}
    while heap:
        cost,k=heapq.heappop(heap)
        if cost != distances[k]:continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nxt=(k[0]+dx,k[1]+dy)
            nc=cost+spacing
            if nc>max_distance or nc>=distances.get(nxt,math.inf):continue
            point=(pose[0]+nxt[0]*spacing,pose[1]+nxt[1]*spacing)
            yaw=math.atan2(dy,dx)
            if not grid.path_free((points[k],point),headings[k],yaw):continue
            distances[nxt]=nc;parents[nxt]=k;points[nxt]=point;headings[nxt]=yaw
            heapq.heappush(heap,(nc,nxt))
    for k,cost in distances.items():
        if k==start:continue
        route=[]; cursor=k
        while cursor!=start:
            route.append(points[cursor]);cursor=parents[cursor]
        route.append(points[start]);route.reverse()
        yield points[k],cost,tuple(route),headings[k]


def choose_view(grid,memory,pose,now,max_distance=4.):
    routes=list(reachable_paths(grid,pose,memory.cell,max_distance))
    for revisit in (False,True):
        candidates=[]
        for xy,cost,path,heading in routes:
            for sector in range(memory.sectors):
                yaw=sector*2*math.pi/memory.sectors
                p=(*xy,yaw)
                if memory.blocked(p,now,revisit) or not grid.path_free((xy,),heading,yaw):continue
                unknown,visual=visible_gain(grid,memory,p)
                if unknown+visual==0:continue
                score=2*unknown/15+visual/40-cost/max_distance-.1*abs(wrap(yaw-pose[2]))/math.pi
                candidates.append((score,p,path,unknown,visual,cost))
        if candidates:
            score,p,path,u,v,cost=max(candidates,key=lambda c:c[0])
            goal=Goal('__explore__',*p,(p[0],p[1],0.),now)
            return goal,path,dict(score=score,unknown_gain=u,visual_gain=v,path_length=cost,revisit=revisit)
    return None
