"""Pure rendering rules: frame identity, freshness, and map coordinates."""
import math


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def matching_candidates(frame, payload, tolerance=0.0001):
    """Never draw a detector result onto an image from another acquisition."""
    if not isinstance(frame, dict) or not isinstance(payload, dict):
        return []
    a, b = frame.get('stamp'), payload.get('stamp')
    if not finite_number(a) or not finite_number(b) or abs(a-b) > tolerance:
        return []
    if frame.get('frame_id') and payload.get('frame_id') and frame['frame_id'] != payload['frame_id']:
        return []
    if frame.get('observation_id') and payload.get('observation_id') and frame['observation_id'] != payload['observation_id']:
        return []
    rows = payload.get('candidates', [])
    return [row for row in rows if isinstance(row, dict) and len(row.get('bbox', [])) == 4
            and all(finite_number(x) for x in row['bbox']) and row['bbox'][2] > row['bbox'][0]
            and row['bbox'][3] > row['bbox'][1]]


def velocity_text(chassis, max_age=.3):
    velocity = chassis.get('velocity', {}) if isinstance(chassis, dict) else {}
    age, v, w = velocity.get('age'), velocity.get('linear_x'), velocity.get('angular_z')
    if velocity.get('valid') is not True or not all(finite_number(x) for x in (age,v,w)) or not 0 <= age <= max_age:
        return None
    return v, w, age


def world_to_grid(x, y, metadata):
    width, height = int(metadata['width']), int(metadata['height'])
    resolution = float(metadata['resolution'])
    ox, oy, yaw = metadata['origin']
    if not all(finite_number(v) for v in (x, y, resolution, ox, oy, yaw)) or resolution <= 0 or min(width,height) <= 0:
        raise ValueError('invalid map metadata')
    dx, dy = x-ox, y-oy
    gx = (math.cos(yaw)*dx + math.sin(yaw)*dy)/resolution
    gy = (-math.sin(yaw)*dx + math.cos(yaw)*dy)/resolution
    # OccupancyGrid row zero is the map bottom, QImage row zero is its top.
    return gx, height-gy


def body_polygon(pose, front=.345, half_width=.285):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*px-s*py,y+s*px+c*py) for px,py in
            ((front,half_width),(front,-half_width),(-front,-half_width),(-front,half_width))]


def normalize_pose(value):
    if isinstance(value, dict):
        value = [value.get('x'), value.get('y'), value.get('yaw',0.)]
    if isinstance(value, (list,tuple)) and len(value) >= 2 and all(finite_number(x) for x in value[:2]):
        yaw = value[2] if len(value)>2 else 0.
        return (float(value[0]),float(value[1]),float(yaw)) if finite_number(yaw) else None
    return None


def control_enabled(controls, action):
    value = controls.get(action, False) if isinstance(controls,dict) else False
    if isinstance(value,dict):
        return value.get('enabled') is True, str(value.get('reason',''))
    return value is True, ''


def sensor_text(value):
    if not isinstance(value,dict):
        return 'waiting', None, None, ''
    state = value.get('state', 'waiting')
    age, hz = value.get('age'), value.get('hz')
    return state, age if finite_number(age) and age>=0 else None, hz if finite_number(hz) and hz>=0 else None, str(value.get('reason',''))


def timeline_category(event):
    kind = str(event.get('kind',event.get('source',event.get('event','')))).lower()
    if any(key in kind for key in ('vlm','confirmation','api')):
        return 'VLM'
    if kind == 'event' or any(key in kind for key in ('detector','candidate','perception')):
        return 'DETECTION'
    if any(key in kind for key in ('mission','state','plan','local','decision','navigation','recovery','safety')):
        return 'LOCAL'
    return 'SYSTEM'


def feed_fresh(sensors, key, limit=.3):
    value=sensors.get(key,{}) if isinstance(sensors,dict) else {}
    if not isinstance(value,dict):return False
    return value.get('state') in ('normal','healthy','ok') and finite_number(value.get('age')) and 0<=value['age']<=limit


def authorization_value(snapshot):
    if not feed_fresh(snapshot.get('sensors',{}),'safety'):
        return None
    value=snapshot.get('safety',{}).get('authorized')
    return value if type(value) is bool else None


def current_mission_state(snapshot):
    if not feed_fresh(snapshot.get('sensors',{}),'mission'):
        return None
    value=snapshot.get('mission',{}).get('state')
    return value if isinstance(value,str) and value else None


class LatestSlot:
    """At most one outstanding Qt wakeup; overwrite pending snapshots in place."""
    def __init__(self):
        import threading
        self.lock=threading.Lock()
        self.value=None
        self.notified=False

    def offer(self,value):
        with self.lock:
            self.value=value
            notify=not self.notified
            self.notified=True
            return notify

    def take(self):
        with self.lock:
            value=self.value
            self.value=None
            self.notified=False
            return value
