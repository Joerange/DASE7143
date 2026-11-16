"""ROS-independent geometry, identity, event gating and untrusted-result validation."""
import json
import math
import uuid
from collections import deque
from dataclasses import dataclass, field
import numpy as np


def strict_result(text, expected_ids):
    """All and only the requested IDs, exactly once; no executable model fields."""
    def unique(pairs):
        d = {}
        for k, v in pairs:
            if k in d:
                raise ValueError('duplicate JSON key')
            d[k] = v
        return d
    if len(text) > 16384:
        raise ValueError('response too large')
    obj = json.loads(text, object_pairs_hook=unique)
    if not isinstance(obj, dict) or set(obj) != {'candidates'}:
        raise ValueError('invalid result object')
    rows = obj['candidates']
    if not isinstance(rows, list) or len(rows) != len(expected_ids):
        raise ValueError('candidate count mismatch')
    out = {}
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'id', 'status'}:
            raise ValueError('invalid result fields')
        k, status = row['id'], row['status']
        if type(k) is not int or k not in expected_ids or k in out:
            raise ValueError('unknown or duplicate candidate ID')
        if status not in ('match', 'no_match', 'uncertain'):
            raise ValueError('invalid semantic status')
        out[k] = status
    if set(out) != set(expected_ids):
        raise ValueError('missing candidate ID')
    return out


def transform_matrix(translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    n = np.linalg.norm(q)
    if not np.isfinite(q).all() or n < 1e-8:
        raise ValueError('invalid transform quaternion')
    x, y, z, w = q / n
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = translation
    if not np.isfinite(t).all():
        raise ValueError('nonfinite transform')
    return t


def depth_surface(depth, encoding, bbox, k, distortion, distortion_model, base_tf,
                  map_tf=None, depth_scale=.001, front=.345, min_ratio=.30):
    """Conservative interior multi-region surface cluster, not nearest pixel.

    base_tf/map_tf map optical coordinates at acquisition time. Reject mixed
    surfaces, floor, isolated support, and excessive spread rather than inventing
    a target range from a nearby obstacle. Inputs remain unmodified.
    """
    invalid = lambda reason: {'valid': False, 'reason': reason}
    if encoding not in ('16UC1', '32FC1') or depth.ndim != 2:
        return invalid('unsupported_depth_encoding')
    k = np.asarray(k, float).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        return invalid('invalid_intrinsics')
    if not np.isfinite(bbox).all() or base_tf is None:
        return invalid('missing_base_tf_or_bbox')
    if encoding == '16UC1' and not 0 < depth_scale <= .01:
        return invalid('invalid_depth_scale')
    h, w = depth.shape
    x1, y1, x2, y2 = bbox
    dx, dy = .15*(x2-x1), .15*(y2-y1)
    x1, y1 = max(0, int(x1+dx)), max(0, int(y1+dy))
    x2, y2 = min(w, int(x2-dx)), min(h, int(y2-dy))
    if x2-x1 < 5 or y2-y1 < 5:
        return invalid('small_depth_roi')
    step = max(1, int(math.sqrt((x2-x1)*(y2-y1)/6000)))
    yy, xx = np.mgrid[y1:y2:step, x1:x2:step]
    z = depth[yy, xx].astype(float) * (depth_scale if encoding == '16UC1' else 1.)
    good = np.isfinite(z) & (z >= .15) & (z <= 8.)
    ratio = float(good.mean())
    if good.sum() < 40 or ratio < min_ratio:
        return invalid('insufficient_depth')
    uv = np.column_stack((xx[good], yy[good])).astype(float)
    z = z[good]
    if distortion is not None and np.any(np.abs(distortion) > 1e-10):
        import cv2
        if distortion_model in ('plumb_bob', 'rational_polynomial'):
            rays = cv2.undistortPoints(uv.reshape(-1, 1, 2), k, np.asarray(distortion)).reshape(-1, 2)
        elif distortion_model == 'equidistant':
            rays = cv2.fisheye.undistortPoints(uv.reshape(-1, 1, 2), k, np.asarray(distortion)).reshape(-1, 2)
        else:
            return invalid('unsupported_distortion_model')
    else:
        rays = (uv - k[:2, 2]) / np.array([k[0, 0], k[1, 1]])
    optical = np.column_stack((rays*z[:, None], z))
    base = optical @ np.asarray(base_tf)[:3, :3].T + np.asarray(base_tf)[:3, 3]
    nonground = (base[:, 2] > .08) & (base[:, 2] < 2.5) & (base[:, 0] > .1)
    optical, base, uv, z = optical[nonground], base[nonground], uv[nonground], z[nonground]
    if len(z) < 40 or len(z)/good.size < min_ratio:
        return invalid('floor_or_insufficient_object_support')
    # Depth discontinuities split surfaces; modest quantization prevents a
    # narrow bridge of edge pixels from joining a foreground and background.
    bins = np.floor(z/.06).astype(int)
    counts = np.bincount(bins)
    dense = counts >= max(4, len(z)*.015)
    groups, current = [], []
    for i, yes in enumerate(dense):
        if yes:
            current.append(i)
        elif current:
            groups.append(current); current = []
    if current:
        groups.append(current)
    masks = [np.isin(bins, group) for group in groups]
    masks.sort(key=lambda m: int(m.sum()), reverse=True)
    if not masks or masks[0].sum() < max(40, .55*len(z)):
        return invalid('ambiguous_depth_clusters')
    if len(masks) > 1 and masks[1].sum() > .15*len(z):
        return invalid('foreground_or_background_ambiguity')
    keep = masks[0]
    p, b, u = optical[keep], base[keep], uv[keep]
    spread = float(np.percentile(p[:, 2], 90)-np.percentile(p[:, 2], 10))
    # Occupied subregions ensure support extends across the visible object.
    cells = ((u[:, 0]-x1)/(x2-x1)*3).astype(int) + 3*((u[:, 1]-y1)/(y2-y1)*3).astype(int)
    coverage = len(np.unique(cells)) / 9.
    if spread > .20 or coverage < 5/9:
        return invalid('poor_surface_support')
    bc = np.median(b, axis=0)
    pc = np.median(p, axis=0)
    out = dict(valid=True, reason='', base_point=bc.tolist(), optical_point=pc.tolist(),
               gap=float(bc[0]-front), bearing=float(math.atan2(bc[1], bc[0])),
               quality=float(ratio*coverage*keep.mean()), spread=spread, points=int(keep.sum()))
    if map_tf is not None:
        out['map_point'] = (np.asarray(map_tf)[:3, :3] @ pc + np.asarray(map_tf)[:3, 3]).tolist()
        # Fit the visible surface, not an assumed unseen side of the bin.
        centered = b-b.mean(axis=0)
        covariance = np.einsum('ni,nj->ij', centered, centered)/len(centered)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, 0]
        if (eigenvalues[1] > .0004 and eigenvalues[0] < .15*eigenvalues[1]
                and abs(normal[2]) < .35):
            if np.dot(normal, bc) > 0:
                normal = -normal  # Outward normal faces the observing robot.
            normal_map = np.asarray(map_tf)[:3, :3] @ (np.asarray(base_tf)[:3, :3].T @ normal)
            horizontal = np.linalg.norm(normal_map[:2])
            if horizontal > .8:
                out['surface_normal_map'] = [float(normal_map[0]/horizontal), float(normal_map[1]/horizontal), 0.]
                out['surface_normal_base'] = normal.tolist()
    else:
        out['valid'] = False; out['reason'] = 'missing_map_tf'
    return out


def iou(a, b):
    ix = max(0, min(a[2], b[2])-max(a[0], b[0]))
    iy = max(0, min(a[3], b[3])-max(a[1], b[1]))
    den = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-ix*iy
    return ix*iy/den if den > 0 else 0.


@dataclass
class Track:
    id: str
    bbox: list
    last: float
    history: deque = field(default_factory=lambda: deque(maxlen=3))
    point: object = None
    state: str = 'NEW'
    until: float = 0.
    attempts: int = 0
    quality: float = 0.
    request_id: str = ''


class CandidateTracker:
    def __init__(self, generation='', max_lost=.7, cooldown=15., geometry_epoch=0):
        self.generation, self.max_lost, self.cooldown = generation, max_lost, cooldown
        self.geometry_epoch = geometry_epoch
        self.tracks, self.rejected = {}, []
        self.counter, self.collect_since = 0, None
        self.pending = None

    def reset(self, generation, geometry_epoch=0):
        self.__init__(generation, self.max_lost, self.cooldown, geometry_epoch)

    def update(self, detections, now):
        self.rejected = [(p, t) for p, t in self.rejected if t > now]
        for key in list(self.tracks):
            t = self.tracks[key]
            retain_static = t.state == 'CONFIRMED' and t.point is not None and now < t.until
            if now-t.last > self.max_lost and not retain_static:
                del self.tracks[key]
            elif now-t.last > self.max_lost:
                t.history.clear()
        if self.pending and now-self.pending['sent'] > 8.:
            self.apply(self.pending['request_id'], {}, now, valid=False)
        existing = list(self.tracks.values())
        options = []
        for d in detections:
            p = d.get('map_point')
            matches = []
            for t in existing:
                overlap = iou(d['bbox'], t.bbox)
                dist = np.linalg.norm(np.array(p)[:2]-np.array(t.point)[:2]) if p is not None and t.point is not None else None
                if dist is not None:
                    if dist < .30:
                        matches.append((t, 1-dist/.30 + .1*overlap))
                elif overlap > .25 and now-t.last < .4:
                    matches.append((t, overlap))
            matches.sort(key=lambda x: x[1], reverse=True)
            # Ambiguous identity invalidates the association, not a relabel.
            options.append(matches)
        assignments = {}
        for i, matches in enumerate(options):
            if len(matches) > 1 and matches[0][1]-matches[1][1] < .25:
                continue
            if matches:
                assignments.setdefault(matches[0][0].id, []).append(i)
        seen, out = set(), []
        for i, d in enumerate(detections):
            matches = options[i]
            if matches and (len(assignments.get(matches[0][0].id, [])) != 1 or
                            (len(matches) > 1 and matches[0][1]-matches[1][1] < .25)):
                continue
            if matches:
                t = matches[0][0]
            else:
                if d['score'] < .30:
                    continue
                self.counter += 1
                t = Track(f'{self.generation}:{self.geometry_epoch}:{self.counter}', list(d['bbox']), now)
                self.tracks[t.id] = t
            seen.add(t.id)
            t.bbox, t.last = list(d['bbox']), now
            if d.get('map_point') is not None:
                t.point = d['map_point']
            t.history.append(d['score'] >= .30)
            if t.state == 'REJECTED' and now >= t.until:
                t.state = 'NEW'
            if t.state == 'CONFIRMED' and now >= t.until:
                t.state = 'NEW'
            if t.point is not None and any(np.linalg.norm(np.array(t.point)[:2]-np.array(p)[:2]) < .25 for p, _ in self.rejected):
                t.state = 'REJECTED'
                t.until = max(exp for p, exp in self.rejected if np.linalg.norm(np.array(t.point)[:2]-np.array(p)[:2]) < .25)
            quality = d.get('image_quality', 0.)
            improved = quality > t.quality*1.3 + 5
            eligible = (t.state == 'NEW' or (t.state == 'RETRY' and now >= t.until) or
                        (t.state == 'UNCERTAIN' and now >= t.until and improved and t.attempts < 2))
            row = dict(d, track_id=t.id, stable=sum(t.history) >= 2,
                       eligible=eligible and sum(t.history) >= 2)
            out.append(row)
        for t in self.tracks.values():
            if t.id not in seen:
                t.history.append(False)
        return out

    def event(self, rows, now, window=.2, max_candidates=6):
        eligible = [r for r in rows if r['eligible'] and r.get('image_usable', True)]
        if self.pending or not eligible:
            self.collect_since = None
            return None
        if self.collect_since is None:
            self.collect_since = now
            return None
        if now-self.collect_since < window:
            return None
        rows = sorted(eligible, key=lambda d: d['score'], reverse=True)[:max_candidates]
        rid = uuid.uuid4().hex
        for i, r in enumerate(rows, 1):
            r['id'] = i
            t = self.tracks[r['track_id']]
            t.state, t.request_id = 'PENDING', rid
            t.attempts += 1
            t.quality = r.get('image_quality', 0.)
        self.pending = dict(request_id=rid, sent=now, tracks={r['id']: r['track_id'] for r in rows})
        self.collect_since = None
        return rid, rows

    def apply(self, request_id, statuses, now, valid=True, error='', retry_after=0.):
        if not self.pending or request_id != self.pending['request_id']:
            return False
        for cid, tid in self.pending['tracks'].items():
            t = self.tracks.get(tid)
            if not t or t.request_id != request_id:
                continue
            if not valid and error == 'worker_busy':
                # No API request was sent for this candidate. A previous geometry
                # context may still occupy the sole HTTP worker after a reset.
                t.state, t.until = 'RETRY', now + min(8.5, max(1., retry_after))
                t.attempts = max(0, t.attempts-1)
                continue
            status = statuses.get(cid, 'uncertain') if valid else 'uncertain'
            t.state = {'match': 'CONFIRMED', 'no_match': 'REJECTED'}.get(status, 'UNCERTAIN')
            t.until = now + (120. if status == 'match' else self.cooldown if status == 'no_match' else 1.)
            if status == 'no_match' and t.point is not None:
                self.rejected.append((t.point, t.until))
        self.pending = None
        return True
