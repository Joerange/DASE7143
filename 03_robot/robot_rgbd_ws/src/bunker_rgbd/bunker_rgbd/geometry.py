"""Adapted from local robot_nav_ws geometry (Apache-2.0); standalone copy.
10th percentile of a supported surface cluster defines front clearance.
"""
import math
import numpy as np

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
    good = np.isfinite(z) & (z >= .25) & (z <= 8.)
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
               gap=float(np.percentile(b[:, 0], 10)-front), bearing=float(math.atan2(bc[1], bc[0])),
               quality=float(ratio*coverage*keep.mean()), spread=spread, points=int(keep.sum()))
    if map_tf is not None:
        out['map_point'] = (np.asarray(map_tf)[:3, :3] @ pc + np.asarray(map_tf)[:3, 3]).tolist()
    else:
        out['reason'] = ''  # RGB-D servo uses base frame; map is not required.
    return out

