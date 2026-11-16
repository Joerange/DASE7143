import math
import numpy as np
from .geometry import transform_matrix


def seconds(stamp):return stamp.sec+stamp.nanosec/1e9


def matrix(tf):
    t,q=tf.transform.translation,tf.transform.rotation
    return transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])


def depth_array(message):
    if message.encoding not in ('16UC1','32FC1'):raise ValueError('unsupported depth encoding')
    dtype=np.dtype(('>' if message.is_bigendian else '<')+('u2' if message.encoding=='16UC1' else 'f4'))
    return np.ndarray((message.height,message.width),dtype,buffer=message.data,
                      strides=(message.step,dtype.itemsize)).copy()


def corridor(depth,encoding,k,transform,distortion=(),distortion_model='plumb_bob'):
    """Observed front obstacles only; never claims unseen side/rear free space."""
    k=np.array(k).reshape(3,3)
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:raise ValueError('invalid intrinsics')
    z=depth[::6,::6].astype(float)*(.001 if encoding=='16UC1' else 1.)
    good=np.isfinite(z)&(z>=.32)&(z<=6.)
    h,w=good.shape
    band=good[h//4:3*h//4,max(0,w//10):max(1,9*w//10)]
    # Missing sectors remain unknown, including zero/NaN holes and near Min-Z invalid data.
    valid=band.size>0 and all(a.size and a.mean()>=.35 for a in np.array_split(band,7,axis=1))
    yy,xx=np.mgrid[:h,:w];xx=xx*6;yy=yy*6
    uv=np.column_stack((xx[good],yy[good])).astype(float)
    if len(distortion) and np.any(np.abs(distortion)>1e-8):
        import cv2
        if distortion_model in ('plumb_bob','rational_polynomial'):
            rays=cv2.undistortPoints(uv.reshape(-1,1,2),k,np.array(distortion)).reshape(-1,2)
        elif distortion_model=='equidistant':
            rays=cv2.fisheye.undistortPoints(uv.reshape(-1,1,2),k,np.array(distortion)).reshape(-1,2)
        else:raise ValueError('unsupported distortion model')
    else:rays=(uv-k[:2,2])/np.array([k[0,0],k[1,1]])
    optical=np.column_stack((rays*z[good,None],z[good]))
    points=optical@transform[:3,:3].T+transform[:3,3]
    points=points[(points[:,2]>.05)&(points[:,2]<1.5)]
    return dict(valid=bool(valid),points=points[:,:2].tolist(),coverage=float(band.mean()) if band.size else 0.)
