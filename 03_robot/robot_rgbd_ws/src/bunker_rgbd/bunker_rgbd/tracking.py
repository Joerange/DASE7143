"""Short-lived local identity; no semantic service, map or cached world target."""
import uuid
from .timing import TRACK_TIMEOUT


def iou(a,b):
    x=max(0.,min(a[2],b[2])-max(a[0],b[0]));y=max(0.,min(a[3],b[3])-max(a[1],b[1]))
    return x*y/max(1e-9,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-x*y)


class Tracker:
    def __init__(self):self.tracks={};self.last_stamp=-1.
    def update(self,rows,stamp):
        if stamp<=self.last_stamp:return []
        self.last_stamp=stamp;used=set();out=[]
        self.tracks={key:t for key,t in self.tracks.items() if stamp-t['stamp']<=TRACK_TIMEOUT}
        for row in rows:
            matches=[(iou(row['bbox'],t['bbox']),key) for key,t in self.tracks.items() if key not in used]
            overlap,key=max(matches,default=(0.,''))
            if overlap<.35:key=uuid.uuid4().hex
            old=self.tracks.get(key,{})
            count=(old.get('count',0)+1 if row.get('valid') and old.get('valid') and stamp-old.get('stamp',stamp)<.22
                   else 1 if row.get('valid') else 0)
            t=dict(row,track_id=key,stamp=stamp,count=count,stable=bool(row.get('valid') and count>=3))
            self.tracks[key]=t;used.add(key);out.append(t)
        return out
