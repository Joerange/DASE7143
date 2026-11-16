"""UI-independent timestamps, evidence and auditable operation outcomes."""
from collections import deque
import json
import math
import time
import uuid


def fresh(value, now, max_age=.3):
    return bool(value and math.isfinite(value.get('_received', -math.inf)) and 0<=now-value['_received']<=max_age
                and 0<=value.get('_age',0.) and value.get('_age',0.)+now-value['_received']<=max_age)


class Feed:
    def __init__(self):
        self.times=deque(maxlen=50);self.stamp=None;self.publishers=None
    def update(self,stamp,now):
        self.times.append(now);self.stamp=stamp
    def status(self,now,ros_now,limit):
        if not self.times:
            return {'state':'waiting','age':None,'hz':0.,'stamp':None}
        stamp_age=ros_now-self.stamp
        age=max(now-self.times[-1],stamp_age)
        hz=(len(self.times)-1)/(self.times[-1]-self.times[0]) if len(self.times)>1 and self.times[-1]>self.times[0] else 0.
        status='disconnected' if self.publishers==0 else 'stale' if age>limit or stamp_age<-.05 else 'normal'
        return {'state':status,'age':max(0.,age),'hz':hz,'stamp':self.stamp,'received_age':now-self.times[-1]}


# Whitelist excludes raw image arrays, credentials, request bodies and model prompts.
EVENT_FIELDS={'stamp','time','kind','source','message','reason','task_generation','geometry_epoch',
              'observation_id','request_id','latency','elapsed','error','state','operation_id','action',
              'result','valid','candidates','track_id','target_track_id','observed_target_track_id','target_revision','refinement_status',
              'map_revision','validated_paths','waiting_for','planning_status','goal','path_length','status'}
CANDIDATE_FIELDS={'id','track_id','label','score','status','bbox'}

def event_record(value):
    out={k:v for k,v in value.items() if k in EVENT_FIELDS}
    if 'candidates' in out:
        out['candidates']=[{k:v for k,v in row.items() if k in CANDIDATE_FIELDS} for row in out['candidates'][:6]]
    # Reject non-JSON/nonfinite content instead of accidentally exporting objects.
    return json.loads(json.dumps(out,allow_nan=False))


class Journal:
    def __init__(self,limit=4000):self.events=deque(maxlen=limit)
    def add(self,kind,message,**fields):
        event=event_record(dict(stamp=time.time(),kind=kind,message=message,**fields))
        self.events.append(event);return event
    def export(self,path,generation=None):
        selected=[e for e in self.events if not generation or e.get('task_generation') in (generation,None,'')]
        path.write_text(json.dumps({'schema':1,'task_generation':generation,'events':selected,
            'excluded':['credentials','images','request_payloads']},ensure_ascii=False,indent=2))
        return str(path)


class StopEvidence:
    """Three independent receipts + real CAN feedback, stable after requested stop."""
    def __init__(self):self.since=None
    def check(self,safety,mission,odom,can_feedback,now,require_can=False):
        reasons=[]
        if not fresh(safety,now,.8) or safety.get('authorized',True) or not safety.get('output_zero',False):reasons.append('guard zero/disarm not confirmed')
        if not fresh(mission,now,.8) or mission.get('active',True) or mission.get('source')!='none' or not mission.get('navigation_idle',False):reasons.append('mission/nav cancellation not confirmed')
        v,w=(odom or {}).get('linear_x',math.inf),(odom or {}).get('angular_z',math.inf)
        if not fresh(odom,now) or not math.isfinite(v) or not math.isfinite(w) or abs(v)>=.015 or abs(w)>=.025:reasons.append('fresh odometry standstill missing')
        if require_can and (not fresh(can_feedback,now) or not can_feedback.get('stopped',False)):reasons.append('live CAN feedback standstill missing')
        if reasons:
            self.since=None
            return False,'; '.join(reasons)
        self.since=now if self.since is None else self.since
        return now-self.since>=.7, 'stable standstill confirmed' if now-self.since>=.7 else 'verifying 0.7s standstill'


class Operation:
    def __init__(self,action,now):
        self.id=uuid.uuid4().hex;self.action=action;self.started=now
        self.phase='requested';self.result='pending';self.message='request queued';self.futures={};self.evidence=StopEvidence()
        self.deadline=now+20.;self.token=None;self.proof_at=None
    def view(self):return dict(id=self.id,operation_id=self.id,action=self.action,phase=self.phase,result=self.result,message=self.message)
    def finish(self,result,message):self.result=result;self.phase='finished';self.message=message
