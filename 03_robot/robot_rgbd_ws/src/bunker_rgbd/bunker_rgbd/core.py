"""Camera-only mission and final guard logic, independent of ROS transports."""
import math
import uuid
from .timing import DATA_TIMEOUT,OBSERVATION_TIMEOUT,COMMAND_TIMEOUT,MISSION_TIMEOUT,SEARCH_TIMEOUT


def finite(*values):
    return all(type(v) in (int,float) and math.isfinite(v) for v in values)


def fresh(value, now, ros_now, limit=DATA_TIMEOUT):
    return bool(value and finite(value.get('stamp'),value.get('_rx')) and
                0<=now-value['_rx']<=limit and -.03<=ros_now-value['stamp']<=limit)


def target_valid(t):
    return bool(t and t.get('valid') is True and isinstance(t.get('track_id'),str) and
                finite(t.get('gap'),t.get('bearing')) and t['gap']>0.)


class PID:
    """Bounded PID with conditional integration and filtered derivative."""
    def __init__(self,kp,ki,kd,low,high):
        self.kp,self.ki,self.kd=kp,ki,kd;self.low,self.high=low,high
        self.integral=0.;self.previous=None;self.derivative=0.
    def update(self,error,dt):
        dt=max(.01,min(.15,dt))
        derivative=0. if self.previous is None else (error-self.previous)/dt
        self.derivative=.7*self.derivative+.3*derivative;self.previous=error
        integral=max(-.5,min(.5,self.integral+error*dt))
        raw=self.kp*error+self.ki*integral+self.kd*self.derivative
        if self.low<raw<self.high or raw>=self.high and error<0 or raw<=self.low and error>0:
            self.integral=integral
        return max(self.low,min(self.high,raw))


def stop_boundary(actual_v):
    speed=max(0.,actual_v)
    return .31+speed*.20+speed**2/(2*.20)


def goal_error(goal,odom):
    if not goal or len(goal)!=2 or not finite(*goal,odom.get('x'),odom.get('y'),odom.get('yaw')):
        return None
    dx,dy=goal[0]-odom['x'],goal[1]-odom['y']
    angle=math.atan2(dy,dx)-odom['yaw']
    return math.hypot(dx,dy),math.atan2(math.sin(angle),math.cos(angle))


class Mission:
    def __init__(self):
        self.generation=uuid.uuid4().hex;self.state='IDLE';self.reason='manual start required'
        self.target='';self.goal=None;self.tracking_mode='search';self.completion_basis='';self.active=False;self.started=0.;self.stopped_since=None
        self.search_rotation=0.;self.last_tick=None
        self.distance_pid=PID(.45,.01,.02,0.,.08);self.angle_pid=PID(.70,.02,.04,-.12,.12)
    def start(self,now):
        self.__init__();self.generation=uuid.uuid4().hex;self.active=True
        self.started=now;self.last_tick=now;self.state='SEARCH';self.reason='rotating to find trash can'
    def stop(self,reason):
        self.active=False;self.state='HOLD';self.reason=reason;self.stopped_since=None
    def step(self,now,ros_now,observation,safety,odom):
        if not self.active:return (0.,0.)
        if now-self.started>MISSION_TIMEOUT:self.stop('mission timeout');return (0.,0.)
        if not fresh(safety,now,ros_now):
            self.stop('guard status stale');return (0.,0.)
        if not safety.get('authorized') or not safety.get('ready'):
            self.stop('guard: '+safety.get('reason','authorization/health lost'));return (0.,0.)
        if not fresh(odom,now,ros_now) or not finite(odom.get('v'),odom.get('w')):
            self.stop('actual velocity unavailable');return (0.,0.)
        current_observation=fresh(observation,now,ros_now,OBSERVATION_TIMEOUT) and observation.get('ready')
        if not current_observation and (self.state=='SEARCH' or self.goal is None):
            self.stop('YOLO/RGB-D observation expired');return (0.,0.)
        dt=min(.1,max(0.,now-(self.last_tick if self.last_tick is not None else now)))
        self.last_tick=now
        # YOLO is asynchronous. Once a goal is recorded, a delayed detection is
        # equivalent to a temporarily invisible target, not a lost depth sensor.
        rows=observation.get('targets',[]) if current_observation else []
        if self.state=='SEARCH':
            self.search_rotation+=abs(odom['w'])*dt
            if not self.target and rows:self.target=rows[0]['track_id']
            chosen=next((t for t in rows if t.get('track_id')==self.target and target_valid(t)),None)
            if chosen:
                if not finite(odom.get('x'),odom.get('y'),odom.get('yaw')):
                    self.stop('odometry pose unavailable');return (0.,0.)
                heading=odom['yaw']+chosen['bearing']
                travel=max(0.,(chosen['gap']+.345)/max(.2,math.cos(chosen['bearing']))-.345-.30)
                self.goal=[odom['x']+travel*math.cos(heading),odom['y']+travel*math.sin(heading)]
                self.target=chosen['track_id'];self.state='APPROACH';self.reason='first YOLO target locked; PID approach' 
                return (0.,0.)
            if self.search_rotation>=2*math.pi or now-self.started>SEARCH_TIMEOUT:
                self.stop('search complete without stable target');return (0.,0.)
            if self.target:
                self.reason='first target locked; waiting for its valid depth'
                return (0.,0.)
            return (0.,.12)
        target=next((t for t in rows if t.get('track_id')==self.target),None)
        visible=target_valid(target)
        remaining=goal_error(self.goal,odom)
        if remaining is None:self.stop('recorded odometry goal unavailable');return (0.,0.)
        self.tracking_mode='visual_depth' if visible else 'recorded_odometry_goal'
        gap,bearing=(target['gap'],target['bearing']) if visible else (remaining[0]+.30,remaining[1])
        if visible and gap<.27:self.stop('too close; no automatic reverse');return (0.,0.)
        if self.state=='VERIFY_STOP':
            if visible and (not .27<=gap<=.34 or abs(bearing)>.12):
                self.stop('final depth outside tolerance');return (0.,0.)
            if not visible:self.completion_basis='odometry_estimate'
            if abs(odom['v'])<.015 and abs(odom['w'])<.025:
                if self.stopped_since is None:self.stopped_since=now
                if now-self.stopped_since>=.7:
                    self.state='DONE';self.active=False
                    self.reason='30cm depth and standstill verified' if self.completion_basis=='depth' else 'recorded distance reached; 30cm clearance estimated from odometry'
            else:self.stopped_since=None
            return (0.,0.)
        reached=(gap<=stop_boundary(odom['v'])) if visible else (remaining[0]<=.02+max(0.,odom['v'])*.20)
        if reached:
            if not visible or gap<=.34 and abs(bearing)<=.12:
                self.state='VERIFY_STOP';self.completion_basis='depth' if visible else 'odometry_estimate'
                self.reason='arrival; verifying standstill ('+self.completion_basis+')'
            else:self.stop('braking stop outside tolerance')
            return (0.,0.)
        w=self.angle_pid.update(bearing,dt)
        # Turn first when off-axis; then use simultaneous slow forward/yaw control.
        v=0. if abs(bearing)>.12 else self.distance_pid.update(gap-.30,dt)
        if v>0.:v=max(.015,v)
        if gap<.65:v=min(v,.035)
        command=(v,w)
        return command


class Safety:
    def __init__(self,capable=False):
        self.capable=capable;self.authorized=False;self.reason='startup disarmed'
        self.last=(0.,0.);self.last_tick=None
    def revoke(self,reason):
        self.authorized=False;self.reason=reason;self.last=(0.,0.)
    def health(self,now,ros_now,inputs):
        for key in ('rgb','depth','odom','mission'):
            if not fresh(inputs.get(key),now,ros_now):return False,key+' stale'
        mission=inputs['mission']
        following_goal=mission.get('active') and mission.get('state') in ('APPROACH','VERIFY_STOP')
        if following_goal:
            if goal_error(mission.get('goal'),inputs['odom']) is None:return False,'recorded goal/odom invalid'
        else:
            if not fresh(inputs.get('observation'),now,ros_now,OBSERVATION_TIMEOUT):return False,'observation stale'
            if not inputs['observation'].get('ready'):return False,'YOLO not ready'
        depth=inputs['depth']
        if not depth.get('valid'):return False,'depth coverage/TF invalid'
        odom=inputs['odom']
        if not finite(odom.get('v'),odom.get('w')):return False,'invalid odometry'
        fb=inputs.get('feedback',{})
        if not fb.get('live'):return False,'live chassis feedback missing'
        if fb.get('vehicle_state')!=0 or fb.get('error_code')!=0:return False,'chassis fault or physical estop'
        if not fb.get('velocity'):return False,'chassis velocity unavailable'
        if abs(fb['velocity'][0]-odom['v'])>.05 or abs(fb['velocity'][1]-odom['w'])>.10:
            return False,('chassis/odom disagreement: CAN=(%.3f, %.3f), odom=(%.3f, %.3f)'%
                          (fb['velocity'][0],fb['velocity'][1],odom['v'],odom['w']))
        return True,'ready'
    def grant(self,enabled,now,ros_now,inputs):
        if not enabled:self.revoke('operator disarmed');return True,self.reason
        ready,reason=self.health(now,ros_now,inputs)
        if not self.capable:reason='runtime control disabled';ready=False
        m=inputs.get('mission',{});o=inputs.get('odom',{})
        if m.get('active') or not finite(o.get('v'),o.get('w')) or abs(o.get('v',1))>=.015 or abs(o.get('w',1))>=.025:
            ready=False;reason='task and actual chassis must be stopped'
        if not ready:self.revoke(reason);return False,reason
        self.authorized=True;self.reason='operator authorized';return True,self.reason
    def tick(self,now,ros_now,inputs,command):
        ready,reason=self.health(now,ros_now,inputs)
        if not ready and self.authorized:self.revoke(reason)
        def zero(why):self.last=(0.,0.);return self.last,ready,why
        if not self.authorized:return zero(reason if not ready else self.reason)
        mission=inputs['mission']
        if not mission.get('active'):return zero('task inactive')
        state=mission.get('state')
        if state in ('VERIFY_STOP','DONE','HOLD','IDLE'):return zero('task stop')
        if state not in ('SEARCH','APPROACH'):
            self.revoke('invalid mission state');return zero(self.reason)
        if not fresh(command,now,ros_now,COMMAND_TIMEOUT) or command.get('generation')!=mission.get('generation') or command.get('state')!=state:
            return zero('command expired or wrong generation/state')
        v,w=command.get('v'),command.get('w')
        if not finite(v,w) or v<0 or v>.081 or abs(w)>.121:
            self.revoke('invalid velocity');return zero(self.reason)
        if state=='SEARCH' and v!=0:
            self.revoke('search must rotate in place');return zero(self.reason)
        odom=inputs['odom']
        if abs(odom['v'])>.13 or abs(odom['w'])>.22:
            self.revoke('measured overspeed');return zero(self.reason)
        if state=='APPROACH':
            observation=inputs.get('observation',{})
            rows=observation.get('targets',[]) if fresh(observation,now,ros_now,OBSERVATION_TIMEOUT) and observation.get('ready') else []
            t=next((t for t in rows if t.get('track_id')==mission.get('target')),None)
            if target_valid(t):
                if t['gap']<=stop_boundary(odom['v']):return zero('depth stop distance')
                v=min(v,.035 if t['gap']<.65 else .08)
                if abs(t['bearing'])>.12:v=0.
            else:
                remaining=goal_error(mission.get('goal'),odom)
                if remaining is None:self.revoke('recorded goal/odom invalid');return zero(self.reason)
                if remaining[0]<=.02+max(0.,odom['v'])*.20:return zero('odometry goal reached')
                v=min(v,.035 if remaining[0]<.35 else .08)
                if abs(remaining[1])>.12:v=0.
        # Conservative forward corridor: observation is local and does not certify unseen sides/rear.
        brake=max(v,max(0.,odom['v']))*.25+max(v,max(0.,odom['v']))**2/.40
        if any(-.345-.08<x<.345+.08+brake and abs(y)<.285+.08 for x,y in inputs['depth']['points']):
            self.revoke('obstacle in observed braking corridor');return zero(self.reason)
        dt=.02 if self.last_tick is None else min(.04,max(0.,now-self.last_tick))
        self.last_tick=now
        self.last=(min(v,self.last[0]+.10*dt),max(self.last[1]-.25*dt,min(self.last[1]+.25*dt,w)))
        return self.last,ready,'authorized'
