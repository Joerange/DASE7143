"""Option B mission: settled fresh views, bounded navigation and explicit memory."""
import math
import uuid
from .core import Mission, wrap
from .exploration import ViewMemory, choose_view

class OptionBMission(Mission):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.memory=ViewMemory()
        self.world=None;self.pose=None;self.view_request=None;self.view_result=None
        self.exploration_plan=None;self.selection={};self.settled=None
        self.turn_yaw=0.;self.sector_count=0;self.explore_started=0.
        self.last_selection=-math.inf;self.selected=None;self.last_now=0.
        self.view_started=0.; self.view_attempts=0

    def transition(self,state,reason=''):
        super().transition(state,reason)
        source='nav' if state=='EXPLORE' else 'search' if state=='VIEW_TURN' else self.source
        if source!=self.source:self.epoch+=1;self.source=source

    def start(self,now):
        super().start(now)
        self.memory=ViewMemory();self.view_request=None;self.view_result=None
        self.exploration_plan=None;self.settled=None;self.sector_count=0;self.view_attempts=0
        self.selected=None;self.selection={};self.last_selection=-math.inf

    def observe(self,now):
        self.view_request=None;self.view_result=None;self.settled=None
        self.view_started=now
        self.transition('VIEW_SETTLE','settle before a fresh camera observation')

    def next_direction(self,now):
        self.view_request=None;self.view_result=None;self.view_attempts=0
        while self.sector_count < self.memory.sectors:
            self.sector_count+=1
            if self.sector_count>=self.memory.sectors:
                break
            self.turn_yaw=wrap(self.turn_yaw+2*math.pi/self.memory.sectors)
            if self.pose and self.memory.blocked((*self.pose[:2], self.turn_yaw), now,
                                                 revisit=self.selection.get('revisit',False)):
                continue
            self.explore_started=now
            self.transition('VIEW_TURN','turn to next unchecked yaw sector')
            return
        self.transition('SELECT_VIEW','local yaw sectors checked; select a new position')

    def accept_view(self,result):
        req=self.view_request
        if not req or self.state!='VIEW_OBSERVE':return False
        if any(result.get(k)!=req.get(k) for k in ('request_id','task_generation','geometry_epoch')):return False
        if result.get('observation_stamp',-math.inf)<=req['after_stamp']:return False
        self.view_result=result
        return True

    def recover(self,reason,now,ros_now,reconfirm=False):
        # Exploration has no target to reacquire; retry a new observation after cancel.
        if self.state in ('EXPLORE','EXPLORE_READY','SELECT_VIEW','VIEW_TURN','VIEW_SETTLE','VIEW_OBSERVE'):
            if reconfirm:
                self.memory=ViewMemory();self.geometry_epoch+=1;self.generation=str(uuid.uuid4())
            self.exploration_plan=None;self.nav_goal=None
            self.observe(now)
            return
        super().recover(reason,now,ros_now,reconfirm)

    def tick(self,now,ros_now,ready,stopped,targets,plans,detection_fresh=True,navigation_idle=True,locked_geometry=None):
        self.last_now=now
        if not self.active:return
        if now>=self.deadline:
            self.hold('search_timeout: selected target not found before deadline');return
        if self.state=='INIT':
            super().tick(now,ros_now,ready,stopped,targets,plans,detection_fresh,navigation_idle,locked_geometry)
            if self.state=='SEARCH_CONTINUOUS' and self.pose:
                self.turn_yaw=self.pose[2];self.observe(now)
            return
        if self.state.startswith('VIEW_') or self.state in ('SELECT_VIEW','EXPLORE_READY','EXPLORE'):
            if not ready or not detection_fresh:
                self.hold('required sensor/localization data unavailable');return
            if self.state=='VIEW_TURN':
                if abs(wrap(self.turn_yaw-self.pose[2]))<.035:self.observe(now)
                elif now-self.explore_started>45:self.hold('view turn timeout')
            elif self.state=='VIEW_SETTLE':
                if stopped and navigation_idle:
                    self.settled=now if self.settled is None else self.settled
                    if now-self.settled>=.7:
                        self.view_request=dict(schema=1,stamp=ros_now,after_stamp=ros_now,
                            request_id=uuid.uuid4().hex,task_generation=self.generation,geometry_epoch=self.geometry_epoch,
                            pose=list(self.pose))
                        self.view_started=now;self.transition('VIEW_OBSERVE','new stationary image and VLM check')
                else:self.settled=None
            elif self.state=='VIEW_OBSERVE':
                result=self.view_result
                if result is not None or now-self.view_started>12:
                    outcome=result.get('status','error') if result else 'timeout'
                    self.memory.record(self.view_request['pose'],now,outcome,self.view_request['request_id'])
                    if outcome=='found':
                        self.coverage=0.;self.last_yaw=None
                        self.transition('SEARCH_CONTINUOUS','VLM found target; acquire a local candidate and current depth')
                        self.acquire_until=now+15
                    elif outcome=='not_found':self.next_direction(now)
                    elif self.view_attempts<1:
                        self.view_attempts+=1;self.observe(now)
                    else:self.next_direction(now)
            elif self.state=='SELECT_VIEW' and stopped and navigation_idle and now-self.last_selection>=2:
                self.last_selection=now
                selected=self.selected
                self.selected=None
                if selected:
                    self.nav_goal,path,self.selection=selected
                    self.exploration_plan=path;self.ready_since=now
                    self.transition('EXPLORE_READY','highest-gain reachable unobserved viewpoint')
            elif self.state=='EXPLORE_READY' and now-self.ready_since>self.timing.goal_timeout:
                self.memory.reject((self.nav_goal.x,self.nav_goal.y,self.nav_goal.yaw),now,'dispatch_timeout')
                self.transition('SELECT_VIEW','try another viewpoint')
            elif self.state=='EXPLORE' and now-self.explore_started>45:
                self.memory.reject((self.nav_goal.x,self.nav_goal.y,self.nav_goal.yaw),now,'navigation_timeout')
                self.transition('SELECT_VIEW','exploration navigation deadline')
            return
        if self.state=='SEARCH_CONTINUOUS' and now>=getattr(self,'acquire_until',math.inf):
            self.memory.record(self.pose,now,'localization_unresolved','')
            self.next_direction(now);return
        # Reuse the tested identity, docking and stopping flow.
        super().tick(now,ros_now,ready,stopped,targets,plans,detection_fresh,navigation_idle,locked_geometry)
        if self.state=='HOLD' and self.reason=='full observed search completed without an accepted reachable target':
            self.active=True;self.next_direction(now)

    def navigation_accepted(self):
        if self.state=='EXPLORE_READY':
            self.explore_started=self.last_now;self.transition('EXPLORE','Nav2 exploring new viewpoint')
        else:super().navigation_accepted()

    def navigation_result(self,succeeded):
        if self.state=='EXPLORE':
            now=self.last_now
            if succeeded:
                self.sector_count=0;self.turn_yaw=self.nav_goal.yaw;self.observe(now)
            else:
                self.memory.reject((self.nav_goal.x,self.nav_goal.y,self.nav_goal.yaw),now,'navigation_failed')
                self.transition('SELECT_VIEW','try next reachable viewpoint')
        else:super().navigation_result(succeeded)
