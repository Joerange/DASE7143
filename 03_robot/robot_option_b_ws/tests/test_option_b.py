import json,math,unittest
from robot_mission.core import Grid,Goal
from robot_mission.exploration import ViewMemory,choose_view,line_visible
from robot_mission.option_b import OptionBMission
from robot_perception.view_protocol import parse_view,build_view_request

def grid():return Grid(40,40,.15,(-3.,-3.,0.),[0]*1600)

class ExplorationTests(unittest.TestCase):
    def test_known_map_generates_new_camera_view(self):
        m=ViewMemory();m.record((0.,0.,0.),1.,'not_found','r')
        c=choose_view(Grid(80,80,.15,(-6.,-6.,0.),[0]*6400),m,(0.,0.,0.),2.,max_distance=1.)
        self.assertIsNotNone(c);self.assertGreater(math.hypot(c[0].x,c[0].y),0)
        self.assertGreater(c[2]['visual_gain'],0);self.assertEqual(c[2]['unknown_gain'],0)
    def test_route_and_visibility_obey_wall(self):
        g=grid()
        for y in range(40):g.data[y*40+24]=100
        self.assertFalse(line_visible(g,(0.,0.),(1.,0.)))
        c=choose_view(g,ViewMemory(),(0.,0.,0.),1.,max_distance=1.)
        self.assertIsNotNone(c);self.assertLess(c[0].x,.6)
        self.assertTrue(g.path_free(c[1],0.,c[0].yaw))
    def test_memory_direction_exclusion_cooldown(self):
        m=ViewMemory();p=(1.,1.,0.);m.record(p,1.,'not_found','r')
        self.assertTrue(m.blocked(p,100));self.assertFalse(m.blocked(p,100,revisit=True))
        self.assertFalse(m.blocked((1.,1.,math.pi),2))
        m.reject((2.,2.,0.),2.,'blocked');self.assertTrue(m.blocked((2.,2.,0.),3.))
        self.assertFalse(m.blocked((2.,2.,0.),100.));json.dumps(m.snapshot())
    def test_failed_view_not_coverage(self):
        m=ViewMemory();m.record((0.,0.,0.),1.,'timeout','r')
        self.assertFalse(m.blocked((0.,0.,0.),2.));self.assertEqual(len(m.history),1)

class OptionBFlowTests(unittest.TestCase):
    def setUp(self):
        self.t=OptionBMission(timeout=300);self.t.start(0.)
        self.t.world=grid();self.t.pose=(0.,0.,0.);self.tick(1.)
    def tick(self,now,stopped=True,idle=True,ready=True):
        self.t.tick(now,now,ready,stopped,{}, {},True,idle)
    def capture(self):
        self.tick(1.1);self.tick(1.9);self.assertEqual(self.t.state,'VIEW_OBSERVE')
        return self.t.view_request
    def reply(self,req,status):return dict(req,observation_stamp=req['after_stamp']+.1,status=status)
    def test_post_stop_capture_and_stale_response(self):
        req=self.capture();old=self.reply(req,'found');old['observation_stamp']=req['after_stamp']
        self.assertFalse(self.t.accept_view(old))
        wrong=self.reply(req,'found');wrong['request_id']='old';self.assertFalse(self.t.accept_view(wrong))
        self.assertTrue(self.t.accept_view(self.reply(req,'not_found')));self.tick(2.)
        self.assertEqual((self.t.state,self.t.source),('VIEW_TURN','search'))
        self.assertEqual(len(self.t.memory.views),1)
    def test_found_to_identity_to_docking(self):
        req=self.capture();self.t.accept_view(self.reply(req,'found'));self.tick(2.)
        self.assertEqual(self.t.state,'SEARCH_CONTINUOUS')
        event=dict(task_generation=self.t.generation,request_id='candidate',observation_id='frame',candidates=[dict(id=1,track_id='bottle')])
        self.assertTrue(self.t.candidate_event(event,2.1))
        self.assertTrue(self.t.confirmation(dict(event,valid=True,candidates=[dict(id=1,track_id='bottle',status='match')]),2.2))
        target=dict(valid=True,stamp=2.3,map_point=[1.,0.,.4],gap=.60,bearing=0.)
        goal=Goal('bottle',.04,0.,0.,(1.,0.,.4),2.3)
        self.t.tick(2.3,2.3,True,True,{'bottle':target},{'bottle':(goal,1.)})
        self.assertEqual(self.t.state,'READY_TO_NAV');self.t.navigation_accepted()
        for now in (2.4,3.2):
            target['stamp']=now;self.t.tick(now,now,True,True,{'bottle':target},{},navigation_idle=True)
        self.assertEqual(self.t.state,'DONE')
    def test_absent_target_timeout(self):
        for state in ('VIEW_OBSERVE','EXPLORE','SELECT_VIEW'):
            self.t.active=True;self.t.transition(state);self.tick(301.)
            self.assertEqual((self.t.state,self.t.source,self.t.active),('HOLD','none',False))
            self.assertIn('search_timeout',self.t.reason)
    def test_views_lead_to_translation_then_observation(self):
        for i in range(12):self.t.next_direction(3.+i)
        self.assertEqual(self.t.state,'SELECT_VIEW')
        self.t.selected=choose_view(grid(),self.t.memory,self.t.pose,20.,max_distance=1.);self.tick(20.)
        self.assertEqual(self.t.state,'EXPLORE_READY');self.t.navigation_accepted()
        self.assertEqual((self.t.state,self.t.source),('EXPLORE','nav'))
        self.t.navigation_result(True);self.assertEqual((self.t.state,self.t.source),('VIEW_SETTLE','none'))
    def test_visited_yaw_is_skipped_by_control(self):
        self.t.memory.record((0.,0.,math.pi/6),1.,'not_found','seen')
        self.t.next_direction(2.)
        self.assertAlmostEqual(self.t.turn_yaw,math.pi/3)
        self.assertEqual(self.t.state,'VIEW_TURN')
    def test_sensor_loss_stops(self):
        self.t.transition('EXPLORE');self.tick(2.,ready=False)
        self.assertEqual((self.t.state,self.t.source),('HOLD','none'))
    def test_cancel_finishes_before_capture(self):
        self.tick(2.,idle=False);self.tick(3.,idle=False);self.assertIsNone(self.t.view_request)
    def test_reset_retires_memory_and_response(self):
        req=self.capture();old=self.t.generation;self.t.memory.record((0.,0.,0.),1.,'not_found','x')
        self.t.recover('reset',3.,3.,reconfirm=True)
        self.assertNotEqual(old,self.t.generation);self.assertFalse(self.t.memory.views)
        self.assertFalse(self.t.accept_view(self.reply(req,'found')))

class ViewProtocolTests(unittest.TestCase):
    def test_parse_and_prompt(self):
        self.assertEqual(parse_view('{"status":"not_found"}'),'not_found')
        with self.assertRaises(ValueError):parse_view('{"status":"found","speed":1}')
        self.assertIn('umbrella',json.dumps(build_view_request('abc','umbrella','model')))
if __name__=='__main__':unittest.main()

class NavigationAdapterTests(unittest.TestCase):
    def test_exploration_dispatch_acceptance_and_late_goal_cancel(self):
        import ast,time
        from pathlib import Path
        from types import SimpleNamespace as NS,MethodType
        from unittest.mock import Mock
        from robot_mission.mapping import CachedPlan,plan_valid
        from robot_mission.core import path_length
        source=Path(__file__).resolve().parents[1]/'src/robot_mission/robot_mission/node.py'
        cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='MissionNode')
        funcs=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('start_exploration','nav_accepted')]
        env=dict(time=time,CachedPlan=CachedPlan,path_length=path_length,plan_valid=plan_valid,
                 NavigateToPose=NS(Goal=lambda:NS()))
        exec(compile(ast.Module(body=funcs,type_ignores=[]),str(source),'exec'),env)
        task=OptionBMission();task.start(0.);task.pose=(0.,0.,0.);task.last_now=1.
        task.nav_goal=Goal('__explore__',.5,0.,0.,(.5,0.,0.),1.)
        task.exploration_plan=((0.,0.),(.5,0.));task.transition('EXPLORE_READY')
        future=Mock();handle=NS(accepted=True,get_result_async=lambda:Mock())
        future.result.return_value=handle
        n=NS(task=task,nav_pending=False,nav_handle=None,cancel_pending=False,
            runtime_authorized=lambda *a:True,now=lambda:1.,grid=grid(),pose=(0.,0.,0.),
            map_version=1,nav_token=0,nav=NS(send_goal_async=Mock(return_value=Mock())),
            pose_msg=lambda goal:goal,feedback_cb=Mock(),nav_result=Mock(),cancel_navigation=Mock())
        for name in ('start_exploration','nav_accepted'):setattr(n,name,MethodType(env[name],n))
        n.start_exploration();self.assertTrue(n.nav_pending)
        n.nav_accepted(future,n.nav_token,task.generation)
        self.assertEqual((task.state,task.source),('EXPLORE','nav'))
        task.hold('stop');n.nav_accepted(future,n.nav_token,task.generation)
        n.cancel_navigation.assert_called()
