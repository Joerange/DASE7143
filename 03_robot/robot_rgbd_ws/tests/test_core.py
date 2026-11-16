"""Small regression suite for the requested search/PID/stop loop."""
import unittest
from bunker_rgbd.core import Mission,Safety,PID,stop_boundary,fresh
from bunker_rgbd.tracking import Tracker
from bunker_rgbd.detection import is_target_label
from bunker_rgbd.timing import SEARCH_TIMEOUT


def inputs(now=10.):
    t=dict(valid=True,track_id='bin',stable=True,gap=1.,bearing=0.)
    return {key:dict(value,stamp=now,_rx=now) for key,value in {
        'rgb':{},'depth':{'valid':True,'points':[]},'observation':{'ready':True,'targets':[t]},
        'odom':{'v':0.,'w':0.,'x':0.,'y':0.,'yaw':0.},'mission':{'generation':'g','active':False,'state':'IDLE','target':'bin','goal':[.7,0.]}
    }.items()} | {'feedback':dict(live=True,vehicle_state=0,error_code=0,velocity=[0.,0.])}


class LoopTests(unittest.TestCase):
    def setUp(self):
        self.m=Mission();self.m.start(10.);self.i=inputs();self.s=dict(stamp=10.,_rx=10.,authorized=True,ready=True)
    def step(self,t=10.):
        for value in self.i.values():value.update(stamp=t,_rx=t)
        self.s.update(stamp=t,_rx=t)
        return self.m.step(t,t,self.i['observation'],self.s,self.i['odom'])
    def test_search_then_direct_pid_approach(self):
        target=self.i['observation']['targets'];self.i['observation']['targets']=[]
        self.assertEqual(self.step(),(0.,.12))
        self.i['observation']['targets']=target;self.step(10.1)
        self.assertEqual(self.m.state,'APPROACH');self.assertGreater(self.step(10.15)[0],0.)
    def test_turn_before_forward(self):
        self.i['observation']['targets'][0]['bearing']=.4
        self.step();v,w=self.step(10.1);self.assertEqual(v,0);self.assertGreater(w,0)
    def test_stop_30cm_requires_measured_stillness(self):
        self.step();self.i['observation']['targets'][0]['gap']=.305
        self.assertEqual(self.step(10.1),(0.,0.));self.assertEqual(self.m.state,'VERIFY_STOP')
        self.i['odom']['v']=.02;self.step(10.2);self.assertNotEqual(self.m.state,'DONE')
        self.i['odom']['v']=0.;self.step(10.4);self.step(11.11)
        self.assertEqual(self.m.state,'DONE');self.assertFalse(self.m.active)
    def test_lost_target_continues_to_recorded_goal_without_switching(self):
        self.step();goal=list(self.m.goal);self.i['observation']['targets'][0]['track_id']='other'
        self.assertGreater(self.step(10.1)[0],0.);self.assertEqual(self.m.target,'bin')
        self.assertEqual(self.m.goal,goal);self.assertEqual(self.m.tracking_mode,'recorded_odometry_goal')
        self.i['odom']['x']=goal[0]-.01;self.step(10.2);self.step(10.3);self.step(11.1)
        self.assertEqual(self.m.state,'DONE');self.assertEqual(self.m.completion_basis,'odometry_estimate')
    def test_delayed_yolo_uses_goal_but_stale_depth_still_stops(self):
        self.step()
        self.i['observation'].update(stamp=9.,_rx=9.)
        self.i['odom'].update(stamp=10.1,_rx=10.1)
        self.s.update(stamp=10.1,_rx=10.1)
        v,w=self.m.step(10.1,10.1,self.i['observation'],self.s,self.i['odom'])
        self.assertGreater(v,0.);self.assertEqual(self.m.tracking_mode,'recorded_odometry_goal')
        i=inputs();g=Safety(True);self.assertTrue(g.grant(True,10.,10.,i)[0])
        i['mission'].update(active=True,state='APPROACH')
        i['observation'].update(stamp=9.,_rx=9.)
        c=dict(stamp=10.,_rx=10.,generation='g',state='APPROACH',v=.05,w=0.)
        self.assertGreater(g.tick(10.,10.,i,c)[0][0],0.)
        self.assertTrue(g.authorized)
        i['depth']['stamp']=9.
        self.assertEqual(g.tick(10.,10.,i,c)[0],(0.,0.));self.assertFalse(g.authorized)
        i=inputs();i['observation']['stamp']=9.
        self.assertFalse(Safety(True).grant(True,10.,10.,i)[0])
    def test_tripled_timeouts_still_expire_at_the_new_limits(self):
        from bunker_rgbd.can_feedback import Feedback
        self.assertTrue(fresh(dict(stamp=9.11,_rx=9.11),10.,10.))
        self.assertFalse(fresh(dict(stamp=9.09,_rx=9.09),10.,10.))
        i=inputs();g=Safety(True);g.grant(True,10.,10.,i)
        i['mission'].update(active=True,state='APPROACH')
        command=dict(stamp=9.41,_rx=9.41,generation='g',state='APPROACH',v=.05,w=0.)
        self.assertGreater(g.tick(10.,10.,i,command)[0][0],0.)
        command.update(stamp=9.39,_rx=9.39)
        self.assertEqual(g.tick(10.,10.,i,command)[0],(0.,0.))
        fb=Feedback();fb.ingest(0x211,bytes(8),10.);fb.ingest(0x221,bytes(8),10.)
        self.assertTrue(fb.snapshot(10.89)['live']);self.assertFalse(fb.snapshot(10.91)['live'])
    def test_pid_bounded_and_integral_does_not_wind_up(self):
        p=PID(.7,.2,.04,-.12,.12)
        for _ in range(100):self.assertLessEqual(abs(p.update(2.,.05)),.12)
        self.assertEqual(p.integral,0.)
        self.assertGreater(stop_boundary(.08),stop_boundary(0.))
    def test_authorization_default_off_and_disarm_always_available(self):
        g=Safety();self.assertFalse(g.grant(True,10.,10.,self.i)[0])
        self.assertTrue(g.grant(False,10.,10.,{})[0])
    def test_basic_faults_stop_guard(self):
        for fault in ('stale_depth','bad_depth','lost_feedback','stale_command','obstacle'):
            with self.subTest(fault=fault):
                i=inputs();g=Safety(True);self.assertTrue(g.grant(True,10.,10.,i)[0])
                i['mission'].update(active=True,state='APPROACH')
                c=dict(stamp=10.,_rx=10.,generation='g',state='APPROACH',v=.05,w=0.)
                if fault=='stale_depth':i['depth']['stamp']=9.
                if fault=='bad_depth':i['depth']['valid']=False
                if fault=='lost_feedback':i['feedback']['live']=False
                if fault=='stale_command':c['stamp']=9.
                if fault=='obstacle':i['depth']['points']=[(.4,.34)]
                self.assertEqual(g.tick(10.,10.,i,c)[0],(0.,0.))
    def test_first_detection_wins_not_nearest_or_highest_confidence(self):
        self.i['observation']['targets'].append(dict(valid=True,stable=True,track_id='nearer',gap=.5,bearing=0.))
        self.step();self.assertEqual(self.m.target,'bin');self.assertAlmostEqual(self.m.goal[0],.70)
        tr=Tracker();rows=[dict(valid=True,bbox=[0,0,20,20],score=.4),dict(valid=True,bbox=[40,40,60,60],score=.9)]
        self.assertEqual(tr.update(rows,1.)[0]['score'],.4)
        self.setUp();self.i['observation']['targets'][0]['valid']=False
        self.i['observation']['targets'].append(dict(valid=True,track_id='second',gap=.5,bearing=0.))
        self.assertEqual(self.step(),(0.,0.));self.assertEqual(self.m.target,'bin')
        self.assertIsNone(self.m.goal)  # Do not replace first target while waiting for its depth.
    def test_saved_model_bin_synonyms_are_all_task_targets(self):
        # Actual missed scene: model class 4, recycling bin, confidence 0.853.
        for label in ('trash can','garbage bin','waste bin','dustbin','recycling bin','Recycling_Bin'):
            self.assertTrue(is_target_label(label),label)
        for label in ('person','chair','bottle','potted plant'):
            self.assertFalse(is_target_label(label),label)
    def test_no_map_or_nav_dependencies_in_launch(self):
        from pathlib import Path
        import ast
        p=Path(__file__).resolve().parents[1]/'src/bunker_rgbd/launch/task.launch.py'
        tree=ast.parse(p.read_text())
        packages={kw.value.value for n in ast.walk(tree) if isinstance(n,ast.Call) for kw in n.keywords
                  if kw.arg=='package' and isinstance(kw.value,ast.Constant)}
        self.assertEqual(packages,{'tf2_ros','bunker_rgbd','bunker_base'})
    def test_search_timeout_without_target(self):
        self.i['observation']['targets']=[];self.assertEqual(self.step(10.+SEARCH_TIMEOUT+1.),(0.,0.));self.assertEqual(self.m.state,'HOLD')


if __name__=='__main__':unittest.main()
