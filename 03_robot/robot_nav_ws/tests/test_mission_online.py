import math
import unittest
from robot_mission.core import Mission, Goal, Grid
from robot_mission.mapping import CachedPlan, classify_update, revalidate_cache, plan_valid, remaining_path


def grid(width=160, height=160, origin=(-4., -4., 0.), value=0):
    return Grid(width, height, .05, origin, [value]*(width*height))


class OnlineMapTests(unittest.TestCase):
    def setUp(self):
        self.grid = grid()
        self.goal = Goal('bin', 1., 0., 0., (1.96, 0., .5), 2.)
        self.plan = CachedPlan(self.goal, 1., ((0., 0.), (.5, 0.), (1., 0.)), 1, 0)

    def test_growth_boundary_translation_and_new_cells_preserve_valid_path(self):
        grown = grid(240, 200, (-6., -5., 0.))
        self.assertEqual(classify_update(self.grid, grown), 'update')
        retained = revalidate_cache({'bin': self.plan}, grown, 2, 0)
        self.assertIn('bin', retained)
        self.assertEqual(retained['bin'].map_revision, 2)
        grown.data[0] = 100  # distant obstacle update also benign
        self.assertTrue(plan_valid(self.plan, grown, 0))

    def test_path_obstacle_and_unknown_are_invalid_without_claiming_reset(self):
        for occupancy in (100, -1):
            changed = grid()
            changed.data[80*160+90] = occupancy  # directly on path x=.5, y=0
            self.assertEqual(classify_update(self.grid, changed), 'update')
            self.assertFalse(plan_valid(self.plan, changed, 0))
            self.assertEqual(revalidate_cache({'bin': self.plan}, changed, 2, 0), {})

    def test_reset_detected_without_confusing_map_origin_translation(self):
        self.assertEqual(classify_update(self.grid, grid(200,200,(-5.,-5.,0.))), 'update')
        self.assertEqual(classify_update(self.grid, grid(20,20,(-.5,-.5,0.))), 'reset')
        changed = grid(); changed.resolution=.10
        self.assertEqual(classify_update(self.grid, changed), 'reset')
        self.assertEqual(classify_update(self.grid, self.grid, 1., 2.), 'reset')

    def test_prior_geometry_epoch_path_cannot_be_used(self):
        self.assertFalse(plan_valid(self.plan, self.grid, 1))

    def test_only_untraversed_path_checked_after_moving(self):
        changed = grid()
        changed.data[80*160+80] = 100
        self.assertFalse(plan_valid(self.plan, changed, 0))
        self.assertTrue(plan_valid(self.plan, changed, 0, (1.,0.)))
        self.assertEqual(remaining_path(self.plan.path, (1.,0.))[-1], (1.,0.))


class OnlineTaskTests(unittest.TestCase):
    def setUp(self):
        self.task = Mission()
        self.task.start(0.)
        self.task.tick(.1,.1,True,True,{}, {})
        self.event = dict(task_generation=self.task.generation, observation_id='o', request_id='r',
                          candidates=[dict(id=1,track_id='bin')])
        self.target = dict(valid=True, stamp=1., map_point=[1.96,0.,.5], gap=1.615,bearing=0.)
        self.goal = Goal('bin',1.,0.,0.,(1.96,0.,.5),1.)
        self.plan = CachedPlan(self.goal,1.,((0.,0.),(1.,0.)),1,0)

    def confirm(self):
        self.task.candidate_event(self.event,.2)
        self.task.confirmation(dict(self.event,valid=True,candidates=[dict(id=1,track_id='bin',status='match')]),.3)

    def test_initial_local_map_suffices_then_candidate_exits_search_before_circle(self):
        self.assertEqual(self.task.state,'SEARCH_CONTINUOUS')
        self.confirm()
        self.task.tick(1.,1.,True,True,{'bin':self.target},{'bin':self.plan})
        self.assertEqual(self.task.state,'READY_TO_NAV')
        self.assertLess(self.task.coverage,2*math.pi)
        self.task.navigation_accepted()
        self.assertEqual(self.task.source,'nav')

    def test_visible_confirmed_bin_without_path_stays_parked_after_api_deadline(self):
        self.confirm()
        self.task.tick(1.,1.,True,True,{'bin':self.target},{})
        self.assertEqual((self.task.state,self.task.source),('WAIT_PATH','none'))
        self.task.tick(12.,12.,True,True,{}, {})
        self.assertEqual((self.task.state,self.task.source),('WAIT_PATH','none'))
        # Fresh observations and a new path can resume without a new API or full circle.
        self.target['stamp']=12.1
        goal=Goal('bin',1.,0.,0.,(1.96,0.,.5),12.1)
        self.task.tick(12.1,12.1,True,True,{'bin':self.target},{'bin':(goal,1.)})
        self.assertEqual(self.task.state,'READY_TO_NAV')

    def test_blocked_path_recovery_requires_post_stop_frame_and_terminal_cancel(self):
        self.confirm();self.task.tick(1.,1.,True,True,{'bin':self.target},{'bin':self.plan})
        self.task.navigation_accepted()
        generation=self.task.generation
        self.task.recover('path blocked',2.,2.)
        self.assertEqual(self.task.source,'none')
        self.assertEqual(self.task.generation,generation)
        self.task.tick(2.1,2.1,True,True,{'bin':self.target},{'bin':self.plan})
        self.assertEqual(self.task.state,'REOBSERVE_RECOVERY')
        self.target['stamp']=2.2
        fresh_goal=Goal('bin',1.,0.,0.,(1.96,0.,.5),2.2)
        plans={'bin':(fresh_goal,1.)}
        self.task.tick(2.2,2.2,True,True,{'bin':self.target},plans,navigation_idle=False)
        self.assertEqual(self.task.source,'none')
        self.task.tick(2.3,2.3,True,True,{'bin':self.target},plans,navigation_idle=True)
        self.assertEqual(self.task.state,'READY_TO_NAV')

    def test_reset_requires_fresh_semantic_context_and_rejects_old_confirmation(self):
        self.confirm()
        old_generation=self.task.generation
        self.task.recover('SLAM reset',2.,2.,reconfirm=True)
        self.assertNotEqual(self.task.generation,old_generation)
        self.assertEqual(self.task.geometry_epoch,1)
        self.assertTrue(self.task.reconfirm_required)
        self.assertFalse(self.task.candidate_event(self.event,2.1))
        new=dict(self.event,task_generation=self.task.generation,request_id='new')
        self.assertTrue(self.task.candidate_event(new,2.2))
        self.assertEqual((self.task.state,self.task.source),('CONFIRM_PENDING','none'))

    def test_065_upper_limit_and_055_lower_limit_enforced(self):
        self.confirm();self.task.tick(1.,1.,True,True,{'bin':self.target},{'bin':self.plan})
        self.task.navigation_accepted()
        self.target.update(stamp=1.1,gap=.66)
        self.task.tick(1.1,1.1,True,False,{'bin':self.target},{})
        self.assertEqual(self.task.state,'APPROACH')
        self.target.update(stamp=1.2,gap=.62)
        self.task.tick(1.2,1.2,True,True,{'bin':self.target},{})
        self.assertEqual(self.task.state,'VERIFY_STOP')
        self.target.update(stamp=1.3,gap=.54)
        self.task.tick(1.3,1.3,True,True,{'bin':self.target},{})
        self.assertEqual((self.task.state,self.task.source),('HOLD','none'))

if __name__=='__main__':unittest.main()
