import math
import unittest
from robot_mission.core import Mission, Goal, Grid, docking_goals, approach_limit, fresh


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.m = Mission()
        self.m.start(0.)
        self.m.tick(.1, .1, True, True, {}, {})
        self.event = dict(task_generation=self.m.generation, observation_id='o1', request_id='r1',
                          candidates=[dict(id=1, track_id='t1')])
        self.target = dict(track_id='t1', valid=True, map_point=[2., 0., .5],
                           base_point=[2., 0., .5], gap=1.655, bearing=0., stamp=1.)
        self.goal = Goal('t1', .98, 0., 0., (2., 0., .5), 1.)

    def confirm(self):
        return dict(task_generation=self.m.generation, observation_id='o1', request_id='r1', valid=True,
                    candidates=[dict(id=1, track_id='t1', status='match')])

    def navigate(self):
        self.assertTrue(self.m.candidate_event(self.event, .2))
        self.assertTrue(self.m.confirmation(self.confirm(), .4))
        self.m.tick(1., 1., True, True, {'t1': self.target}, {'t1': (self.goal, 1.)})
        self.assertEqual(self.m.state, 'READY_TO_NAV')
        self.m.navigation_accepted()

    def test_no_candidates_no_confirmation_or_navigation(self):
        self.event['candidates'] = []
        self.assertFalse(self.m.candidate_event(self.event, .2))
        self.assertEqual(self.m.state, 'SEARCH_CONTINUOUS')
        self.assertIsNone(self.m.pending)

    def test_invalid_ids_and_generation(self):
        self.m.candidate_event(self.event, .2)
        response = self.confirm()
        response['candidates'][0]['id'] = 2
        self.assertFalse(self.m.confirmation(response, .4))
        self.m.tick(1., 1., True, True, {'t1': self.target}, {'t1': (self.goal, 1.)})
        self.assertEqual(self.m.state, 'SEARCH_CONTINUOUS')
        self.m.start(2.)
        self.assertFalse(self.m.confirmation(self.confirm(), 2.1))

    def test_confirmation_requires_standstill_and_fresh_geometry(self):
        self.m.candidate_event(self.event, .2)
        self.m.confirmation(self.confirm(), .4)
        self.m.tick(1., 1., True, False, {'t1': self.target}, {'t1': (self.goal, 1.)})
        self.assertEqual(self.m.state, 'ACQUIRE')
        self.m.tick(2., 2., True, True, {'t1': self.target}, {'t1': (self.goal, 1.)})
        self.assertEqual(self.m.state, 'WAIT_PATH')

    def test_timeout_discards_late_reply(self):
        self.m.candidate_event(self.event, .2)
        self.m.tick(9., 9., True, True, {}, {})
        self.assertFalse(self.m.confirmation(self.confirm(), 9.1))
        self.assertEqual(self.m.state, 'SEARCH_CONTINUOUS')

    def test_continuous_nav_to_approach_then_verified_stop(self):
        self.navigate()
        epoch = self.m.epoch
        self.target.update(gap=1.3, stamp=1.1)
        self.m.tick(1.1, 1.1, True, False, {'t1': self.target}, {})
        self.assertEqual((self.m.state, self.m.epoch, self.m.source), ('APPROACH', epoch, 'nav'))
        self.target.update(gap=.62, stamp=1.2)
        self.m.tick(1.2, 1.2, True, False, {'t1': self.target}, {})
        self.assertEqual(self.m.state, 'VERIFY_STOP')
        self.assertEqual(self.m.source, 'none')
        self.target['stamp'] = 1.3
        self.m.tick(1.3, 1.3, True, True, {'t1': self.target}, {})
        self.target['stamp'] = 2.1
        self.m.tick(2.1, 2.1, True, True, {'t1': self.target}, {})
        self.assertEqual(self.m.state, 'DONE')

    def test_nav_success_not_physical_success(self):
        self.navigate()
        self.m.navigation_result(True)
        self.m.tick(1.1, 1.1, True, True, {'t1': self.target}, {})
        self.assertEqual(self.m.state, 'HOLD')

    def test_target_loss_and_movement_stop(self):
        self.navigate()
        self.target.update(gap=1.3, stamp=1.1)
        self.m.tick(1.1, 1.1, True, False, {'t1': self.target}, {})
        self.m.tick(1.2, 1.2, True, False, {}, {})
        self.assertEqual(self.m.state, 'REOBSERVE_RECOVERY')
        self.assertEqual(self.m.source, 'none')

    def test_readiness_loss_and_total_deadline(self):
        self.m.tick(1., 1., False, True, {}, {})
        self.assertEqual(self.m.state, 'HOLD')
        self.m.start(1.)
        self.m.tick(182., 182., True, True, {}, {})
        self.assertEqual(self.m.state, 'HOLD')

    def test_sweep_and_physical_front_offset(self):
        grid = Grid(200, 200, .05, (-5., -5., 0.), [0]*40000)
        goals = docking_goals(self.target, (0., 0.), grid)
        self.assertAlmostEqual(goals[0].x, 2.-.345-.60-.015)
        # An obstacle at the side of the vehicle blocks even if center is free.
        ix, iy = math.floor((goals[0].x+5.)/.05), math.floor((.4+5.)/.05)
        grid.data[iy*200+ix] = 100
        self.assertEqual(docking_goals(self.target, (0., 0.), grid), [])
        self.assertFalse(grid.free(99., 99.))

    def test_depth_freshness_and_braking_envelope(self):
        self.assertFalse(fresh(float('nan'), 1., .3))
        self.assertFalse(fresh(2., 1., .3))
        self.assertEqual(approach_limit(.55), 0.)
        self.assertLess(approach_limit(.601), approach_limit(.9))
        self.assertLessEqual(approach_limit(1.), .05)


if __name__ == '__main__':
    unittest.main()
