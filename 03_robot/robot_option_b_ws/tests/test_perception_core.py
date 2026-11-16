import json
import unittest
import numpy as np
from robot_perception.core import CandidateTracker, depth_surface, strict_result, transform_matrix


class ConfirmationTests(unittest.TestCase):
    def test_exact_ids_and_statuses(self):
        self.assertEqual(strict_result('{"candidates":[{"id":2,"status":"match"},{"id":1,"status":"uncertain"}]}',{1,2}), {1:'uncertain',2:'match'})

    def test_malformed_incomplete_and_injected_results(self):
        bad = ['not json', '{"candidates":[]}',
               '{"candidates":[{"id":1,"status":"match"},{"id":1,"status":"match"}]}',
               '{"candidates":[{"id":7,"status":"match"}]}',
               '{"candidates":[{"id":true,"status":"match"}]}',
               '{"candidates":[{"id":1,"status":"match","cmd_vel":1}]}',
               '{"candidates":[{"id":1,"status":"match"}],"candidates":[]}',
               '{"candidates":[{"id":1,"status":"go"}]}']
        for value in bad:
            with self.subTest(value=value), self.assertRaises(ValueError):
                strict_result(value,{1})


class TrackingTests(unittest.TestCase):
    def detection(self, x=10, point=None, quality=50):
        return dict(bbox=[x,10,x+50,80], score=.8, map_point=point, image_quality=quality, image_usable=True)

    def test_empty_scene_never_requests(self):
        t = CandidateTracker('task')
        for i in range(100):
            rows = t.update([],i*.1)
            self.assertIsNone(t.event(rows,i*.1))

    def trigger(self,t, start=0):
        t.update([self.detection()],start)
        rows=t.update([self.detection()],start+.1)
        self.assertIsNone(t.event(rows,start+.1))
        rows=t.update([self.detection()],start+.31)
        return t.event(rows,start+.31)

    def test_stability_pending_dedup_and_old_request(self):
        t=CandidateTracker('a')
        rid, rows = self.trigger(t)
        self.assertEqual(len(rows),1)
        for i in range(10):
            now=.4+i*.1
            self.assertIsNone(t.event(t.update([self.detection()],now),now))
        self.assertFalse(t.apply('other',{1:'match'},1.5))
        self.assertTrue(t.apply(rid,{1:'match'},1.5))
        self.assertIsNone(t.event(t.update([self.detection()],1.6),1.6))
        t.reset('b')
        self.assertFalse(t.apply(rid,{1:'match'},1.7))

    def test_geometry_epoch_reset_clears_map_identity_and_rejected_cache(self):
        t = CandidateTracker('a')
        rid, rows = self.trigger(t)
        before = rows[0]['track_id']
        t.apply(rid, {1:'match'}, .4)
        t.rejected = [([1., 0., .5], 10.)]
        t.reset('a', geometry_epoch=1)
        self.assertFalse(t.apply(rid, {1:'match'}, .5))
        self.assertEqual(t.rejected, [])
        _, rows = self.trigger(t, 1.)
        self.assertNotEqual(rows[0]['track_id'], before)

    def test_old_context_worker_busy_defers_without_semantic_rejection(self):
        t = CandidateTracker('new', geometry_epoch=2)
        rid, rows = self.trigger(t)
        track = rows[0]['track_id']
        t.apply(rid, {}, .4, valid=False, error='worker_busy', retry_after=2.)
        self.assertEqual(t.tracks[track].state, 'RETRY')
        self.assertEqual(t.tracks[track].attempts, 0)
        for i in range(5, 24):
            now = i/10.
            self.assertIsNone(t.event(t.update([self.detection()], now), now))
        # Same clear view may be retried because its first API request never ran.
        self.assertIsNone(t.event(t.update([self.detection()], 2.5), 2.5))
        self.assertIsNotNone(t.event(t.update([self.detection()], 2.71), 2.71))

    def test_map_motion_compensation_and_ambiguity(self):
        t=CandidateTracker('a')
        first=t.update([self.detection(10,[1.,0.,.5])],0)[0]['track_id']
        moved=t.update([self.detection(150,[1.01,0.,.5])],.1)
        self.assertEqual(first,moved[0]['track_id'])
        self.assertEqual(t.update([self.detection(150,[1.,0.,.5]),self.detection(220,[1.02,0.,.5])],.2), [])

    def test_confirmed_static_map_record_reassociates_after_occlusion(self):
        t=CandidateTracker('a')
        for now in (0,.1,.31):
            rows=t.update([self.detection(point=[1,0,.5])],now)
            event=t.event(rows,now)
        rid,rows=event
        track=rows[0]['track_id']
        t.apply(rid,{1:'match'},.4)
        t.update([],2.)
        reacquired=t.update([self.detection(200,[1.02,0,.5])],3.)
        self.assertEqual(reacquired[0]['track_id'],track)
        self.assertFalse(reacquired[0]['eligible'])
        # Invalid geometry cannot associate an old off-screen track by pixel alone.
        lost=t.update([self.detection(350)],4.)
        self.assertNotEqual(lost[0]['track_id'],track)

    def test_refused_instance_cache_survives_track_loss(self):
        t=CandidateTracker('a')
        for now in (0,.1,.31):
            rows=t.update([self.detection(point=[1,0,.5])],now)
            event=t.event(rows,now)
        rid,_=event
        t.apply(rid,{1:'no_match'},.4)
        t.update([],1.2)
        rows=t.update([self.detection(200,[1.05,0,.5])],1.3)
        self.assertFalse(rows[0]['eligible'])
        self.assertEqual(t.tracks[rows[0]['track_id']].state,'REJECTED')


class DepthTests(unittest.TestCase):
    def setUp(self):
        self.k=[100,0,50,0,100,50,0,0,1]
        # optical z -> base x, optical x -> -base y, optical y -> -base z
        self.tf=np.array([[0,0,1,.2],[-1,0,0,0],[0,-1,0,.6],[0,0,0,1.]])
        self.args=([10,10,90,90],self.k,[],'plumb_bob',self.tf,self.tf)

    def test_millimeter_and_float_encoding_same_surface(self):
        a=depth_surface(np.ones((100,100),np.uint16)*1000,'16UC1',*self.args)
        b=depth_surface(np.ones((100,100),np.float32),'32FC1',*self.args)
        self.assertTrue(a['valid'],a)
        self.assertAlmostEqual(a['gap'],.855)
        self.assertAlmostEqual(a['gap'],b['gap'])

    def test_holes_wrong_units_and_mixed_surfaces_rejected(self):
        for depth,encoding in [(np.zeros((100,100)),'16UC1'),(np.ones((100,100))*1000,'32FC1')]:
            self.assertFalse(depth_surface(depth,encoding,*self.args)['valid'])
        d=np.ones((100,100))*1000
        d[:,50:]=2500
        self.assertFalse(depth_surface(d,'16UC1',*self.args)['valid'])

    def test_no_map_or_base_transform_is_not_navigable(self):
        d=np.ones((100,100))*1000
        self.assertFalse(depth_surface(d,'16UC1',*self.args[:-1],None)['valid'])
        self.assertFalse(depth_surface(d,'16UC1',self.args[0],self.k,[],'plumb_bob',None)['valid'])

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            transform_matrix([0,0,0],[0,0,0,0])


if __name__=='__main__':
    unittest.main()
