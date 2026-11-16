"""Acquisition and asynchronous result validity when SLAM changes geometry."""
import ast
import json
from pathlib import Path
import queue
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from robot_perception.context import ObservationContext, can_request


class ObservationContextTests(unittest.TestCase):
    def setUp(self):
        self.ctx = ObservationContext()
        self.heartbeat('a', 0, 10.)

    def heartbeat(self, generation, epoch, stamp, state='SEARCH_CONTINUOUS', reconfirm=False):
        return self.ctx.update(dict(schema=1, task_generation=generation, geometry_epoch=epoch,
                               stamp=stamp, state=state, reconfirm_required=reconfirm), stamp, stamp)

    def test_recovery_events_require_explicit_reconfirmation(self):
        self.assertTrue(can_request('SEARCH_CONTINUOUS', False))
        self.assertFalse(can_request('WAIT_PATH', False))
        self.assertFalse(can_request('REOBSERVE_RECOVERY', False))
        self.assertTrue(can_request('REOBSERVE_RECOVERY', True))
        self.assertFalse(can_request('HOLD', True))

    def test_normal_map_updates_do_not_change_capture_context(self):
        self.heartbeat('a', 0, 11., state='WAIT_PATH')
        self.assertTrue(self.ctx.accepts_capture(10.5))
        self.assertEqual(self.ctx.since_stamp, 10.)
        self.assertTrue(self.ctx.matches(dict(task_generation='a', geometry_epoch=0)))

    def test_reset_rejects_pre_reset_frame_even_when_delivered_later(self):
        self.heartbeat('b', 1, 11., state='REOBSERVE_RECOVERY', reconfirm=True)
        self.assertFalse(self.ctx.accepts_capture(10.95))
        self.assertTrue(self.ctx.accepts_capture(11.01))
        self.assertFalse(self.ctx.matches(dict(task_generation='a', geometry_epoch=0)))
        self.assertFalse(self.ctx.matches(dict(task_generation='b', geometry_epoch=0)))
        self.assertTrue(self.ctx.matches(dict(task_generation='b', geometry_epoch=1)))

    def test_retired_and_reordered_mission_messages_do_not_restore_old_context(self):
        self.heartbeat('b', 1, 11.)
        self.assertFalse(self.heartbeat('a', 0, 11.2))
        self.assertFalse(self.heartbeat('b', 0, 11.2))
        self.assertFalse(self.heartbeat('b', 1, 10.9))
        self.assertEqual(self.ctx.key, ('b', 1))

    def test_stale_heartbeat_cannot_trigger_requests(self):
        self.assertTrue(self.ctx.permits_event(10.1, 10.1))
        self.assertFalse(self.ctx.permits_event(11.1, 10.2))
        self.assertFalse(self.ctx.permits_event(10.2, 11.1))


class DeferredOutputTests(unittest.TestCase):
    """Execute the actual ROS adapter output methods with transport mocks."""
    @staticmethod
    def method(filename, classname, name):
        path = Path(__file__).resolve().parents[1]/'src/robot_perception/robot_perception'/filename
        tree = ast.parse(path.read_text())
        cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == classname)
        function = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name == name)
        context = dict(json=json, time=NS(monotonic=lambda: 20.1), String=lambda **kw: NS(**kw))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), context)
        return context[name]

    def setUp(self):
        self.ctx = ObservationContext()
        self.ctx.update(dict(schema=1, task_generation='new', geometry_epoch=3, stamp=20.,
                             state='REOBSERVE_RECOVERY', reconfirm_required=True), 20., 20.)

    def test_queued_old_frame_and_event_dropped_new_recovery_event_published(self):
        node = NS(observation_context=self.ctx, now=lambda: 20.1, cfg={'max_frame_age': .5},
                  events=queue.Queue(), results=queue.Queue(),
                  data_publishers={key: NS(publish=Mock()) for key in ('event', 'candidates', 'estimates')})
        old = dict(task_generation='old', geometry_epoch=2, stamp=20.05)
        new = dict(task_generation='new', geometry_epoch=3, stamp=20.05)
        pre_reset = dict(task_generation='new', geometry_epoch=3, stamp=19.99)
        for event in (old, pre_reset, new):
            node.events.put(event)
            node.results.put({'candidates': event, 'estimates': event})
        self.method('perception_node.py', 'PerceptionNode', 'publish_results')(node)
        for publisher in node.data_publishers.values():
            publisher.publish.assert_called_once()
            self.assertEqual(json.loads(publisher.publish.call_args.args[0].data), new)

    def test_new_reset_event_replaces_obsolete_bounded_queue_slot(self):
        node = NS(observation_context=self.ctx, now=lambda: 20.1, lock=threading.RLock(), events=queue.Queue(maxsize=1))
        old = dict(task_generation='old', geometry_epoch=2, stamp=20.05)
        new = dict(task_generation='new', geometry_epoch=3, stamp=20.05)
        node.events.put(old)
        method = self.method('perception_node.py', 'PerceptionNode', 'put_event')
        # Queue implementation is part of the adapter method's execution context.
        method.__globals__['queue'] = queue
        self.assertTrue(method(node, new))
        self.assertEqual(node.events.get_nowait(), new)
        self.assertFalse(method(node, old))
        self.assertTrue(node.events.empty())

    def test_late_network_response_from_old_context_not_published(self):
        node = NS(observation_context=self.ctx, now=lambda: 20.1, publisher=NS(publish=Mock()))
        event = dict(task_generation='old', geometry_epoch=2, stamp=20.05)
        self.method('vlm_confirm_node.py', 'VLMConfirmNode', 'emit')(node, event, True, {1:'match'})
        node.publisher.publish.assert_not_called()

    def test_confirmation_includes_program_owned_geometry_epoch(self):
        node = NS(observation_context=self.ctx, now=lambda: 20.1, publisher=NS(publish=Mock()))
        event = dict(task_generation='new', geometry_epoch=3, stamp=20.05,
                     observation_id='o', request_id='r', candidates=[dict(id=1, track_id='new:3:1')])
        self.method('vlm_confirm_node.py', 'VLMConfirmNode', 'emit')(node, event, True, {1:'match'})
        emitted = json.loads(node.publisher.publish.call_args.args[0].data)
        self.assertEqual(emitted['geometry_epoch'], 3)
        self.assertEqual(emitted['candidates'][0]['track_id'], 'new:3:1')


if __name__ == '__main__':
    unittest.main()
