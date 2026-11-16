"""Keyboard/service ordering and truthful stop feedback, without ROS/hardware."""
import unittest
from concurrent.futures import Future
from types import SimpleNamespace
from bunker_rgbd.window_control import WindowControl
from bunker_rgbd.timing import STOP_WAIT_TIMEOUT


class WindowTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        def send(action, value=None):
            future = Future()
            self.calls.append((action, value, future))
            return future
        self.control = WindowControl(send)

    def complete(self, success):
        self.calls[-1][2].set_result(SimpleNamespace(success=success, message='test response'))

    def feedback(self, now, live=True):
        c = self.control
        c.safety = dict(stamp=now, _rx=now, authorized=False, output_zero=True,
                        output_topic='/cmd_vel', feedback_live=live, feedback_stopped=live)
        c.mission = dict(stamp=now, _rx=now, active=False)
        c.odom = dict(stamp=now, _rx=now, v=0., w=0.)

    def test_start_requires_explicit_key_and_successful_guard_response(self):
        self.control.tick(10., 10.)
        self.assertEqual(self.calls, [])
        self.control.key(ord('s'), 10.)
        self.assertEqual(self.calls[-1][:2], ('arm', True))
        self.complete(False)
        self.control.tick(10.1, 10.1)
        self.assertNotIn('start', [call[0] for call in self.calls])
        self.assertIn('denied', self.control.message)
        self.control.key(ord('s'), 11.)
        self.complete(True)
        self.control.tick(11.1, 11.1)
        self.assertEqual(self.control.stage,'waiting_ready')
        self.assertNotIn('start',[call[0] for call in self.calls])
        self.control.safety=dict(stamp=11.2,_rx=11.2,authorized=True,ready=True)
        self.control.mission=dict(stamp=11.2,_rx=11.2,active=False,start_ready=True)
        self.control.tick(11.2,11.2)
        self.assertEqual(self.calls[-1][0], 'start')
        self.complete(True)
        self.control.tick(11.3, 11.3)
        self.assertEqual(self.control.stage, 'idle')

    def test_stop_cancels_start_chain_and_waits_for_fresh_standstill(self):
        c = self.control
        c.key(ord('s'), 10.)
        arm = self.calls[-1][2]
        c.key(32, 10.1)
        self.assertEqual([call[:2] for call in self.calls][-2:], [('arm', False), ('stop', None)])
        arm.set_result(SimpleNamespace(success=True, message='late grant'))
        self.feedback(10.2)
        c.tick(10.2, 10.2)
        self.assertNotIn('start', [call[0] for call in self.calls])
        self.assertEqual(self.calls[-1][:2], ('arm', False))
        self.feedback(10.8, live=False)
        c.tick(10.8, 10.8)
        self.assertEqual(c.stage, 'stopping')
        self.feedback(11.)
        c.tick(11., 11.)
        self.feedback(11.8)
        c.tick(11.8, 11.8)
        self.assertEqual(c.message, 'Stopped; motion disabled')

    def test_close_without_feedback_exits_but_never_claims_stopped(self):
        self.control.key(ord('q'), 10.)
        self.control.tick(10.+STOP_WAIT_TIMEOUT+.1,10.+STOP_WAIT_TIMEOUT+.1)
        self.assertTrue(self.control.exit_ready)
        self.assertIn('UNKNOWN', self.control.message)


if __name__ == '__main__':
    unittest.main()
