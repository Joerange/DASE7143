import importlib.util
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import math
import collections
import threading
import unittest
from robot_console.rendering import (matching_candidates, velocity_text, world_to_grid, body_polygon,
                                     control_enabled, timeline_category, authorization_value, current_mission_state, LatestSlot)


class ConsoleRenderingTests(unittest.TestCase):
    def test_cached_authorization_and_mission_require_fresh_feeds(self):
        snapshot={'safety':{'authorized':True},'mission':{'state':'NAVIGATE'},
                  'sensors':{'safety':{'state':'normal','age':.1},'mission':{'state':'normal','age':.1}}}
        self.assertIs(authorization_value(snapshot),True)
        self.assertEqual(current_mission_state(snapshot),'NAVIGATE')
        snapshot['sensors']['safety']['age']=.31
        snapshot['sensors']['mission']['state']='stale'
        self.assertIsNone(authorization_value(snapshot))
        self.assertIsNone(current_mission_state(snapshot))
        snapshot['sensors']['safety']={'state':'normal','age':.02}
        snapshot['safety']['authorized']=False
        self.assertIs(authorization_value(snapshot),False)

    def test_latest_snapshot_slot_bounds_pending_gui_updates(self):
        slot=LatestSlot()
        notifications=0
        for i in range(10000):
            notifications+=int(slot.offer({'sequence':i}))
        self.assertEqual(notifications,1)
        self.assertEqual(slot.take(),{'sequence':9999})
        self.assertIsNone(slot.take())
        self.assertTrue(slot.offer({'sequence':10000}))
        self.assertFalse(slot.offer({'sequence':10001}))
        self.assertEqual(slot.take(),{'sequence':10001})

    def test_detection_boxes_require_exact_acquisition_frame(self):
        row={'bbox':[10.,20.,50.,80.]}
        self.assertEqual(matching_candidates({'stamp':1.},{'stamp':1.,'candidates':[row]}),[row])
        self.assertEqual(matching_candidates({'stamp':1.},{'stamp':1.03,'candidates':[row]}),[])
        self.assertEqual(matching_candidates({'stamp':1.,'observation_id':'a'},
                                            {'stamp':1.,'observation_id':'b','candidates':[row]}),[])

    def test_can_up_does_not_turn_unknown_speed_into_zero(self):
        self.assertIsNone(velocity_text({'can_up':True,'velocity':{'valid':False,'linear_x':0.,'angular_z':0.,'age':.02}}))
        self.assertIsNone(velocity_text({'velocity':{'valid':True,'linear_x':0.,'angular_z':0.,'age':.8}}))
        self.assertEqual(velocity_text({'velocity':{'valid':True,'linear_x':.04,'angular_z':.02,'age':.1}}),(.04,.02,.1))

    def test_map_expansion_origin_and_rotation_are_respected(self):
        m={'width':100,'height':80,'resolution':.1,'origin':[0.,0.,0.]}
        self.assertEqual(world_to_grid(1.,2.,m),(10.,60.))
        m.update(width=140,height=120,origin=[-2.,-3.,0.])
        self.assertEqual(world_to_grid(1.,2.,m),(30.,70.))
        m.update(origin=[0.,0.,math.pi/2])
        gx,gy=world_to_grid(0.,1.,m)
        self.assertAlmostEqual(gx,10.);self.assertAlmostEqual(gy,120.)

    def test_physical_body_and_control_defaults(self):
        self.assertEqual(body_polygon((0.,0.,0.))[0],(.345,.285))
        self.assertEqual(control_enabled({},'arm'),(False,''))
        self.assertEqual(control_enabled({'arm':{'enabled':False,'reason':'no odom'}},'arm'),(False,'no odom'))

    def test_timeline_keeps_semantic_and_local_decisions_distinct(self):
        self.assertEqual(timeline_category({'kind':'vlm_confirmation'}),'VLM')
        self.assertEqual(timeline_category({'kind':'candidate_event'}),'DETECTION')
        self.assertEqual(timeline_category({'kind':'mission_state'}),'LOCAL')

    def test_demo_can_up_requires_actual_communication_before_arm(self):
        from robot_console.demo import FakeClient
        demo=FakeClient()
        self.assertTrue(demo.can_up)
        self.assertFalse(demo.request({'op':'action','action':'arm'})['ok'])
        self.assertTrue(demo.request({'op':'action','action':'driver_start'})['ok'])
        self.assertTrue(demo.request({'op':'action','action':'arm'})['ok'])
        self.assertTrue(demo.request({'op':'action','action':'mission_start'})['ok'])
        self.assertTrue(demo.request({'op':'action','action':'mission_stop'})['ok'])
        self.assertIsNone(demo.mission_started)

    def test_failed_can_off_requires_verified_standstill(self):
        from robot_console.demo import FakeClient
        demo=FakeClient()
        demo.inject_fault('missing_standstill')
        answer=demo.request({'op':'action','action':'can_disable'})
        self.assertTrue(answer['ok'])  # Accepted request is not successful readback.
        self.assertEqual(demo.operations[answer['operation_id']]['result'],'failed')
        self.assertTrue(demo.can_up)
        self.assertTrue(demo.driver_running)

    def test_stop_timeout_is_unknown_not_false_success(self):
        from robot_console.demo import FakeClient
        demo=FakeClient()
        demo.request({'op':'action','action':'driver_start'})
        demo.request({'op':'action','action':'arm'})
        demo.request({'op':'action','action':'mission_start'})
        demo.inject_fault('stop_timeout')
        answer=demo.request({'op':'action','action':'mission_stop'})
        self.assertTrue(answer['ok'])
        self.assertEqual(demo.operations[answer['operation_id']]['result'],'unknown')
        self.assertTrue(demo.can_up)
        self.assertFalse(demo.armed)
        self.assertIsNotNone(demo.mission_started)

    def test_reconnect_reads_actual_state_not_last_button_intent(self):
        from robot_console.demo import FakeClient
        demo=FakeClient()
        with patch.object(demo,'image_frames',return_value={}):
            before=demo.request({'op':'snapshot'})
            self.assertEqual(before['can']['state'],'UP')
            demo.set_connected(False)
            with self.assertRaises(ConnectionError):demo.request({'op':'snapshot'})
            # An external actor may change actual state while viewer is disconnected.
            demo.can_up=False;demo.driver_running=False;demo.communication_ok=False
            demo.set_connected(True)
            after=demo.request({'op':'snapshot'})
            self.assertEqual(after['can']['state'],'DOWN')
            self.assertFalse(after['chassis']['velocity']['valid'])
            self.assertFalse(after['controls']['mission_start'])

    def test_worker_timeout_marks_unknown_and_snapshot_failure_preserves_ack(self):
        path=Path(__file__).resolve().parents[1]/'src/robot_console/robot_console/ui.py'
        module=ast.parse(path.read_text())
        cls=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='BackendWorker')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='run')
        env={'time':NS(monotonic=lambda:10.)}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),env)
        for replies,expect_unknown in [([TimeoutError('reply lost')],True),
                                      ([{'ok':True,'operation_id':'accepted-op'},TimeoutError('snapshot lost')],False)]:
            quit_event=NS(stopped=False)
            quit_event.is_set=lambda:quit_event.stopped
            quit_event.wait=lambda seconds:setattr(quit_event,'stopped',True)
            worker=NS(actions=collections.deque(['can_disable']),lock=threading.Lock(),quit_event=quit_event,
                      client=NS(request=Mock(side_effect=replies)),response=NS(emit=Mock()),
                      disconnected=NS(emit=Mock()),latest_snapshot=LatestSlot(),snapshot=NS(emit=Mock()))
            env['run'](worker)
            worker.response.emit.assert_called_once()
            outcome=worker.response.emit.call_args.args[1]
            self.assertEqual(outcome.get('outcome')=='unknown',expect_unknown)
            if not expect_unknown:self.assertEqual(outcome['operation_id'],'accepted-op')

    def test_transport_unknown_is_not_reported_as_unexecuted(self):
        path=Path(__file__).resolve().parents[1]/'src/robot_console/robot_console/ui.py'
        module=ast.parse(path.read_text())
        cls=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='ConsoleWindow')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='action_result')
        env={'ACTIONS':[('can_disable','关闭 CAN')],'time':NS(time=lambda:10.)}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),env)
        window=NS(operation_id='previous-known-op',operation=NS(setText=Mock(),text=lambda:''),append_event=Mock())
        env['action_result'](window,'can_disable',{'ok':False,'outcome':'unknown','message':'reply lost'})
        text=window.operation.setText.call_args.args[0]
        self.assertIn('结果未知',text)
        self.assertNotIn('未执行',text)
        self.assertEqual(window.operation_id,'previous-known-op')
        self.assertEqual(window.ambiguous_action,'can_disable')
        env['action_result'](window,'can_disable',{'ok':False,'error':'server rejected operation'})
        self.assertIn('未执行',window.operation.setText.call_args.args[0])
        self.assertIsNone(window.ambiguous_action)

    def test_close_and_escape_do_not_send_control_operations(self):
        path=Path(__file__).resolve().parents[1]/'src/robot_console/robot_console/ui.py'
        module=ast.parse(path.read_text())
        cls=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='ConsoleWindow')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('closeEvent','exit_fullscreen')]
        env={}
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),env)
        window=NS(worker=NS(action=Mock()),isFullScreen=lambda:True,showNormal=Mock())
        event=NS(accept=Mock())
        env['closeEvent'](window,event)
        env['exit_fullscreen'](window)
        event.accept.assert_called_once()
        window.showNormal.assert_called_once()
        window.worker.action.assert_not_called()


@unittest.skipUnless(importlib.util.find_spec('PyQt5'),'PyQt5 is unavailable locally')
class ConsoleQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
        from PyQt5.QtWidgets import QApplication
        cls.app=QApplication.instance() or QApplication([])

    def test_demo_window_has_all_panels_and_persistent_stop(self):
        from robot_console.ui import ConsoleWindow
        from robot_console.demo import FakeClient
        window=ConsoleWindow(FakeClient(),demo=True)
        try:
            window.worker.quit_event.set();window.worker.wait(1500)
            window.receive(FakeClient().snapshot())
            self.assertEqual(set(window.panels),{'rgb','depth','lidar','map'})
            self.assertTrue(window.buttons['mission_stop'].isEnabled())
            self.assertFalse(window.buttons['arm'].isEnabled())
            self.assertIn('未知',window.speed.text())
            window.offline('fixture_disconnect')
            self.assertTrue(window.buttons['mission_stop'].isEnabled())
            self.assertFalse(window.buttons['mission_start'].isEnabled())
        finally:
            window.allow_close=True;window.close();window.worker.stop()

    def test_monitor_waiting_map_and_unknown_operation_render(self):
        from robot_console.ui import ConsoleWindow
        from robot_console.demo import FakeClient
        window=ConsoleWindow(FakeClient(),demo=True)
        try:
            window.worker.quit_event.set();window.worker.wait(1500)
            window.operation_id='timeout'
            window.receive({'map':None,'scan':{},'frames':{},'mission':{},'safety':{},'controls':{},
                            'chassis':{},'operations':{'timeout':{'action':'mission_stop','result':'unknown',
                            'message':'no fresh standstill proof'}}})
            self.assertTrue(window.buttons['mission_stop'].isEnabled())
            self.assertIn('结果未知',window.operation.text())
            self.assertTrue(window.panels['map'].view.map_image.isNull())
        finally:
            window.close();window.worker.stop()

    def test_health_summary_and_stale_authorization_are_explicit(self):
        from robot_console.ui import ConsoleWindow
        from robot_console.demo import FakeClient
        demo=FakeClient()
        window=ConsoleWindow(demo,demo=True)
        try:
            window.worker.quit_event.set();window.worker.wait(1500)
            snapshot=demo.snapshot()
            snapshot['safety']['authorized']=True
            snapshot['mission']['state']='NAVIGATE'
            window.receive(snapshot)
            self.assertIn('已允许',window.authorization.text())
            for name in ('SLAM 定位','Nav2','安全节点'):
                self.assertIn(name,window.health_line.text())
            self.assertIn('ROS 更新时间',window.health_line.toolTip())
            self.assertIn('can0',window.can.text())
            self.assertIn('ERROR-ACTIVE',window.can.text())
            snapshot['controls']['arm']=True
            snapshot['controls']['mission_start']=True
            snapshot['sensors']['safety']['state']='stale'
            snapshot['sensors']['mission']['age']=1.
            window.receive(snapshot)
            self.assertIn('未知',window.authorization.text())
            self.assertIn('过期',window.state.text())
            self.assertNotIn('导航靠近',window.state.text())
            self.assertFalse(window.buttons['arm'].isEnabled())
            self.assertFalse(window.buttons['mission_start'].isEnabled())
        finally:
            window.close();window.worker.stop()


if __name__=='__main__':unittest.main()
