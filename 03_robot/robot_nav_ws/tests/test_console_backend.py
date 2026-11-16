"""Production supervisor methods with fake ROS/OS boundaries, no CAN writes."""
import ast
from concurrent.futures import Future
import json
import math
from pathlib import Path
import queue
import struct
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from robot_console.core import Feed, Journal, Operation, StopEvidence, fresh
from robot_console.can_feedback import Feedback

SOURCE=Path(__file__).resolve().parents[1]/'src/robot_console/robot_console/backend.py'


def supervisor():
    module=ast.parse(SOURCE.read_text())
    selected=[n for n in module.body if isinstance(n,ast.FunctionDef) and n.name=='safe_json' or
              isinstance(n,ast.ClassDef) and n.name=='Backend']
    context=dict(Node=object, time=NS(monotonic=lambda:10.), math=math, json=json,
                 SetBool=NS(Request=lambda:NS(data=False)),Trigger=NS(Request=lambda:NS()),
                 fresh=fresh,Operation=Operation,Feed=Feed,queue=queue,copy=__import__('copy'),
                 signal=__import__('signal'),helper_call=Mock())
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(SOURCE),'exec'),context)
    backend=object.__new__(context['Backend'])
    backend.control_enabled=True;backend.can={'state':'UP'};backend.values={};backend.chassis={}
    backend.passive=NS(snapshot=lambda:{});backend.driver=None;backend.driver_external=False
    backend.pool=NS(submit=Mock());backend.arm_client=Mock();backend.stop_client=Mock();backend.start_client=Mock()
    backend.call_ros=Mock();backend.operation_event=Mock()
    return backend


class ConsoleEvidenceTests(unittest.TestCase):
    def test_acquisition_age_continues_to_age_after_receipt(self):
        self.assertTrue(fresh({'_received':10.,'_age':.2},10.05))
        self.assertFalse(fresh({'_received':10.,'_age':.2},10.2))
        self.assertFalse(fresh({'_received':10.,'_age':-.1},10.))

    def test_sensor_waiting_normal_stale_disconnected(self):
        f=Feed();self.assertEqual(f.status(10,10,.3)['state'],'waiting')
        f.update(10,10);f.update(10.1,10.1)
        self.assertAlmostEqual(f.status(10.1,10.1,.3)['hz'],10.)
        self.assertEqual(f.status(10.1,10.1,.3)['state'],'normal')
        self.assertEqual(f.status(11,11,.3)['state'],'stale')
        f.publishers=0;self.assertEqual(f.status(11,11,.3)['state'],'disconnected')

    def test_real_can_feedback_needs_both_frame_types_and_expires(self):
        f=Feedback();f.ingest(0x211,bytes(8),10.)
        self.assertFalse(f.snapshot(10.1)['live'])
        f.ingest(0x221,struct.pack('<hhhh',100,-20,0,0),10.1)
        self.assertEqual(f.snapshot(10.1)['velocity'],[.1,-.02])
        self.assertFalse(f.snapshot(10.1)['stopped'])
        self.assertFalse(f.snapshot(10.5)['live'])

    def test_stop_evidence_rejects_cached_nan_or_active_mission(self):
        e=StopEvidence()
        s=dict(_received=10.,authorized=False,output_zero=True)
        m=dict(_received=10.,source='none',active=False,navigation_idle=True)
        o=dict(_received=10.,linear_x=0.,angular_z=0.)
        self.assertFalse(e.check(s,m,o,{},10.,True)[0])
        o['linear_x']=float('nan');self.assertIn('odometry',e.check(s,m,o,{},10.)[1])
        o['linear_x']=0.;m['active']=True
        self.assertIn('cancellation',e.check(s,m,o,{},10.)[1])
        m['active']=False
        for t in (10.,10.4,10.8):
            for v in (s,m,o):v['_received']=t
            result=e.check(s,m,o,{},t)
        self.assertTrue(result[0])

    def test_export_is_generation_scoped_and_redacted(self):
        j=Journal();j.add('confirmation','result',task_generation='A',request_id='r',observation_id='o',
                         candidates=[{'id':1,'status':'match','api_key':'secret'}],api_key='secret',rgb_jpeg='image')
        j.add('mission','old',task_generation='B')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'events.json';j.export(p,'A');raw=p.read_text()
        self.assertNotIn('secret',raw);self.assertNotIn('"old"',raw)
        self.assertEqual(json.loads(raw)['events'][0]['request_id'],'r')


class ConsoleOperationsTests(unittest.TestCase):
    def test_can_up_without_chassis_cannot_start_task(self):
        b=supervisor();b.values={'safety':{'authorized':True}};b.chassis={'communication_ok':False}
        op=Operation('mission_start',10.);b.begin(op)
        self.assertEqual(op.result,'failed');b.call_ros.assert_not_called()

    def test_malformed_fresh_odom_fails_closed_without_json_crash(self):
        b=supervisor();b.session='current';b.driver_external=True
        b.can=dict(state='UP',read_at=10.)
        b.passive=NS(snapshot=lambda:dict(live=True,stopped=True))
        b.values={'bunker':dict(_received=10.),'odom':dict(_received=10.,linear_x=float('nan'),angular_z=0.)}
        b.update_chassis(10.,100.)
        self.assertFalse(b.chassis['communication_ok']);self.assertFalse(b.chassis['velocity']['valid'])
        self.assertIsNone(b.chassis['velocity']['linear_x']);json.dumps(b.chassis,allow_nan=False)

    def test_cached_down_can_still_requests_disarm_and_stop(self):
        b=supervisor();b.can={'state':'DOWN','read_at':0.};op=Operation('can_disable',10.)
        b.begin(op)
        self.assertEqual(b.call_ros.call_count,2);self.assertEqual(op.result,'pending')
        self.assertEqual(op.phase,'stopping')

    def test_real_guard_denial_is_reported_not_fake_toggle(self):
        b=supervisor();op=Operation('arm',10.);f=Future();f.set_result(NS(success=False,message='sensor_health:scan_stale'))
        b.call_ros.return_value=f;b.begin(op);b.advance(op,10.1)
        self.assertEqual(op.result,'failed');self.assertIn('scan_stale',op.message)

    def test_arm_success_requires_actual_fresh_state_readback(self):
        b=supervisor();op=Operation('arm',10.);f=Future();f.set_result(NS(success=True,message='accepted'))
        b.call_ros.return_value=f;b.begin(op);b.advance(op,10.01)
        self.assertEqual(op.result,'pending')
        b.values={'safety':dict(_received=9.,authorized=True)};b.advance(op,10.1)
        self.assertEqual(op.result,'pending')
        b.values={'safety':dict(_received=10.1,authorized=True)};b.advance(op,10.1)
        self.assertEqual(op.result,'success')

    def test_can_disable_without_verified_stop_keeps_interface(self):
        b=supervisor();op=Operation('can_disable',10.);b.begin(op)
        b.advance(op,10.1);self.assertEqual(op.phase,'stopping')
        b.advance(op,op.deadline+.01)
        self.assertEqual(op.result,'unknown');self.assertEqual(b.can['state'],'UP')
        b.pool.submit.assert_not_called();self.assertEqual(b.call_ros.call_count,2)

    def test_stop_timeout_remains_unknown_and_requests_both_services(self):
        b=supervisor();b.call_ros.side_effect=RuntimeError('service absent')
        op=Operation('mission_stop',10.);b.begin(op);b.advance(op,op.deadline+.01)
        self.assertEqual(op.result,'unknown');self.assertEqual(b.call_ros.call_count,2)

    def test_can_down_proof_change_during_helper_reply_aborts_shutdown(self):
        b=supervisor();b.driver=Mock();op=Operation('can_disable',10.)
        op.phase='prepare_down';f=Future();f.set_result({'token':'opaque'});op.futures['io']=f
        b.values={'safety':dict(_received=10.,authorized=True,output_zero=False)}
        b.advance(op,10.1)
        self.assertEqual(op.result,'unknown');b.driver.send_signal.assert_not_called();b.pool.submit.assert_not_called()

    def test_can_down_rechecks_guard_after_driver_exits(self):
        b=supervisor();b.driver=NS(poll=lambda:0);op=Operation('can_disable',10.)
        op.phase='driver_stopping';op.proof_at=10.;op.token='opaque'
        b.passive=NS(snapshot=lambda:dict(_received=10.1,stopped=True))
        b.values={'safety':dict(_received=10.1,authorized=True,output_zero=False),
                  'mission':dict(_received=10.1,active=True,source='nav',navigation_idle=False)}
        b.advance(op,10.1)
        self.assertEqual(op.result,'unknown');b.pool.submit.assert_not_called()

    def test_reconnect_snapshot_uses_actual_state_not_last_success(self):
        b=supervisor();b.now=lambda:100.;b.lock=threading.RLock();b.feeds={};b.session='new'
        b.frames={};b.scan={};b.map=None;b.path=[];b.pose=None;b.rgb_candidates={};b.journal=Journal()
        previous=Operation('arm',9.);previous.finish('success','old grant');b.operations={previous.id:previous}
        b.values={'safety':dict(_received=10.,stamp=100.,authorized=False,ready=True),
                  'mission':dict(_received=10.,stamp=100.,active=False,nav_ready=True,navigation_idle=True)}
        b.chassis={'communication_ok':True}
        snapshot=b.snapshot()
        self.assertFalse(snapshot['safety']['authorized']);self.assertFalse(snapshot['controls']['mission_start'])
        self.assertTrue(snapshot['controls']['arm'])

    def test_ui_backend_contains_no_velocity_publisher_or_arbitrary_shell(self):
        module=ast.parse(SOURCE.read_text())
        calls=[n for n in ast.walk(module) if isinstance(n,ast.Call)]
        publishers=[n for n in calls if isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher']
        self.assertEqual(len(publishers),2)
        self.assertTrue(all(isinstance(n.args[0],ast.Name) and n.args[0].id=='String' for n in publishers))
        self.assertFalse(any(k.arg=='shell' and isinstance(k.value,ast.Constant) and k.value.value for n in calls for k in n.keywords))


if __name__=='__main__':unittest.main()
