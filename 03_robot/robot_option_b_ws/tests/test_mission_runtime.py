"""Production mission callbacks under runtime guard leases; no ROS or motion."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import unittest
from robot_mission.core import Mission, fresh


class MissionRuntimeAuthorizationTests(unittest.TestCase):
    def setUp(self):
        self.clock=10.
        source=Path(__file__).resolve().parents[1]/'src/robot_mission/robot_mission/node.py'
        cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='MissionNode')
        names={'decode','safety_cb','runtime_authorized','start_cb','stop_cb','publish_status'}
        methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in names]
        context=dict(time=NS(monotonic=lambda:self.clock),json=json,math=math,fresh=fresh,String=lambda **kw:NS(**kw))
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),context)
        self.context=context
        self.node=NS(task=Mission(),now=lambda:self.clock,capable=True,enabled=False,
                     authorization={},authorization_rx=-math.inf,authority_boot='',authority_revision=-1,
                     retired_authority_boots=set(),safety={},safety_rx=-math.inf,detector={},targets={},plans={},
                     nav_pending=False,nav_handle=None,cancel_pending=False,
                     lifecycle_ready=lambda now:True,cancel_navigation=Mock(),publish_status=Mock(),
                     last_state=None,pose=None,map_version=0,active_plan=None,state_pub=NS(publish=Mock()),
                     get_parameter=lambda n:NS(value='trash can'),get_logger=lambda:NS(info=Mock()),log=NS(put=Mock()))
        for name in ('decode','safety_cb','runtime_authorized','start_cb','stop_cb'):
            setattr(self.node,name,MethodType(context[name],self.node))

    def status(self,**changes):
        data=dict(schema=1,stamp=self.clock,task_generation=self.node.task.generation,
                  geometry_epoch=self.node.task.geometry_epoch,authorized=True,ready=True,
                  runtime_control_capable=True,authority_session='guard-boot:1:console')
        data.update(changes)
        self.node.safety_cb(NS(data=json.dumps(data)))
        return data

    def start(self):
        return self.node.start_cb(None,NS())

    def test_boot_and_missing_guard_permission_cannot_start(self):
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.start().success)
        self.assertFalse(self.node.task.active)

    def test_valid_guard_grant_does_not_autostart_and_manual_start_stays_init(self):
        self.status()
        self.assertTrue(self.node.enabled)
        self.assertFalse(self.node.task.active)
        old=self.node.task.generation
        self.assertTrue(self.start().success)
        self.assertNotEqual(self.node.task.generation,old)
        self.assertEqual((self.node.task.state,self.node.task.source),('INIT','none'))
        self.assertEqual(self.node.safety,{})
        self.assertTrue(self.node.runtime_authorized(self.clock,self.clock))

    def test_start_requires_current_generation_and_geometry_readiness(self):
        self.status(task_generation='old-mission')
        self.assertFalse(self.start().success)
        self.clock += .01
        self.status(geometry_epoch=3)
        self.assertFalse(self.start().success)
        self.clock += .01
        self.status()
        self.assertTrue(self.start().success)

    def test_old_task_health_cannot_satisfy_new_task_readiness(self):
        prior=self.status()
        self.assertTrue(self.start().success)
        self.clock += .01
        self.status(task_generation=prior['task_generation'])
        self.assertEqual(self.node.safety,{})
        self.assertEqual(self.node.task.state,'INIT')

    def test_receipt_and_ros_stamp_freshness_are_both_required(self):
        self.status();self.clock += .31
        self.assertFalse(self.start().success)
        self.node.authorization_rx=self.clock
        self.assertFalse(self.start().success)
        self.node.authorization['stamp']=self.clock
        self.node.authorization_rx=self.clock-.31
        self.assertFalse(self.start().success)

    def test_false_or_unready_or_inactive_nav_blocks_start(self):
        self.status(authorized=False)
        self.assertFalse(self.start().success)
        self.clock += .01;self.status(ready=False)
        self.assertFalse(self.start().success)
        self.clock += .01;self.status()
        self.node.lifecycle_ready=lambda now:False
        self.assertFalse(self.start().success)

    def test_nonboolean_authorized_cannot_grant_permission(self):
        self.status(authorized='true')
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.start().success)
        self.node.safety_cb(NS(data='[]'))
        self.assertFalse(self.start().success)

    def test_preview_only_mission_cannot_start_even_with_guard_grant(self):
        self.node.capable=False
        self.status()
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.start().success)

    def test_disarm_revokes_active_task_and_regrant_never_restarts_it(self):
        self.status();self.start()
        self.node.task.transition('SEARCH_CONTINUOUS')
        self.clock += .01
        self.status(authorized=False,authority_session='guard-boot:2:console')
        self.assertEqual((self.node.task.state,self.node.task.source),('HOLD','none'))
        self.assertFalse(self.node.task.active)
        self.node.cancel_navigation.assert_called()
        self.clock += .01
        self.status(authority_session='guard-boot:3:console')
        self.assertTrue(self.node.enabled)
        self.assertFalse(self.node.task.active)
        self.assertEqual(self.node.task.state,'HOLD')

    def test_guard_restart_and_old_grant_replay_do_not_reauthorize(self):
        self.status();self.start()
        self.clock += .01
        self.status(authorized=False,authority_session='guard-new:0:')
        self.assertFalse(self.node.enabled)
        self.clock += .01
        self.status(authority_session='guard-boot:10:console')
        self.assertFalse(self.node.enabled)
        self.assertEqual(self.node.authority_boot,'guard-new')

    def test_lower_authority_revision_cannot_restore_permission(self):
        self.status(authorized=False,authority_session='guard-boot:2:console')
        self.clock += .01
        self.status(authority_session='guard-boot:1:console')
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.start().success)

    def test_status_exposes_active_without_starting_task(self):
        self.context['publish_status'](self.node)
        envelope=json.loads(self.node.state_pub.publish.call_args.args[0].data)
        self.assertIs(envelope['active'],False)
        self.status();self.start()
        self.context['publish_status'](self.node)
        envelope=json.loads(self.node.state_pub.publish.call_args.args[0].data)
        self.assertIs(envelope['active'],True)
        self.assertEqual(envelope['source'],'none')

    def test_status_exposes_only_actual_pending_context_ids(self):
        self.context['publish_status'](self.node)
        envelope=json.loads(self.node.state_pub.publish.call_args.args[0].data)
        self.assertIsNone(envelope['request_id'])
        self.assertIsNone(envelope['observation_id'])
        self.node.task.pending={'request_id':'actual-request','observation_id':'actual-frame',
                                'candidates':{'candidate':'track-7'}}
        self.node.task.target='track-7'
        self.context['publish_status'](self.node)
        envelope=json.loads(self.node.state_pub.publish.call_args.args[0].data)
        self.assertEqual(envelope['request_id'],'actual-request')
        self.assertEqual(envelope['observation_id'],'actual-frame')
        self.assertEqual(envelope['target_track_id'],'track-7')
        self.node.task.hold('stop')
        self.context['publish_status'](self.node)
        envelope=json.loads(self.node.state_pub.publish.call_args.args[0].data)
        self.assertIs(envelope['active'],False)
        self.assertIsNone(envelope['request_id'])
        self.assertIsNone(envelope['observation_id'])


if __name__=='__main__':unittest.main()
