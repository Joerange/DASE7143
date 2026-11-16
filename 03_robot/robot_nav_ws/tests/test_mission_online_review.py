"""Independent adapter regressions: execute production callbacks without ROS."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from robot_mission.core import Mission, Goal, Grid, finite, fresh
from robot_mission.mapping import CachedPlan, classify_update, revalidate_cache, plan_valid


def message(width=200, height=200, origin=(-5.,-5.), blocked=False):
    data=[0]*(width*height)
    if blocked:
        i=math.floor((.5-origin[0])/.05)
        j=math.floor((0.-origin[1])/.05)
        data[j*width+i]=100
    return NS(header=NS(frame_id='map',stamp=NS(sec=10,nanosec=0)),
              info=NS(width=width,height=height,resolution=.05,
                      map_load_time=NS(sec=0,nanosec=0),
                      origin=NS(position=NS(x=origin[0],y=origin[1]),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
              data=data)


class OnlineAdapterReviewTests(unittest.TestCase):
    def setUp(self):
        file=Path(__file__).resolve().parents[1]/'src/robot_mission/robot_mission/node.py'
        cls=next(x for x in ast.parse(file.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='MissionNode')
        methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in ('map_cb','revoke_geometry','nav_accepted')]
        self.context=dict(math=math,time=NS(monotonic=lambda:10.),finite=finite,fresh=fresh,
                          yaw_of=lambda q:0.,Grid=Grid,classify_update=classify_update,
                          revalidate_cache=revalidate_cache,plan_valid=plan_valid)
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(file),'exec'),self.context)
        task=Mission();task.start(1.)
        task.confirmed={'bin':2.};task.confirmed_at=2.;task.target='bin'
        goal=Goal('bin',1.,0.,0.,(1.96,0.,.5),9.9)
        task.nav_goal=goal;task.transition('NAVIGATE')
        plan=CachedPlan(goal,1.,((0.,0.),(.5,0.),(1.,0.)),1,0)
        self.actions=[]
        self.node=NS(task=task,now=lambda:10.,map_stamp=9.,map_rx=9.,map_load_time=0.,
                     map_fingerprint=None,map_version=1,grid=Grid(160,160,.05,(-4.,-4.,0.),[0]*25600),
                     plans={'bin':plan},active_plan=plan,pose=(0.,0.,0.),plan_backoff={},
                     nav_alignment=(0.,0.,0.),targets={'bin':{}},plan_token=1,
                     plan_handle=None,plan_job=None,nav_pending=False,nav_handle=None,nav_token=1)
        self.node.cancel_navigation=lambda:self.actions.append(('cancel',self.node.task.source))
        self.node.publish_status=lambda:self.actions.append(('status',self.node.task.source))
        self.node.revoke_geometry=lambda reason,reconfirm=False:self.context['revoke_geometry'](self.node,reason,reconfirm)

    def test_actual_map_callback_preserves_navigation_during_normal_expansion(self):
        self.context['map_cb'](self.node,message())
        self.assertEqual((self.node.task.state,self.node.task.source),('NAVIGATE','nav'))
        self.assertFalse(self.actions)
        self.assertEqual(self.node.plans['bin'].map_revision,2)

    def test_actual_map_callback_revokes_before_requesting_cancel_on_blocked_path(self):
        self.context['map_cb'](self.node,message(blocked=True))
        self.assertEqual((self.node.task.state,self.node.task.source),('REOBSERVE_RECOVERY','none'))
        self.assertEqual(self.actions[0],('cancel','none'))
        self.assertFalse(self.node.plans)
        self.assertIsNone(self.node.active_plan)
        self.assertFalse(self.node.task.reconfirm_required)

    def test_map_coordinate_reset_retires_semantic_and_path_context(self):
        old=self.node.task.generation
        msg=message();msg.info.resolution=.1
        self.context['map_cb'](self.node,msg)
        self.assertNotEqual(self.node.task.generation,old)
        self.assertEqual(self.node.task.geometry_epoch,1)
        self.assertTrue(self.node.task.reconfirm_required)
        self.assertEqual(self.actions[0],('cancel','none'))

    def test_late_nav_acceptance_after_reset_is_canceled_without_reauthorizing(self):
        old=self.node.task.generation
        self.node.nav_pending=True
        self.node.revoke_geometry('SLAM reset',reconfirm=True)
        handle=NS(accepted=True,get_result_async=lambda:NS(add_done_callback=lambda cb:None))
        self.context['nav_accepted'](self.node,NS(result=lambda:handle),1,old)
        self.assertFalse(self.node.nav_pending)
        self.assertIs(self.node.nav_handle,handle)  # retained until terminal cancel
        self.assertEqual(self.node.task.source,'none')
        self.assertEqual(self.actions[-1],('cancel','none'))


if __name__=='__main__':unittest.main()
