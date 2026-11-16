"""Ordinary-user ROS control supervisor and Unix-only desktop data service.

UI rendering is a different process. No cmd_vel publisher exists in this module.
Only the root CAN helper can change link state, and only this installed service
PID is accepted by that helper. Read-only startup never starts drivers or tasks.
"""
import argparse
import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import socketserver
import struct
import subprocess
import threading
import time
import uuid
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from bunker_msgs.msg import BunkerStatus
from tf2_ros import Buffer, TransformListener
from .core import Feed, Journal, Operation, fresh
from .can_feedback import PassiveCAN


def stamp(s):return s.sec+s.nanosec/1e9

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def safe_json(value):
    if isinstance(value,dict):return {k:safe_json(v) for k,v in value.items() if not k.startswith('_')}
    if isinstance(value,(list,tuple)):return [safe_json(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def helper_call(op,**fields):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(3.);client.connect('/run/robot-console/can.sock')
        client.sendall((json.dumps(dict(op=op,**fields))+'\n').encode())
        raw=b''
        while b'\n' not in raw and len(raw)<65536:
            part=client.recv(4096)
            if not part:break
            raw+=part
        answer=json.loads(raw)
        if not answer.get('ok'):raise RuntimeError(answer.get('error','CAN helper rejected operation'))
        return answer


class Backend(Node):
    def __init__(self,args):
        super().__init__('robot_console_control')
        self.control_enabled=args.control_enabled
        self.control_group=MutuallyExclusiveCallbackGroup()
        self.interface='can0';self.session=uuid.uuid4().hex
        self.lock=threading.RLock();self.request_queue=queue.Queue(maxsize=64)
        self.pool=ThreadPoolExecutor(max_workers=2,thread_name_prefix='control_io')
        self.journal=Journal();self.operations={};self.current=None
        self.values={};self.feeds={};self.frames={};self.latest_images={};self.rgb_cache=deque(maxlen=30);self.rgb_candidates={}
        self.map=None;self.scan={};self.path=[];self.pose=None
        self.can={'interface':'can0','state':'UNKNOWN','bitrate':None};self.can_polled=-math.inf;self.can_future=None;self.helper_available=False
        self.driver=None;self.driver_log=None;self.driver_external=False
        self.passive=PassiveCAN('can0')
        self.stopping=threading.Event();self.image_worker=threading.Thread(target=self.images,daemon=True);self.image_worker.start()
        self.export_dir=Path(args.export_dir).expanduser();self.export_dir.mkdir(parents=True,exist_ok=True)
        self.heartbeat_pub=self.create_publisher(String,'/console/control_heartbeat',1)
        self.chassis_pub=self.create_publisher(String,'/chassis/health',1)
        self.arm_client=self.create_client(SetBool,'/safety/set_motion_enabled',callback_group=self.control_group)
        self.start_client=self.create_client(Trigger,'/mission/start',callback_group=self.control_group)
        self.stop_client=self.create_client(Trigger,'/mission/stop',callback_group=self.control_group)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.topics={'rgb':'/camera/camera/color/image_raw','depth':'/camera/camera/aligned_depth_to_color/image_raw',
                     'lidar':'/scan','odom':'/odom','safety':'/safety/status','map':'/map',
                     'bunker':'/bunker_status','mission':'/mission/control_state','candidates':'/perception/candidates'}
        for kind in ('rgb','depth'):
            self.create_subscription(Image,self.topics[kind],lambda m,k=kind:self.image_cb(k,m),qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data,callback_group=self.control_group)
        self.create_subscription(BunkerStatus,'/bunker_status',self.bunker_cb,1,callback_group=self.control_group)
        self.create_subscription(NavPath,'/plan',self.path_cb,1)
        mq=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,mq)
        for kind,topic in [('mission','/mission/control_state'),('mission_telemetry','/mission/state'),('safety','/safety/status'),('candidates','/perception/candidates'),
                           ('targets','/target/estimates'),('event','/perception/candidate_event'),('confirmation','/perception/confirmation'),
                           ('lidar_health','/perception/lidar_health')]:
            self.create_subscription(String,topic,lambda m,k=kind:self.json_cb(k,m),10 if kind in ('event','confirmation') else 1,
                callback_group=self.control_group if kind in ('mission','safety') else None)
        self.last_events={};self.last_graph=0.;self.chassis={}
        self.create_timer(.05,self.tick,callback_group=self.control_group)
        self.create_timer(.1,self.monitor_graph)
        self.journal.add('system','Control service started disarmed',result='monitor' if not self.control_enabled else 'control capable')

    def now(self):return self.get_clock().now().nanoseconds/1e9
    def record_feed(self,kind,t):
        self.feeds.setdefault(kind,Feed()).update(t,time.monotonic())
    def put(self,kind,value,t):
        value=dict(value,_received=time.monotonic(),_age=self.now()-t,stamp=t)
        with self.lock:self.values[kind]=value;self.record_feed(kind,t)
    def json_cb(self,kind,msg):
        try:
            value=json.loads(msg.data)
            if value.get('schema')!=1:return
            t=float(value['stamp'])
            if not math.isfinite(t):return
            # Never retain raw candidate images/request payloads in telemetry/log.
            value={k:v for k,v in value.items() if k not in ('rgb_jpeg','crops','camera_info','base_from_camera','map_from_camera')}
            self.put(kind,value,t)
            if kind in ('mission','safety','confirmation','event','candidates','targets'):
                key=(value.get('task_generation'),value.get('geometry_epoch'),value.get('state'),value.get('reason'),value.get('request_id'),
                     value.get('authorization_reason'),value.get('waiting_for'),value.get('validated_paths'),
                     value.get('target_revision'),value.get('refinement_status'),
                     tuple(sorted((k,v.get('reason')) for k,v in value.get('planning_status',{}).items())),
                     tuple((c.get('track_id'),c.get('status')) for c in value.get('candidates',[])),
                     tuple((c.get('track_id'),c.get('valid'),c.get('reason')) for c in value.get('targets',[])))
                if key!=self.last_events.get(kind):
                    self.last_events[kind]=key
                    fields={k:value[k] for k in ('task_generation','geometry_epoch','observation_id','request_id','candidates','elapsed','error','state','valid','target_track_id','observed_target_track_id','target_revision','refinement_status','map_revision','validated_paths','waiting_for','planning_status') if k in value}
                    label={'candidates':'YOLO detections','event':'Candidate batch requested','confirmation':'Cloud confirmation',
                           'mission':value.get('reason','Mission decision')+'; '+value.get('waiting_for','')+
                           ('; '+', '.join(dict.fromkeys(v.get('reason','') for v in value.get('planning_status',{}).values())) if value.get('planning_status') else '')+
                           ('; target #%s %s' % (value['target_revision'],value.get('refinement_status','')) if value.get('navigation_mode')=='adaptive_map' else ''),
                           'targets':'Depth localization: '+', '.join(c.get('track_id','')+' '+('valid' if c.get('valid') else c.get('reason','waiting')) for c in value.get('targets',[])),
                           'safety':value.get('reason','Safety status')+'; '+value.get('authorization_reason','')}[kind]
                    with self.lock:self.journal.add(kind,label,**fields)
        except (ValueError,TypeError,KeyError):return
    def image_cb(self,kind,msg):
        with self.lock:
            self.latest_images[kind]=msg;self.record_feed(kind,stamp(msg.header.stamp))
            if kind=='rgb':self.rgb_cache.append(msg)
    def images(self):
        while not self.stopping.wait(.1):
            with self.lock:
                items=dict(self.latest_images);self.latest_images.clear()
                candidates=self.values.get('candidates',{})
                matched=next((m for m in reversed(self.rgb_cache) if abs(stamp(m.header.stamp)-candidates.get('stamp',-1.))<1e-5),None)
                if matched is not None and self.now()-stamp(matched.header.stamp)<.8:items['rgb']=matched
            for kind,msg in items.items():
                try:
                    if kind=='rgb':
                        if msg.encoding not in ('rgb8','bgr8'):continue
                        image=np.ndarray((msg.height,msg.width,3),np.uint8,buffer=msg.data,strides=(msg.step,3,1)).copy()
                        if msg.encoding=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                    else:
                        if msg.encoding not in ('16UC1','32FC1'):continue
                        dt=('>' if msg.is_bigendian else '<')+('u2' if msg.encoding=='16UC1' else 'f4')
                        depth=np.ndarray((msg.height,msg.width),np.dtype(dt),buffer=msg.data,strides=(msg.step,np.dtype(dt).itemsize)).astype(float)
                        if msg.encoding=='16UC1':depth*=.001
                        image=cv2.applyColorMap((np.clip((depth-.2)/3.8,0,1)*255).astype('uint8'),cv2.COLORMAP_TURBO)
                        image[~np.isfinite(depth)|(depth<=0)]=0
                    ok,buf=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])
                    if ok:
                        with self.lock:
                            self.frames[kind]={'jpeg':base64.b64encode(buf).decode(),'stamp':stamp(msg.header.stamp),'width':msg.width,'height':msg.height}
                            if kind=='rgb':
                                self.rgb_candidates=copy.deepcopy(candidates) if abs(stamp(msg.header.stamp)-candidates.get('stamp',-1.))<1e-5 else {}
                except (ValueError,cv2.error,BufferError):pass
    def scan_cb(self,msg):
        value={'stamp':stamp(msg.header.stamp),'ranges':[float(v) if math.isfinite(v) else None for v in msg.ranges],
               'angle_min':msg.angle_min,'angle_increment':msg.angle_increment,'frame':msg.header.frame_id}
        with self.lock:
            self.scan=value
            self.record_feed('lidar',stamp(msg.header.stamp))
    def map_cb(self,msg):
        o=msg.info.origin
        value={'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,
               'origin':[o.position.x,o.position.y,yaw(o.orientation)],'data':list(msg.data),'stamp':stamp(msg.header.stamp)}
        with self.lock:
            self.map=value
            self.record_feed('map',stamp(msg.header.stamp))
    def path_cb(self,msg):
        with self.lock:self.path=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
    def odom_cb(self,msg):
        self.put('odom',{'linear_x':msg.twist.twist.linear.x,'angular_z':msg.twist.twist.angular.z},stamp(msg.header.stamp))
    def bunker_cb(self,msg):
        self.put('bunker',{'battery_voltage':msg.battery_voltage,'vehicle_state':msg.vehicle_state,
                         'control_mode':msg.control_mode,'error_code':msg.error_code},stamp(msg.header.stamp))
    def can_read(self):
        raw=subprocess.run(['/usr/sbin/ip','-details','-json','link','show','dev','can0'],capture_output=True,text=True,timeout=2.)
        if raw.returncode:return {'interface':'can0','state':'UNKNOWN','reason':'interface readback failed'}
        d=json.loads(raw.stdout)[0];info=d.get('linkinfo',{}).get('info_data',{})
        return {'interface':'can0','state':'UP' if 'UP' in d.get('flags',[]) else 'DOWN',
                'bus_state':info.get('state'),'bitrate':info.get('bittiming',{}).get('bitrate'),'read_at':time.monotonic()}
    def call_ros(self,client,request):
        if not client.service_is_ready():raise RuntimeError('ROS control service unavailable')
        return client.call_async(request)
    def request(self,data):
        if data.get('op')=='status':
            # Small control-only IPC response: no images, map or event history.
            with self.lock:
                now=time.monotonic();ros_now=self.now()
                return safe_json(dict(ok=True,stamp=ros_now,monitor_only=not self.control_enabled,
                    session_id=self.session,safety=self.values.get('safety',{}),
                    mission=self.values.get('mission',{}),chassis=self.chassis,can=self.can,
                    sensors={k:self.feeds.get(k,Feed()).status(now,ros_now,.8) for k in ('safety','mission')},
                    operations={k:o.view() for k,o in self.operations.items()}))
        if data.get('op')=='snapshot':return self.snapshot()
        if data.get('op')!='action' or data.get('action') not in ('can_enable','can_disable','driver_start','driver_stop','arm','disarm','mission_start','mission_stop','export'):
            return {'ok':False,'error':'unknown operation'}
        op=Operation(data['action'],time.monotonic())
        with self.lock:self.operations[op.id]=op
        try:self.request_queue.put_nowait(op)
        except queue.Full:op.finish('failed','control request queue full')
        return {'ok':True,'operation_id':op.id}
    def operation_event(self,op):
        self.journal.add('operation',op.message,operation_id=op.id,action=op.action,result=op.result,status=op.phase,
                         task_generation=self.values.get('mission',{}).get('task_generation'))
    def begin(self,op):
        self.operation_event(op)
        if op.action=='export':
            dest=self.export_dir/('events_'+time.strftime('%Y%m%d_%H%M%S')+'_'+op.id[:6]+'.json')
            path=self.journal.export(dest,self.values.get('mission',{}).get('task_generation'))
            op.finish('success',path);return
        if op.action in ('mission_stop','disarm','can_disable','driver_stop'):
            if op.action in ('can_disable','driver_stop') and not self.control_enabled:
                op.finish('failed','monitor-only backend; controlled service installation required');return
            # Immediate independent disarm; no waiting for camera, map or network.
            req=SetBool.Request();req.data=False
            for name,client,request in [('disarm',self.arm_client,req),('stop',self.stop_client,Trigger.Request())]:
                try:op.futures[name]=self.call_ros(client,request)
                except Exception as exc:op.futures[name]=None
            op.phase='stopping';op.message='disarm/stop requested; waiting for real standstill'
            return
        if not self.control_enabled:
            op.finish('failed','monitor-only backend; controlled service installation required');return
        if op.action=='can_enable':
            op.futures['io']=self.pool.submit(helper_call,'enable');op.phase='io'
        elif op.action=='driver_start':
            s=self.values.get('safety',{})
            if (self.can.get('state')!='UP' or time.monotonic()-self.can.get('read_at',-math.inf)>2.
                    or not fresh(s,time.monotonic(),.8) or s.get('authorized',True) or not s.get('output_zero',False)
                    or s.get('output_topic')!='/cmd_vel' or not s.get('runtime_control_capable',False)):
                op.finish('failed','CAN UP and fresh disarmed final-output guard are required');return
            if self.driver_external or self.driver and self.driver.poll() is None:
                op.finish('failed','driver already exists; no duplicate driver started');return
            exe=Path(get_package_prefix('bunker_base'))/'lib/bunker_base/bunker_base_node'
            self.driver_log=(self.export_dir/'supervised_driver.log').open('a')
            self.driver=subprocess.Popen([str(exe),'--ros-args','-r','__node:=bunker_base','-p','port_name:=can0','-p','is_bunker_mini:=true',
                 '-p','simulated_robot:=false'],stdout=self.driver_log,stderr=subprocess.STDOUT,start_new_session=True)
            op.phase='driver_starting';op.deadline=time.monotonic()+15.;op.message='driver starting; waiting for live CAN feedback and ROS state'
        elif op.action=='arm':
            op.before_authority=self.values.get('safety',{}).get('authority_session')
            req=SetBool.Request();req.data=True;op.futures['ros']=self.call_ros(self.arm_client,req);op.phase='arm_ack'
        elif op.action=='mission_start':
            if not self.chassis.get('communication_ok') or not self.values.get('safety',{}).get('authorized'):
                op.finish('failed','actual guard authorization and chassis communication required');return
            op.futures['ros']=self.call_ros(self.start_client,Trigger.Request());op.phase='mission_ack'
    def advance(self,op,now):
        if now>op.deadline:
            op.finish('unknown','operation timed out; link unchanged unless independently read back; query actual state')
            return
        if op.phase=='stopping':
            done,reason=op.evidence.check(self.values.get('safety'),self.values.get('mission'),self.values.get('odom'),self.passive.snapshot(),now,
                                         require_can=True)
            op.message=reason
            if not done:return
            if op.action in ('mission_stop','disarm'):
                op.finish('success','stopped: guard disarmed, navigation idle, fresh odometry and live CAN feedback stable');return
            if self.driver_external or self.driver is None:
                op.finish('failed','standstill confirmed but chassis driver is not owned by controlled service; CAN unchanged');return
            op.proof_at=now
            if op.action=='can_disable':
                op.futures['io']=self.pool.submit(helper_call,'prepare_down');op.phase='prepare_down'
            else:
                self.driver.send_signal(signal.SIGINT);op.phase='driver_stopping'
        elif op.phase=='prepare_down' and op.futures['io'].done():
            answer=op.futures['io'].result();op.token=answer['token']
            valid,reason=op.evidence.check(self.values.get('safety'),self.values.get('mission'),self.values.get('odom'),self.passive.snapshot(),now,require_can=True)
            if not valid:
                op.finish('unknown','stop proof changed before driver shutdown: '+reason+'; CAN unchanged');return
            self.driver.send_signal(signal.SIGINT);op.phase='driver_stopping'
        elif op.phase=='driver_stopping':
            if self.driver.poll() is None:return
            feedback=self.passive.snapshot()
            safety=self.values.get('safety',{});mission=self.values.get('mission',{})
            interlock=(fresh(safety,now,.8) and not safety.get('authorized',True) and safety.get('output_zero',False)
                       and fresh(mission,now,.8) and not mission.get('active',True) and mission.get('source')=='none'
                       and mission.get('navigation_idle',False))
            if now-op.proof_at>3. or not fresh(feedback,now) or not feedback.get('stopped') or not interlock:
                op.finish('unknown','driver exited but live standstill proof expired; CAN remains UP');return
            if op.action=='driver_stop':op.finish('success','owned driver exited after verified stop; CAN remains UP');return
            op.futures['io']=self.pool.submit(helper_call,'commit_down',token=op.token);op.phase='io'
        elif op.phase=='io' and op.futures['io'].done():
            answer=op.futures['io'].result();observed=answer.get('can',{})
            if observed.get('interface')!='can0' or type(observed.get('admin_up')) is not bool:
                op.finish('unknown','helper returned invalid CAN readback');return
            self.can=dict(interface='can0',state='UP' if observed['admin_up'] else 'DOWN',
                          bitrate=observed.get('bitrate'),bus_state=observed.get('bus_state'),read_at=now)
            wanted='DOWN' if op.action=='can_disable' else 'UP'
            op.finish('success' if self.can['state']==wanted else 'unknown','CAN actual readback '+self.can['state'])
        elif op.phase=='driver_starting':
            if self.driver.poll() is not None:op.finish('failed','driver exited before communication ready')
            elif self.chassis.get('communication_ok'):op.finish('success','driver and live chassis communication ready; motion remains disabled')
        elif op.phase in ('arm_ack','mission_ack') and op.futures['ros'].done():
            result=op.futures['ros'].result()
            if not result.success:op.finish('failed',result.message);return
            op.phase='arm_readback' if op.action=='arm' else 'mission_readback';op.message='service accepted; waiting for state readback'
        elif op.phase=='arm_readback':
            if fresh(self.values.get('safety'),now,.8) and self.values['safety'].get('authorized'):
                op.finish('success','independent guard confirms motion permission; task still requires Start')
            elif (fresh(self.values.get('safety'),now,.8) and
                  self.values['safety'].get('authority_session')!=getattr(op,'before_authority',None)):
                op.finish('failed','permission revoked: '+self.values['safety'].get('authorization_reason','unknown'))
        elif op.phase=='mission_readback':
            m=self.values.get('mission',{})
            if fresh(m,now,.8) and m.get('state') not in ('DONE','HOLD') and m.get('motion_enabled'):
                op.finish('success','mission accepted; actual state '+m.get('state','unknown'))
    def update_chassis(self,now,ros_now):
        fb=self.passive.snapshot();odom=self.values.get('odom',{});bunker=self.values.get('bunker',{})
        driver_running=self.driver_external or bool(self.driver and self.driver.poll() is None)
        can_up=self.can.get('state')=='UP' and now-self.can.get('read_at',-math.inf)<2.
        finite_velocity=all(isinstance(odom.get(k),(float,int)) and math.isfinite(odom[k]) for k in ('linear_x','angular_z'))
        conflict=self.driver_external and bool(self.driver and self.driver.poll() is None)
        communication=bool(can_up and driver_running and not conflict and fb['live'] and fresh(bunker,now,.6) and fresh(odom,now,.6) and finite_velocity)
        velocity={'linear_x':odom.get('linear_x'),'angular_z':odom.get('angular_z'),'age':now-odom.get('_received',now)+odom.get('_age',0.), 'valid':communication and fresh(odom,now,.6)}
        self.chassis=safe_json(dict(schema=1,stamp=ros_now,session_id=self.session,can_interface='can0',can_up=can_up,
                          driver_running=driver_running,communication_ok=communication,velocity=velocity,
                          can_feedback=safe_json(fb),battery_voltage=bunker.get('battery_voltage'),
                          reason='' if communication else 'CAN UP alone is not chassis communication; require fresh feedback + one driver + finite odom'))

    def monitor_graph(self):
        # Discovery/TF and image callbacks use a different executor group from
        # leases, measured odometry, stop requests and service acknowledgments.
        now=time.monotonic()
        if now-self.last_graph>1.:
            self.last_graph=now
            driver_pids=[]
            for entry in Path('/proc').iterdir():
                if not entry.name.isdigit():continue
                try:
                    args=(entry/'cmdline').read_bytes().split(b'\0')
                    if any(Path(x.decode(errors='replace')).name=='bunker_base_node' for x in args[:2]):
                        driver_pids.append(int(entry.name))
                except (OSError,ValueError):pass
            own_pid=self.driver.pid if self.driver is not None else None
            publishers={kind:self.count_publishers(topic) for kind,topic in self.topics.items()}
            with self.lock:
                self.driver_external=any(pid!=own_pid for pid in driver_pids)
                for kind,count in publishers.items():self.feeds.setdefault(kind,Feed()).publishers=count
        try:
            tr=self.tf.lookup_transform('map','base_link',Time())
            pose=[tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)]
            with self.lock:
                self.pose=pose;self.record_feed('tf',stamp(tr.header.stamp))
        except Exception:
            with self.lock:self.pose=None

    def tick(self):
        now=time.monotonic();ros_now=self.now()
        with self.lock:
            if self.can_future and self.can_future.done():
                try:self.can=self.can_future.result()
                except Exception:self.can={'interface':'can0','state':'UNKNOWN','reason':'CAN readback unavailable'}
                self.can_future=None
            if now-self.can_polled>1. and self.can_future is None:
                self.can_polled=now;self.can_future=self.pool.submit(self.can_read)
            self.update_chassis(now,ros_now)
            if self.control_enabled:
                self.heartbeat_pub.publish(String(data=json.dumps(dict(schema=1,stamp=ros_now,session_id=self.session))))
                self.chassis_pub.publish(String(data=json.dumps(self.chassis,allow_nan=False)))
            # Stop/disarm requests preempt any UI action. They never wait behind image encoding or a cloud call.
            waiting=[]
            while not self.request_queue.empty():waiting.append(self.request_queue.get_nowait())
            priority=[o for o in waiting if o.action in ('mission_stop','disarm','can_disable','driver_stop')]
            if priority and self.current:
                self.current.finish('unknown','superseded by explicit stop/disarm');self.operation_event(self.current);self.current=None
            ordered=priority+[o for o in waiting if o not in priority]
            for op in ordered:
                if self.current:
                    op.finish('failed','another control operation is in progress');self.operation_event(op);continue
                try:self.begin(op)
                except Exception as exc:op.finish('failed',str(exc)[:240])
                if op.result=='pending':self.current=op
                else:self.operation_event(op)
            if self.current:
                try:self.advance(self.current,now)
                except Exception as exc:self.current.finish('failed',str(exc)[:240])
                if self.current.result!='pending':self.operation_event(self.current);self.current=None
            while len(self.operations)>100:self.operations.pop(next(iter(self.operations)))
    def snapshot(self):
        with self.lock:
            now=time.monotonic();ros_now=self.now()
            limits={'rgb':.6,'depth':.6,'lidar':1.,'odom':.6,'safety':.8,'mission':.8,'map':10.,'tf':.8,'bunker':.6,'candidates':1.5}
            safety=self.values.get('safety',{})
            scan_limit=safety.get('freshness_limits',{}).get('scan_timeout')
            if fresh(safety,now,.8) and isinstance(scan_limit,(int,float)) and math.isfinite(scan_limit) and scan_limit>0:
                limits['lidar']=scan_limit
            sensors={key:self.feeds.get(key,Feed()).status(now,ros_now,limit) for key,limit in limits.items()}
            mission=dict(self.values.get('mission_telemetry',{}),**self.values.get('mission',{}));safety=self.values.get('safety',{})
            for key,source,valid in [('nav2','mission',mission.get('nav_ready')),('localization','safety',safety.get('ready'))]:
                feed=sensors[source]
                sensors[key]=dict(feed,state=('normal' if valid else 'waiting') if feed['state']=='normal' else feed['state'])
            controls={key:self.control_enabled for key in ('can_enable','can_disable','driver_start','driver_stop','arm','mission_start')}
            controls.update(mission_stop=True,disarm=True,export=True)
            controls['arm']=self.control_enabled and fresh(safety,now,.8) and safety.get('ready',False) and self.chassis.get('communication_ok',False) and mission.get('nav_ready',False) and not safety.get('authorized',False)
            controls['mission_start']=self.control_enabled and fresh(safety,now,.8) and safety.get('authorized',False) and safety.get('ready',False) and fresh(mission,now,.8) and mission.get('nav_ready',False) and not mission.get('active',False) and mission.get('navigation_idle',False)
            can=dict(self.can)
            if now-can.get('read_at',-math.inf)>2.:can.update(state='UNKNOWN',reason='CAN link readback stale')
            payload={'ok':True,'stamp':ros_now,'connected':True,'monitor_only':not self.control_enabled,'session_id':self.session,
                'frames':dict(self.frames),'scan':self.scan,'map':self.map,'path':mission.get('path',self.path),'pose':mission.get('pose') or self.pose,
                'mission':mission,'safety':safety,'chassis':self.chassis,'can':can,'sensors':sensors,
                'lidar_health':self.values.get('lidar_health',{}),
                'candidates':self.rgb_candidates,'latest_detections':self.values.get('candidates',{}),'targets':self.values.get('targets',{}),
                'events':list(self.journal.events)[-300:],'operations':{k:o.view() for k,o in self.operations.items()},'controls':controls}
        # All large objects are immutable snapshots. Never serialize a map while
        # holding the lock needed by the control heartbeat or odometry callback.
        return safe_json(payload)


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads=True
    def __init__(self,path,node):
        self.node=node;super().__init__(path,Handler)
class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(2.)
        if hasattr(socket,'SO_PEERCRED'):
            pid,uid,gid=struct.unpack('3i',self.request.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
            if uid!=os.getuid():return
        raw=self.rfile.readline(8193)
        if len(raw)>8192:return
        try:result=self.server.node.request(json.loads(raw))
        except Exception:result={'ok':False,'error':'invalid request'}
        self.wfile.write((json.dumps(result,allow_nan=False)+'\n').encode())


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--control-enabled',action='store_true')
    parser.add_argument('--socket',default='/run/user/'+str(os.getuid())+'/robot-console.sock')
    parser.add_argument('--export-dir',default='~/robot_nav_ws/console_exports')
    args=parser.parse_args()
    path=Path(args.socket);path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        try:
            client=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);client.settimeout(.5);client.connect(str(path));client.close()
            raise RuntimeError('console backend already running')
        except (ConnectionRefusedError,FileNotFoundError):path.unlink(missing_ok=True)
    rclpy.init();node=Backend(args);server=Server(str(path),node);os.chmod(path,0o600)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        # Service loss expires guard authority. Do not cut CAN or kill a possibly
        # moving chassis driver as an unverified substitute for stopping.
        executor.shutdown()
        try:
            with node.lock:
                node.journal.export(node.export_dir/('events_session_'+time.strftime('%Y%m%d_%H%M%S')+'_'+node.session[:8]+'.json'))
        except OSError as error:
            node.get_logger().error('Event archive failed: '+type(error).__name__)
        node.stopping.set();node.passive.stop.set();server.shutdown();server.server_close();path.unlink(missing_ok=True)
        node.pool.shutdown(wait=False,cancel_futures=True);node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
