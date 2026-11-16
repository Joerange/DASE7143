#!/usr/bin/env python3
"""Jetson desktop integration: demo or camera monitor; never chassis or motion."""
import argparse
import ctypes as C
import json
import os
from pathlib import Path
import re
import subprocess
import time

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data

os.environ['ROS_DOMAIN_ID']='182'
os.environ['ROS_LOCALHOST_ONLY']='1'
workspace=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--monitor',action='store_true',help='Use the real camera and YOLO, with motion disabled')
parser.add_argument('--capture',action='store_true',help='Save one camera frame for detection diagnosis')
parser.add_argument('--expect-target',action='store_true',help='Require trash-bin detections in the real scene')
parser.add_argument('--probe-control',action='store_true',help='Replay the scene in pure software, without a command publisher')
options=parser.parse_args();monitor=options.monitor
if (options.capture or options.expect_target or options.probe_control) and not monitor:parser.error('capture/expect-target/probe-control require --monitor')
log_path=workspace/('validation/jetson/one_click_monitor.log' if monitor else 'validation/jetson/one_click_demo.log')
log_path.parent.mkdir(parents=True,exist_ok=True)

# Send events only to the demo window we find by its exact title.
x=C.CDLL('libX11.so.6')
x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=C.c_void_p
x.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int];x.XInternAtom.restype=C.c_ulong
x.XStringToKeysym.argtypes=[C.c_char_p];x.XStringToKeysym.restype=C.c_ulong
x.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong];x.XKeysymToKeycode.restype=C.c_uint
x.XDefaultRootWindow.argtypes=[C.c_void_p];x.XDefaultRootWindow.restype=C.c_ulong
x.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.c_void_p]
x.XFlush.argtypes=[C.c_void_p];x.XCloseDisplay.argtypes=[C.c_void_p]
display=x.XOpenDisplay(None)
assert display,'Run in the Jetson graphical desktop'

class Key(C.Structure):
    _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),
              ('window',C.c_ulong),('root',C.c_ulong),('subwindow',C.c_ulong),('time',C.c_ulong),
              ('x',C.c_int),('y',C.c_int),('x_root',C.c_int),('y_root',C.c_int),
              ('state',C.c_uint),('keycode',C.c_uint),('same_screen',C.c_int)]

class Client(C.Structure):
    _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),
              ('window',C.c_ulong),('message_type',C.c_ulong),('format',C.c_int),('data',C.c_long*5)]

def key(window,symbol):
    code=x.XKeysymToKeycode(display,x.XStringToKeysym(symbol))
    for kind,mask in ((2,1),(3,2)):
        event=Key(type=kind,display=display,window=window,root=x.XDefaultRootWindow(display),
                  keycode=code,same_screen=1)
        buffer=C.create_string_buffer(192);C.memmove(buffer,C.byref(event),C.sizeof(event))
        x.XSendEvent(display,window,True,mask,buffer)
    x.XFlush(display)

def close_window(window):
    event=Client(type=33,display=display,window=window,
                 message_type=x.XInternAtom(display,b'WM_PROTOCOLS',False),format=32)
    event.data[0]=x.XInternAtom(display,b'WM_DELETE_WINDOW',False)
    buffer=C.create_string_buffer(192);C.memmove(buffer,C.byref(event),C.sizeof(event))
    x.XSendEvent(display,window,False,0,buffer);x.XFlush(display)

rclpy.init();audit=rclpy.create_node('rgbd_window_smoke')
statuses=[];commands=[];observations=[];rgb_frames=[];last_rgb=[None]
last_depth=[None];last_info=[None];control_probe=None
audit.create_subscription(String,'/rgbd/safety/status',lambda m:statuses.append(json.loads(m.data)),10)
audit.create_subscription(Twist,'/rgbd/cmd_vel_preview',lambda m:commands.append((m.linear.x,m.angular.z)),10)
audit.create_subscription(String,'/rgbd/observation',lambda m:observations.append(json.loads(m.data)),10)
def rgb_cb(msg):
    rgb_frames.append((msg.width,msg.height));last_rgb[0]=msg
audit.create_subscription(Image,'/camera/camera/color/image_raw',rgb_cb,qos_profile_sensor_data)
if options.probe_control:
    from tf2_ros import Buffer,TransformListener
    from rclpy.time import Time
    from bunker_rgbd.ros_utils import corridor,depth_array,matrix
    tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,audit)
    audit.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',lambda m:last_depth.__setitem__(0,m),qos_profile_sensor_data)
    audit.create_subscription(CameraInfo,'/camera/camera/aligned_depth_to_color/camera_info',lambda m:last_info.__setitem__(0,m),qos_profile_sensor_data)

def replay_scene(stream=None):
    # No ROS services or publishers: only the production pure-Python control cores.
    from bunker_rgbd.core import Mission,Safety
    depth,info=last_depth[0],last_info[0]
    base=matrix(tf_buffer.lookup_transform('base_link',depth.header.frame_id,Time.from_msg(depth.header.stamp)))
    corridor_data=corridor(depth_array(depth),depth.encoding,info.k,base,info.d,info.distortion_model)
    task=Mission();guard=Safety(True);command={};outputs=[];trace=[]
    state={}
    failures={}
    frames=stream if stream else [None]*101
    for step,source in enumerate(frames):
        t=100.+(source['stamp']-frames[0]['stamp'] if source else step*.02)
        state=dict(active=task.active,state=task.state,generation=task.generation,target=task.target,goal=task.goal)
        inputs={key:dict(value,stamp=t,_rx=t) for key,value in dict(
            rgb={},depth=corridor_data,observation=observations[-1],mission=state,
            odom=dict(x=0.,y=0.,yaw=0.,v=0.,w=0.)).items()}
        inputs['feedback']=dict(live=True,vehicle_state=0,control_mode=1,error_code=0,velocity=[0.,0.])
        if source:
            for key in ('rgb','depth','observation'):
                age=source['input_ages'][key]
                inputs[key].update(stamp=t-age['stamp'],_rx=t-age['received'])
            inputs['depth']['valid']=source['depth_valid']
        healthy,why=guard.health(t,t,inputs)
        if not healthy:failures[why]=failures.get(why,0)+1
        if step==0:
            granted,grant_reason=guard.grant(True,t,t,inputs)
            if granted:task.start(t)
        (v,w),ready,reason=guard.tick(t,t,inputs,command)
        outputs.append([v,w])
        request=task.step(t,t,inputs['observation'],dict(stamp=t,_rx=t,ready=ready,authorized=guard.authorized,reason=reason),inputs['odom'])
        command=dict(stamp=t,_rx=t,generation=task.generation,state=task.state,v=request[0],w=request[1])
        event=[task.state,task.reason,reason]
        if not trace or event!=trace[-1]:trace.append(event)
    return dict(kind='SOFTWARE ONLY: frozen real scene, synthetic stationary odometry/feedback; no commands published',
                granted=granted,grant_reason=grant_reason,depth_valid=corridor_data['valid'],coverage=corridor_data['coverage'],
                close_points=[p for p in corridor_data['points'] if -.425<p[0]<.47 and abs(p[1])<.365][:20],
                max_forward=max(v for v,w in outputs),max_yaw=max(abs(w) for v,w in outputs),trace=trace,
                actual_sensor_fault_counts=failures,real_sensor_timing=bool(stream),
                yolo_over_old_300ms=sum(s['input_ages']['observation']['stamp']>.3 for s in (stream or [])),
                max_yolo_age=max((s['input_ages']['observation']['stamp'] for s in (stream or [])),default=0.))

def until(predicate,timeout=15.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(audit,timeout_sec=.03)
        if predicate():return
    raise AssertionError('Timed out: '+log_path.read_text()[-3500:])

def find_window():
    tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
    found=re.search(r'(0x[0-9a-f]+) "Bunker Mini"',tree)
    return int(found.group(1),16) if found else None

assert not find_window(),'Close an existing RGB-D window before running this demo test'
if monitor:
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:command=(entry/'cmdline').read_bytes().split(b'\0')[0]
        except (OSError,ProcessLookupError):continue
        assert command.rsplit(b'/',1)[-1]!=b'realsense2_camera_node','An existing camera session is running; leave it untouched'
session=None
try:
    with log_path.open('w') as log:
        session=subprocess.Popen([str(workspace/'scripts/run_task.sh'),'--monitor' if monitor else '--demo'],stdout=log,stderr=subprocess.STDOUT)
        until(lambda:find_window() is not None,30.)
        window=find_window()
        until(lambda:bool(statuses))
        if monitor:
            until(lambda:len(observations)>=10 and any(o.get('ready') for o in observations),90.)
            if options.expect_target:until(lambda:sum(bool(o.get('targets')) for o in observations)>=10,20.)
            if options.probe_control:
                until(lambda:last_depth[0] is not None and last_info[0] is not None)
                begin=len(statuses);end=time.monotonic()+15.
                until(lambda:time.monotonic()>=end,20.)
                stream=[s for s in statuses[begin:] if all(k in s.get('input_ages',{}) for k in ('rgb','depth','observation'))]
                from bunker_rgbd.timing import DATA_TIMEOUT
                for index,s in enumerate(stream):
                    if s['depth_valid'] and all(max(s['input_ages'][k].values())<=DATA_TIMEOUT for k in ('rgb','depth','observation')):
                        stream=stream[index:];break
                control_probe=replay_scene(stream)
                print('CONTROL_PROBE '+json.dumps(control_probe),flush=True)
                (log_path.parent/'control_probe.json').write_text(json.dumps(control_probe,indent=2)+'\n')
            if options.capture:
                from cv_bridge import CvBridge
                import cv2
                cv2.imwrite(str(log_path.parent/'detection_rgb.png'),CvBridge().imgmsg_to_cv2(last_rgb[0],'bgr8'))
        key(window,b's')
        until(lambda:'Permission denied:' in log_path.read_text())
        key(window,b'space')
        until(lambda:'Preview stopped' in log_path.read_text())
        close_window(window)
        until(lambda:session.poll() is not None,20.)
    text=log_path.read_text()
    pids=[int(pid) for pid in re.findall(r'process started with pid \[(\d+)\]',text)]
    result=dict(mode='Real camera + YOLO; no CAN or chassis' if monitor else 'SIMULATED - no sensors, CAN or chassis',start_key_denied=True,stop_key_confirmed=True,
                window_close_exited=session.returncode==0,owned_processes=len(pids),
                clean_process_exits=len(re.findall(r'process has finished cleanly \[pid \d+\]',text)),
                surviving_processes=[pid for pid in pids if Path('/proc/'+str(pid)).exists()],
                safety_messages=len(statuses),authorized=any(s.get('authorized') for s in statuses),
                preview_messages=len(commands),all_zero=bool(commands) and all(v==w==0 for v,w in commands),
                final_publishers=audit.count_publishers('/cmd_vel'),rgb_frames=len(rgb_frames),
                yolo_observations=len(observations),perception_ready=any(o.get('ready') for o in observations),
                target_observations=sum(bool(o.get('targets')) for o in observations),
                valid_depth_observations=sum(any(t.get('valid') for t in o.get('targets',[])) for o in observations),
                detected_labels=sorted({t.get('label','') for o in observations for t in o.get('targets',[])}))
    print(json.dumps(result,indent=2))
    log_path.with_suffix('.json').write_text(json.dumps(result,indent=2)+'\n')
    if options.capture:
        (log_path.parent/'detection_observations.json').write_text(json.dumps(observations[-30:],indent=2)+'\n')
    assert result['window_close_exited'] and len(pids)>=5 and not result['surviving_processes']
    assert result['clean_process_exits']==len(pids),text[-4000:]
    assert result['all_zero'] and not result['authorized'] and result['final_publishers']==0
    assert 'bunker_base_node' not in text
    if options.probe_control:
        assert control_probe['granted'] and max(control_probe['max_forward'],control_probe['max_yaw'])>0
        assert not any(t[0]=='HOLD' for t in control_probe['trace']),control_probe
    if monitor:assert result['perception_ready'] and rgb_frames
    else:assert 'realsense2_camera_node' not in text
finally:
    if session is not None and session.poll() is None:
        session.send_signal(2)
        try:session.wait(timeout=15)
        except subprocess.TimeoutExpired:session.kill();session.wait()
    audit.destroy_node();rclpy.try_shutdown();x.XCloseDisplay(display)
