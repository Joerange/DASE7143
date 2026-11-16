#!/usr/bin/env python3
"""Live sensor + disarmed task acceptance. NO chassis, map, odom or movement."""
import os
os.environ['ROS_DOMAIN_ID'] = '176'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

out = Path.home()/'robot_main_task_backup_20261109'/'stationary'
out.mkdir(parents=True, exist_ok=True)
if (out/'report.json').exists():
    (out/('report_previous_'+str(int(time.time()))+'.json')).write_bytes((out/'report.json').read_bytes())
report = {'scope':'Live sensors, disarmed task. No map/odom/chassis/nav controllers; no API candidates or motion.',
          'domain':176, 'counts':{}, 'samples':{}}

def can():
    return json.loads(subprocess.check_output(['ip','-j','link','show','type','can'], text=True))

report['can_before'] = can()
if any('UP' in x.get('flags', []) for x in report['can_before']):
    raise RuntimeError('CAN is already up; test will not alter it')
log = (out/'launch.log').open('w')
p = subprocess.Popen(['ros2','launch','robot_bringup','task.launch.py','start_sensors:=true',
                      'motion_enabled:=false','enable_navigation:=false','autostart:=false'],
                     stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init()
node = rclpy.create_node('stationary_main_task_audit')
preview_nonzero=[]

def receive(name, msg):
    report['counts'][name] = report['counts'].get(name,0)+1
    if name=='scan':
        import math
        valid=[x for x in msg.ranges if math.isfinite(x)]
        report['samples'][name]={'frame':msg.header.frame_id,'rays':len(msg.ranges),'finite':len(valid),
                                 'min':min(valid) if valid else None}
    elif name=='preview':
        if msg.linear.x != 0 or msg.angular.z != 0:
            preview_nonzero.append([msg.linear.x,msg.angular.z])
    else:
        value=json.loads(msg.data)
        if name=='candidates':
            value.pop('rgb_jpeg',None)
        report['samples'][name]=value

for name, topic in [('candidates','/perception/candidates'),('estimates','/target/estimates'),
                    ('mission','/mission/state'),('safety','/safety/status'),
                    ('events','/perception/candidate_event'),('confirmation','/perception/confirmation')]:
    node.create_subscription(String,topic,lambda msg,n=name:receive(n,msg),10)
node.create_subscription(LaserScan,'/scan',lambda m:receive('scan',m),qos_profile_sensor_data)
node.create_subscription(Twist,'/cmd_vel_guard_preview',lambda m:receive('preview',m),10)
try:
    start=time.monotonic()
    valid_since=None
    while time.monotonic()-start < 60:
        rclpy.spin_once(node,timeout_sec=.05)
        if p.poll() is not None:
            raise RuntimeError('launch terminated early')
        ready=report['samples'].get('candidates',{}).get('ready',False)
        if ready and report['counts'].get('scan',0)>10:
            valid_since = valid_since or time.monotonic()
            if time.monotonic()-valid_since>12:
                break
    client=node.create_client(Trigger,'/mission/start')
    if not client.wait_for_service(timeout_sec=2):
        raise RuntimeError('mission start service missing')
    future=client.call_async(Trigger.Request())
    until=time.monotonic()+3
    while not future.done() and time.monotonic()<until:
        rclpy.spin_once(node,timeout_sec=.05)
    report['disarmed_start_rejected']=future.done() and not future.result().success
    report['cmd_vel_publishers']=len(node.get_publishers_info_by_topic('/cmd_vel'))
    report['preview_nonzero']=preview_nonzero
    report['elapsed']=time.monotonic()-start
    report['checks']={
        'rgbd_yolo_live':report['samples'].get('candidates',{}).get('ready',False),
        'scan_live':report['counts'].get('scan',0)>10,
        'independent_depth_guard_live':report['samples'].get('safety',{}).get('sensor_health',{}).get('depth',False),
        'zero_api_events':report['counts'].get('events',0)==0 and report['counts'].get('confirmation',0)==0,
        'zero_final_publishers':report['cmd_vel_publishers']==0,
        'disarmed_start_rejected':report['disarmed_start_rejected'],
        'preview_zero':not preview_nonzero,
        'missing_odom_blocks_safety':not report['samples'].get('safety',{}).get('ready',True)}
    report['passed']=all(report['checks'].values())
except Exception as exc:
    report['error']=str(exc)
    report['passed']=False
finally:
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        p.wait(timeout=5)
    log.close()
    report['can_after']=can()
    report['can_remained_down']=not any('UP' in x.get('flags',[]) for x in report['can_after'])
    report['passed']=report.get('passed',False) and report['can_remained_down']
    (out/'report.json').write_text(json.dumps(report,indent=2))
    node.destroy_node()
    rclpy.shutdown()
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
