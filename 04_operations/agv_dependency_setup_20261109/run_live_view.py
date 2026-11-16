#!/usr/bin/env python3
"""Own sensor/UI processes; closing the dashboard shuts down this sensor session."""
import os,sys,subprocess,signal,json,time
from pathlib import Path
root=Path(__file__).resolve().parent
os.environ.update(ROS_DOMAIN_ID='174',ROS_LOCALHOST_ONLY='1',DISPLAY=':1',
 XAUTHORITY='/run/user/1000/gdm/Xauthority',DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/1000/bus')
children=[]
def start(name,command):
 log=(root/('live_'+name+'.log')).open('w')
 p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 children.append((name,p,log));return p
def signal_stop(signum,frame):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,signal_stop)
try:
 can=json.loads(subprocess.check_output(['ip','-j','link','show','type','can'],text=True))
 if any('UP' in c.get('flags',[]) for c in can):raise RuntimeError('CAN unexpectedly enabled')
 from ament_index_python.packages import get_package_share_directory
 cfg=Path(get_package_share_directory('lslidar_driver'))/'config/lslidar_cx.yaml'
 start('camera',['ros2','launch','realsense2_camera','rs_launch.py','enable_color:=true','enable_depth:=true',
  'align_depth.enable:=true','enable_sync:=true','rgb_camera.color_profile:=640x480x30',
  'depth_module.depth_profile:=640x480x30','rgb_camera.power_line_frequency:=1'])
 start('lidar',['ros2','run','lslidar_driver','lslidar_driver_node','--ros-args','-r','__ns:=/cx',
  '-r','__node:=lslidar_driver_node','--params-file',str(cfg)])
 ui=start('dashboard',[str(Path.home()/'yolo-world-agv/.venv/bin/python'),str(root/'live_sensor_view.py')])
 (root/'live_session.json').write_text(json.dumps({'supervisor_pid':os.getpid(),'started':time.strftime('%Y-%m-%d %H:%M:%S'),
  'children':{n:p.pid for n,p,l in children},'ros_domain_id':174,'display':':1','motion_control':False},indent=2))
 ui.wait()
finally:
 for name,p,log in children:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for name,p,log in children:
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
  log.close()
