#!/usr/bin/env python3
"""Stationary sensor capture only; no chassis or velocity publishers."""
import os
os.environ['ROS_DOMAIN_ID']='174'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['YOLO_CONFIG_DIR']=str(__import__('pathlib').Path.home()/'yolo-world-agv/.ultralytics')
import json,time,signal,subprocess
from pathlib import Path
from collections import deque
from datetime import datetime,timezone
import numpy as np
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from ament_index_python.packages import get_package_share_directory

root=Path(__file__).resolve().parent
out=root/('front_reference_'+datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir()
report={'started_utc':datetime.now(timezone.utc).isoformat(),'user_reference':'green bin approximately on chassis centreline in front',
        'motion_commands_sent':False,'ros_domain_id':174}
processes=[];rgb=deque(maxlen=45);depth=deque(maxlen=45);clouds=deque(maxlen=10);info={}
rclpy.init();node=rclpy.create_node('front_reference_capture')
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def cbinfo(m):info[m.header.frame_id]={'width':m.width,'height':m.height,'k':list(m.k),'d':list(m.d),'distortion_model':m.distortion_model}
subs=[node.create_subscription(Image,'/camera/camera/color/image_raw',rgb.append,qos_profile_sensor_data),
      node.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',depth.append,qos_profile_sensor_data),
      node.create_subscription(CameraInfo,'/camera/camera/color/camera_info',cbinfo,qos_profile_sensor_data),
      node.create_subscription(PointCloud2,'/cx/lslidar_point_cloud',clouds.append,qos_profile_sensor_data)]
def start(name,cmd):
 log=(out/(name+'.log')).open('w');p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,p,log))
try:
 can=json.loads(subprocess.check_output(['ip','-j','link','show','type','can'],text=True))
 if any('UP' in c.get('flags',[]) for c in can):raise RuntimeError('CAN unexpectedly active')
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
 if len(node.get_node_names_and_namespaces())!=1:raise RuntimeError('Capture domain already in use')
 start('camera',['ros2','launch','realsense2_camera','rs_launch.py','enable_color:=true','enable_depth:=true',
   'align_depth.enable:=true','enable_sync:=true','rgb_camera.color_profile:=640x480x30','depth_module.depth_profile:=640x480x30'])
 cfg=Path(get_package_share_directory('lslidar_driver'))/'config/lslidar_cx.yaml'
 start('lidar',['ros2','run','lslidar_driver','lslidar_driver_node','--ros-args','-r','__ns:=/cx','-r','__node:=lslidar_driver_node','--params-file',str(cfg)])
 end=time.monotonic()+14
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
 if not rgb or not depth or not clouds:raise RuntimeError('Missing sensor stream')
 r=rgb[-3];d=min(depth,key=lambda m:abs(stamp(m)-stamp(r)));c=min(clouds,key=lambda m:abs(stamp(m)-stamp(r)))
 if r.encoding!='rgb8' or d.encoding!='16UC1':raise RuntimeError('Unexpected image encoding')
 a=np.ndarray((r.height,r.width,3),dtype=np.uint8,buffer=r.data,strides=(r.step,3,1)).copy()
 z=np.ndarray((d.height,d.width),dtype='>u2' if d.is_bigendian else '<u2',buffer=d.data,strides=(d.step,2)).copy()
 fields={f.name:f for f in c.fields}
 dtype=np.dtype({'names':['x','y','z'],'formats':['>f4' if c.is_bigendian else '<f4']*3,
    'offsets':[fields[s].offset for s in ['x','y','z']],'itemsize':c.point_step})
 structured=np.ndarray((c.height,c.width),dtype=dtype,buffer=c.data,strides=(c.row_step,c.point_step))
 xyz=np.column_stack([structured[s].ravel() for s in ['x','y','z']])
 xyz=xyz[np.isfinite(xyz).all(axis=1)]
 cv2.imwrite(str(out/'rgb.png'),cv2.cvtColor(a,cv2.COLOR_RGB2BGR))
 np.savez_compressed(out/'sensor_data.npz',depth_raw=z,cloud_xyz=xyz)
 report.update(rgb_stamp=stamp(r),depth_stamp=stamp(d),cloud_stamp=stamp(c),rgb_depth_dt_ms=abs(stamp(r)-stamp(d))*1000,
  rgb_cloud_dt_ms=abs(stamp(r)-stamp(c))*1000,camera_info=info,cloud_frame=c.header.frame_id,cloud_points=len(xyz),
  depth_scale_assumption_m=.001,depth_scale_note='ROS 16UC1 nominal mm; independent distance calibration still pending')
 report['capture_status']='passed'
finally:
 for name,p,log in processes:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for name,p,log in processes:
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
  log.close();report[name+'_exit']=p.returncode
 node.destroy_node();rclpy.shutdown()
 (out/'report.json').write_text(json.dumps(report,indent=2))

# Inference uses the saved RGB after sensors have stopped.
from ultralytics import YOLOWorld
model=YOLOWorld(str(Path.home()/'yolo-world-agv/models/yolov8s-worldv2-trash.pt'))
result=model.predict(str(out/'rgb.png'),device=0,imgsz=640,conf=.15,half=True,agnostic_nms=True,verbose=False)[0]
detections=[]
for b in result.boxes:
 box=b.xyxy[0].tolist();x1,y1,x2,y2=box
 rx1,rx2=int(x1+.2*(x2-x1)),int(x2-.2*(x2-x1));ry1,ry2=int(y1+.2*(y2-y1)),int(y2-.2*(y2-y1))
 roi=z[max(0,ry1):min(z.shape[0],ry2),max(0,rx1):min(z.shape[1],rx2)];good=roi[roi>0]
 detections.append({'class':result.names[int(b.cls.item())],'confidence':float(b.conf.item()),'bbox_xyxy':box,
   'inner_roi_valid_fraction':float(np.mean(roi>0)) if roi.size else 0.,
   'inner_depth_raw_p10_p50_p90':np.percentile(good,[10,50,90]).tolist() if good.size else []})
report['detections']=detections
(out/'report.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'output':str(out),'detections':detections},indent=2))
