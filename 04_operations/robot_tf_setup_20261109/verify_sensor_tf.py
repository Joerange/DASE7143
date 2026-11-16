#!/usr/bin/env python3
"""Stationary mounting-TF and RGB-D consistency check in isolated ROS domain 175."""
import os
os.environ.update(ROS_DOMAIN_ID='175', ROS_LOCALHOST_ONLY='1')
import json, time, signal, subprocess, math
from pathlib import Path
from datetime import datetime, timezone
import numpy as np
import cv2
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo, Image

root=Path(__file__).resolve().parent
report={'started_utc':datetime.now(timezone.utc).isoformat(),'ros_domain_id':175,
        'motion_commands_sent':False,'odometry_faked':False}
processes=[];edges={};parents_seen={};data={}
rclpy.init();node=rclpy.create_node('stationary_sensor_tf_verifier')
buffer=Buffer();listener=TransformListener(buffer,node)

def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)

def record_tf(msg):
 for tf in msg.transforms:
  edges[tf.child_frame_id]=tf.header.frame_id
  parents_seen.setdefault(tf.child_frame_id,set()).add(tf.header.frame_id)

subs=[node.create_subscription(TFMessage,'/tf_static',record_tf,
        QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
 node.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',lambda m:data.update(depth=m),qos_profile_sensor_data),
 node.create_subscription(CameraInfo,'/camera/camera/aligned_depth_to_color/camera_info',lambda m:data.update(info=m),qos_profile_sensor_data)]

def matrix(tf):
 q=tf.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
 R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
 t=tf.transform.translation
 T=np.eye(4);T[:3,:3]=R;T[:3,3]=[t.x,t.y,t.z]
 return T

def start(name,command):
 log=(root/(name+'.log')).open('w')
 p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 processes.append((name,p,log))

try:
 report['can_before']=json.loads(subprocess.check_output(['ip','-j','link','show','type','can'],text=True))
 if any('UP' in i['flags'] for i in report['can_before']):raise RuntimeError('Unexpected active CAN')
 pump(2)
 if len(node.get_node_names_and_namespaces())!=1:raise RuntimeError('Test ROS domain already in use')
 start('mounting_tf',['ros2','launch','robot_bringup','sensor_tf.launch.py'])
 start('camera',['ros2','launch','realsense2_camera','rs_launch.py','enable_color:=true','enable_depth:=true',
       'align_depth.enable:=true','enable_sync:=true','rgb_camera.color_profile:=640x480x30',
       'depth_module.depth_profile:=640x480x30','rgb_camera.power_line_frequency:=1'])
 pump(14)
 report['tf_parents']=edges.copy()
 report['parent_count_per_child']={k:len(v) for k,v in parents_seen.items()}
 report['tf_static_publishers']=[p.node_name for p in node.get_publishers_info_by_topic('/tf_static')]
 expected_publishers={'camera','base_to_lidar_static_tf','base_to_camera_mount_static_tf','camera_mount_to_link_static_tf'}
 if set(report['tf_static_publishers'])!=expected_publishers or len(report['tf_static_publishers'])!=4:
  raise RuntimeError('Unexpected static TF publishers in isolated test domain')
 for child,parent in {'laser_link':'base_link','camera_mount':'base_link','camera_link':'camera_mount'}.items():
  if edges.get(child)!=parent or len(parents_seen.get(child,set()))!=1:raise RuntimeError('Missing or conflicting mounting transform: '+child)
 if 'odom' in edges or 'base_link' in edges:raise RuntimeError('Unexpected synthetic odometry')
 report['base_from_frames']={}
 matrices={}
 for frame in ['laser_link','camera_mount','camera_link','camera_color_optical_frame','camera_depth_optical_frame']:
  T=matrix(buffer.lookup_transform('base_link',frame,Time()));matrices[frame]=T;report['base_from_frames'][frame]=T.tolist()
 Tl=matrices['laser_link'];Tc=matrices['camera_color_optical_frame']
 lidar_forward=(Tl@np.array([0.,-1.07,0.,1.]))[:3]
 if not np.allclose(lidar_forward,[1.195,0,.600],atol=1e-6):raise RuntimeError('LiDAR +90 degree rotation check failed')
 report['lidar_minus_y_reference_in_base']=lidar_forward.tolist()
 direction=Tc[:3,2];report['camera_optical_forward_in_base']=direction.tolist()
 if direction[0]<.95 or direction[2]>-.15:raise RuntimeError('Camera optical direction is not forward/down as configured')
 report['cmd_vel_publishers']=len(node.get_publishers_info_by_topic('/cmd_vel'))
 if report['cmd_vel_publishers']:raise RuntimeError('Unexpected velocity publisher')
 if not {'depth','info'}<=data.keys():raise RuntimeError('No live aligned-depth / CameraInfo')
 msg,ci=data['depth'],data['info']
 if msg.encoding!='16UC1':raise RuntimeError('Unexpected depth encoding')
 z=np.ndarray((msg.height,msg.width),dtype='>u2' if msg.is_bigendian else '<u2',buffer=msg.data,strides=(msg.step,2)).copy()
 K=np.array(ci.k).reshape(3,3);D=np.array(ci.d)
 yy,xx=np.mgrid[360:470:3,45:595:3]
 dep=z[yy,xx].ravel()*.001;uv=np.column_stack([xx.ravel(),yy.ravel()]).astype(float)
 rays=cv2.undistortPoints(uv[:,None,:],K,D)[:,0,:]
 points=np.column_stack([rays*dep[:,None],dep]);valid=(dep>.3)&(dep<3.)
 points=points[valid];base=points@Tc[:3,:3].T+Tc[:3,3]
 report['live_floor_roi_consistency']={'roi_pixels':[45,360,595,470],'valid_points':len(points),
  'base_z_p5_p50_p95_m':np.percentile(base[:,2],[5,50,95]).tolist(),
  'fraction_abs_z_under_3cm':float(np.mean(abs(base[:,2])<.03)),
  'note':'Visible floor ROI check only; not a complete extrinsic accuracy claim.'}
 np.savez_compressed(root/'live_floor_check.npz',depth_raw=z,k=K,d=D,base_from_color_optical=Tc)
 if report['live_floor_roi_consistency']['fraction_abs_z_under_3cm']<.8:
  raise RuntimeError('Floor ROI inconsistent with approximate height/tilt; inspect capture before accepting')
 report['status']='passed'
except Exception as e:
 report['status']='failed';report['error']=type(e).__name__+': '+str(e)
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
 pump(3);report['nodes_after_cleanup']=node.get_node_names_and_namespaces()
 node.destroy_node();rclpy.shutdown()
 report['can_after']=json.loads(subprocess.check_output(['ip','-j','link','show','type','can'],text=True))
 report['completed_utc']=datetime.now(timezone.utc).isoformat()
 (root/'tf_validation_report.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report,indent=2))
 if report.get('status')!='passed':raise SystemExit(1)
