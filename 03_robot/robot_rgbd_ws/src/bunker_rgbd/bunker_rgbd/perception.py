"""RGB-D + YOLO only. Bounded input and output slots; no cloud or map."""
import json
import os
import re
from pathlib import Path
import threading
import time
import uuid
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger,SetBool
from cv_bridge import CvBridge
from message_filters import Subscriber,ApproximateTimeSynchronizer
from tf2_ros import Buffer,TransformListener
from .ros_utils import seconds,matrix,depth_array
from .geometry import depth_surface
from .tracking import Tracker
from .window_control import WindowControl
from .detection import BIN_LABELS,is_target_label
from .timing import OBSERVATION_TIMEOUT,WORKER_EXIT_TIMEOUT

WINDOW_TITLE='Bunker Mini'


def display_text(value):
    """Use neutral detection wording in the operator view, including backend errors."""
    return re.sub(r'yolo[\w.-]*','Detection',str(value),flags=re.IGNORECASE)


class Perception(Node):
    def __init__(self):
        super().__init__('rgbd_perception')
        defaults={'model_path':str(Path.home()/'yolo-world-agv/models/yolov8s-worldv2-trash.pt'),
                  'device':'0','confidence':.40,'target_label':'trash can','show_window':True,'demo':False}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.cfg={k:self.get_parameter(k).value for k in defaults}
        self.bridge=CvBridge();self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.lock=threading.Lock();self.latest=None;self.output=None;self.stop_event=threading.Event()
        self.display=None;self.selected='';self.mission_state={};self.window_open=False
        self.show_window=bool(self.cfg['show_window'] and os.getenv('DISPLAY'))
        self.controls=WindowControl(self.send_control)
        self.model_status='SIMULATED' if self.cfg['demo'] else 'Loading detector...'
        self.control_message=''
        self.control_clients={'arm':self.create_client(SetBool,'/rgbd/safety/set_motion_enabled'),
                              'start':self.create_client(Trigger,'/rgbd/mission/start'),
                              'stop':self.create_client(Trigger,'/rgbd/mission/stop')}
        self.last_frame=np.zeros((480,640,3),np.uint8);self.last_packet={'targets':[]};self.render_at=0.
        self.create_subscription(String,'/rgbd/mission/state',self.mission_cb,1)
        self.create_subscription(String,'/rgbd/safety/status',self.safety_cb,1)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        self.tracker=Tracker();self.pub=self.create_publisher(String,'/rgbd/observation',1)
        self.rgb=Subscriber(self,Image,'/camera/camera/color/image_raw',qos_profile=qos_profile_sensor_data)
        self.depth=Subscriber(self,Image,'/camera/camera/aligned_depth_to_color/image_raw',qos_profile=qos_profile_sensor_data)
        self.info=Subscriber(self,CameraInfo,'/camera/camera/aligned_depth_to_color/camera_info',qos_profile=qos_profile_sensor_data)
        self.sync=ApproximateTimeSynchronizer([self.rgb,self.depth,self.info],5,.05)
        self.sync.registerCallback(self.observe)
        self.create_timer(.02,self.publish)
        self.worker=threading.Thread(target=self.demo if self.cfg['demo'] else self.infer,daemon=True);self.worker.start()
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def send_control(self,action,value=None):
        client=self.control_clients[action]
        if not client.service_is_ready():return None
        request=SetBool.Request(data=value) if action=='arm' else Trigger.Request()
        return client.call_async(request)
    def mission_cb(self,msg):
        try:
            self.mission_state=json.loads(msg.data);self.selected=self.mission_state.get('target','')
            self.controls.mission=dict(self.mission_state,_rx=time.monotonic())
        except ValueError:pass
    def safety_cb(self,msg):
        try:self.controls.safety=dict(json.loads(msg.data),_rx=time.monotonic())
        except ValueError:pass
    def odom_cb(self,msg):
        self.controls.odom=dict(stamp=seconds(msg.header.stamp),_rx=time.monotonic(),
                                v=msg.twist.twist.linear.x,w=msg.twist.twist.angular.z)
    def demo(self):
        # Explicit no-hardware display fixture. Never supplies ready perception.
        while not self.stop_event.wait(.1):
            frame=np.zeros((480,640,3),np.uint8)
            cv2.rectangle(frame,(260,150),(390,390),(75,100,90),-1)
            cv2.putText(frame,'SIMULATED - NO CAMERA / CAN / MOTION',(18,100),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,100,255),2)
            packet=dict(schema=1,stamp=self.now(),ready=False,reason='SIMULATED display only',
                        targets=[dict(track_id='demo-bin',bbox=[260,150,390,390],label='trash can',score=.92,valid=True,gap=.8)])
            with self.lock:self.display=(frame,packet)
    def observe(self,rgb,depth,info):
        if (rgb.width,rgb.height)!=(depth.width,depth.height) or (depth.width,depth.height)!=(info.width,info.height):return
        if not rgb.header.frame_id or not rgb.header.frame_id==depth.header.frame_id==info.header.frame_id:return
        if not -.03<=self.now()-seconds(rgb.header.stamp)<=OBSERVATION_TIMEOUT:return
        with self.lock:self.latest=(rgb,depth,info,time.monotonic())
    def infer(self):
        try:
            os.environ.setdefault('YOLO_CONFIG_DIR',str(Path.home()/'yolo-world-agv/.ultralytics'))
            from ultralytics import YOLOWorld
            if not Path(self.cfg['model_path']).is_file():raise RuntimeError('local YOLO weights missing; no automatic download')
            model=YOLOWorld(self.cfg['model_path'])
            self.get_logger().info('YOLO classes: '+str(model.names)+'; accepted bin labels: '+str(BIN_LABELS))
            params=dict(device=self.cfg['device'],imgsz=640,conf=float(self.cfg['confidence']),half=self.cfg['device']!='cpu',
                        agnostic_nms=True,verbose=False)
            model.predict(np.zeros((480,640,3),np.uint8),**params)
            self.model_status='Detection ready'
            while not self.stop_event.wait(.005):
                with self.lock:item=self.latest;self.latest=None
                if item is None:continue
                rgb,depth,info,rx=item;t=seconds(rgb.header.stamp)
                if time.monotonic()-rx>OBSERVATION_TIMEOUT or self.now()-t>OBSERVATION_TIMEOUT:continue
                bgr=self.bridge.imgmsg_to_cv2(rgb,'bgr8');arr=depth_array(depth)
                result=model.predict(bgr,**params)[0]
                # Retry after inference so acquisition-time TF has reached the buffer.
                try:base=matrix(self.tf.lookup_transform('base_link',rgb.header.frame_id,Time.from_msg(rgb.header.stamp)))
                except Exception:base=None
                rows=[]
                for box in result.boxes:
                    label=result.names[int(box.cls.item())]
                    if not is_target_label(label,self.cfg['target_label']):continue
                    bbox=box.xyxy[0].cpu().tolist()
                    geom=depth_surface(arr,depth.encoding,bbox,info.k,info.d,info.distortion_model,base)
                    rows.append(dict(geom,bbox=bbox,score=float(box.conf.item()),label=label))
                if time.monotonic()-rx>OBSERVATION_TIMEOUT or self.now()-t>OBSERVATION_TIMEOUT:continue
                packet=dict(schema=1,stamp=t,depth_stamp=seconds(depth.header.stamp),observation_id=uuid.uuid4().hex,
                            ready=base is not None,reason='' if base is not None else 'camera extrinsics unavailable',
                            targets=self.tracker.update(rows,t),raw_detections=len(result.boxes))
                with self.lock:
                    self.output=packet
                    if self.show_window:self.display=(bgr.copy(),packet)
        except Exception as error:
            self.model_status='Detection error: '+str(error)
            self.get_logger().error('Perception stopped: '+str(error))
            # No fresh output will be published; independent guard expires.
    def publish(self):
        with self.lock:
            value=self.output;self.output=None
            display=self.display;self.display=None
        if value and -.03<=self.now()-value['stamp']<=OBSERVATION_TIMEOUT:self.pub.publish(String(data=json.dumps(value,allow_nan=False)))
        now=time.monotonic();self.controls.tick(now,self.now())
        if self.controls.message!=self.control_message:
            self.control_message=self.controls.message;self.get_logger().info(self.control_message)
        if self.controls.exit_ready:
            self.get_logger().info(self.controls.message+'; closing this launch session')
            raise SystemExit(0)
        if self.show_window:
            if not self.window_open:
                cv2.namedWindow(WINDOW_TITLE,cv2.WINDOW_NORMAL);self.window_open=True
                cv2.imshow(WINDOW_TITLE,self.last_frame)
            # Process window events before drawing; never recreate a closed window.
            key=cv2.waitKey(1)&255
            self.controls.key(key,now)
            try:visible=cv2.getWindowProperty(WINDOW_TITLE,cv2.WND_PROP_VISIBLE)>=1
            except cv2.error:visible=False
            if not visible:
                self.controls.stop(now,closing=True);self.show_window=False
                return
            if display:self.last_frame,self.last_packet=display
            if now>=self.render_at:
                self.render_at=now+.1
                frame=self.render_frame(self.last_frame.copy(),self.last_packet)
                cv2.imshow(WINDOW_TITLE,frame)

    def render_frame(self,frame,packet):
        for target in packet['targets']:
            x1,y1,x2,y2=map(int,target['bbox']);selected=target['track_id']==self.selected
            color=(0,220,255) if selected else (0,220,0)
            cv2.rectangle(frame,(x1,y1),(x2,y2),color,3 if selected else 1)
            label=('LOCKED ' if selected else '')+target['label']+' %.2f '%target['score']+target['track_id'][:6]
            if target.get('valid'):label+=' gap %.2fm'%target['gap']
            cv2.putText(frame,display_text(label),(max(0,x1),max(16,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.42,color,1)
        state=self.mission_state.get('state','MONITOR')
        text=state+' | '+self.mission_state.get('tracking_mode','waiting for RGB-D')
        cv2.putText(frame,display_text(text),(8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,220,255),1)
        reason=display_text(self.mission_state.get('reason','Waiting for task'))
        cv2.putText(frame,reason[:100],(8,44),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        reason=display_text(self.controls.safety.get('reason','Waiting for safety node'))
        cv2.putText(frame,reason[:100],(8,64),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,180,255),1)
        detector=self.model_status+' | bins: '+str(len(packet['targets']))+' | conf >= %.2f'%self.cfg['confidence']
        cv2.putText(frame,display_text(detector)[:100],(8,86),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        message=self.controls.message
        if state=='DONE' and self.controls.stage=='idle':message='Task completed. S: new task | Q: exit'
        cv2.putText(frame,display_text(message)[:100],(8,108),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        if state=='DONE':
            height,width=frame.shape[:2];center=height//2
            cv2.rectangle(frame,(12,center-42),(width-12,center+42),(35,90,40),-1)
            cv2.rectangle(frame,(12,center-42),(width-12,center+42),(80,220,110),2)
            title='TASK COMPLETED'
            (text_width,text_height),_=cv2.getTextSize(title,cv2.FONT_HERSHEY_SIMPLEX,1.,2)
            scale=min(1.,(width-48)/max(1,text_width))
            (text_width,text_height),_=cv2.getTextSize(title,cv2.FONT_HERSHEY_SIMPLEX,scale,2)
            cv2.putText(frame,title,((width-text_width)//2,center+text_height//2),
                        cv2.FONT_HERSHEY_SIMPLEX,scale,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame,'S / Enter: start   SPACE / X: stop   Q / Esc / close: stop + exit',
                    (8,frame.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
        return frame


def main():
    rclpy.init();node=Perception()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.stop_event.set()
        # Let the current Torch/CUDA inference return before Python destroys its
        # native runtime or the ROS node used by the worker.
        node.worker.join(timeout=WORKER_EXIT_TIMEOUT)
        if node.worker.is_alive():
            node.get_logger().error('YOLO worker did not stop within %g seconds; terminating this session'%WORKER_EXIT_TIMEOUT)
            os._exit(1)  # Launch still reaps the other owned processes.
        cv2.destroyAllWindows();node.destroy_node();rclpy.try_shutdown()
