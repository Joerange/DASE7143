"""Independent 50Hz final writer. Does not need LiDAR, map, SLAM, Nav2 or VLM."""
import json
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer,TransformListener
from .core import Safety,finite
from .ros_utils import seconds,matrix,depth_array,corridor
from .can_feedback import PassiveCAN


class GuardNode(Node):
    def __init__(self):
        super().__init__('rgbd_guard');self.declare_parameter('allow_runtime_control',False)
        capable=bool(self.get_parameter('allow_runtime_control').value)
        self.guard=Safety(capable);self.inputs={};self.command={};self.info=None
        self.reported_authorization=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.lock=threading.Lock();self.latest=None;self.stop_event=threading.Event()
        self.passive=PassiveCAN('can0') if capable else None
        self.output_topic='/cmd_vel' if capable else '/rgbd/cmd_vel_preview'
        self.output=self.create_publisher(Twist,self.output_topic,1)
        self.status=self.create_publisher(String,'/rgbd/safety/status',1)
        self.create_subscription(Image,'/camera/camera/color/image_raw',self.rgb_cb,qos_profile_sensor_data)
        self.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',self.depth_cb,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/camera/aligned_depth_to_color/camera_info',self.info_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        for key,topic in [('observation','/rgbd/observation'),('mission','/rgbd/mission/state'),('command','/rgbd/cmd_request')]:
            self.create_subscription(String,topic,lambda msg,k=key:self.json_cb(k,msg),1)
        self.create_service(SetBool,'/rgbd/safety/set_motion_enabled',self.grant_cb)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker=threading.Thread(target=self.process_depth,daemon=True);self.worker.start()
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def rgb_cb(self,m):self.inputs['rgb']=dict(stamp=seconds(m.header.stamp),_rx=time.monotonic())
    def info_cb(self,m):self.info=m
    def odom_cb(self,m):self.inputs['odom']=dict(stamp=seconds(m.header.stamp),_rx=time.monotonic(),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,x=m.pose.pose.position.x,y=m.pose.pose.position.y, yaw=math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.z+m.pose.pose.orientation.x*m.pose.pose.orientation.y),1-2*(m.pose.pose.orientation.y**2+m.pose.pose.orientation.z**2)))
    def depth_cb(self,m):
        with self.lock:self.latest=(m,self.info,time.monotonic())
    def process_depth(self):
        while not self.stop_event.wait(.02):
            with self.lock:item=self.latest;self.latest=None
            if item is None:continue
            m,info,rx=item
            try:
                if info is None or abs(seconds(info.header.stamp)-seconds(m.header.stamp))>.10:raise ValueError('camera info not synchronized')
                if (m.width,m.height)!=(info.width,info.height) or m.header.frame_id!=info.header.frame_id:raise ValueError('invalid alignment')
                transform=matrix(self.tf.lookup_transform('base_link',m.header.frame_id,Time.from_msg(m.header.stamp)))
                value=corridor(depth_array(m),m.encoding,info.k,transform,info.d,info.distortion_model)
            except Exception:value=dict(valid=False,points=[],coverage=0.)
            self.inputs['depth']=dict(value,stamp=seconds(m.header.stamp),_rx=rx)
    def json_cb(self,key,msg):
        try:
            d=json.loads(msg.data)
            if d.get('schema')!=1 or not isinstance(d.get('stamp'),(float,int)):return
            old=self.command if key=='command' else self.inputs.get(key,{})
            if d['stamp']<=old.get('stamp',-1):return
            value=dict(d,_rx=time.monotonic())
            if key=='command':self.command=value
            else:self.inputs[key]=value
        except (ValueError,TypeError):pass
    def snapshot(self):
        result=dict(self.inputs);result['feedback']=self.passive.snapshot() if self.passive else {}
        return result
    def grant_cb(self,req,res):
        res.success,res.message=self.guard.grant(req.data,time.monotonic(),self.now(),self.snapshot())
        if not self.guard.authorized:self.output.publish(Twist())
        return res
    def tick(self):
        now=time.monotonic();ros_now=self.now()
        snapshot=self.snapshot()
        try:
            if self.guard.capable and self.count_publishers('/cmd_vel')>1:
                self.guard.revoke('another final velocity publisher exists')
            (v,w),ready,reason=self.guard.tick(now,ros_now,snapshot,self.command)
        except Exception as error:
            self.guard.revoke('internal guard error: '+type(error).__name__);v=w=0.;ready=False;reason=self.guard.reason
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.output.publish(msg)
        authorization=(self.guard.authorized,self.guard.reason)
        if authorization!=self.reported_authorization:
            self.get_logger().info('Motion '+('authorized: ' if self.guard.authorized else 'disabled: ')+self.guard.reason)
            self.reported_authorization=authorization
        self.status.publish(String(data=json.dumps(dict(schema=1,stamp=ros_now,ready=ready,
            authorized=self.guard.authorized,reason=reason,output_zero=(v==0 and w==0),output_topic=self.output_topic,
            feedback_live=bool(snapshot['feedback'].get('live')),feedback_stopped=bool(snapshot['feedback'].get('stopped')),
            input_ages={key:dict(stamp=ros_now-value['stamp'],received=now-value['_rx'])
                        for key,value in snapshot.items() if finite(value.get('stamp'),value.get('_rx'))},
            depth_valid=bool(snapshot.get('depth',{}).get('valid')),depth_coverage=snapshot.get('depth',{}).get('coverage',0.),
            velocity_request=[self.command.get('v',0.),self.command.get('w',0.)],velocity_output=[v,w],
            velocity_feedback=snapshot['feedback'].get('velocity'),
            velocity_odometry=[snapshot.get('odom',{}).get('v'),snapshot.get('odom',{}).get('w')],
            sensor_mode='rgbd_only',coverage_scope='observed front only; sides/rear require operator clearance'))))


def main():
    rclpy.init();node=GuardNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.guard.revoke('guard shutting down')
        if rclpy.ok():node.output.publish(Twist())
        node.stop_event.set()
        if node.passive:node.passive.stop.set()
        node.destroy_node();rclpy.try_shutdown()
