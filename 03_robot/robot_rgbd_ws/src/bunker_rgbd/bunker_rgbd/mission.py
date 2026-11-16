import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from .core import Mission,fresh,finite
from .timing import OBSERVATION_TIMEOUT
from .ros_utils import seconds


class MissionNode(Node):
    def __init__(self):
        super().__init__('rgbd_mission');self.task=Mission();self.obs={};self.safety={};self.odom={}
        self.reported_state=None
        self.pub=self.create_publisher(String,'/rgbd/mission/state',1)
        self.cmd=self.create_publisher(String,'/rgbd/cmd_request',1)
        self.disarm=self.create_client(SetBool,'/rgbd/safety/set_motion_enabled')
        self.create_subscription(String,'/rgbd/observation',lambda m:self.json_cb('obs',m),1)
        self.create_subscription(String,'/rgbd/safety/status',lambda m:self.json_cb('safety',m),1)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        self.create_service(Trigger,'/rgbd/mission/start',self.start_cb)
        self.create_service(Trigger,'/rgbd/mission/stop',self.stop_cb)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def json_cb(self,key,msg):
        try:
            d=json.loads(msg.data)
            if d.get('schema')==1 and isinstance(d.get('stamp'),(float,int)):
                old=getattr(self,key)
                if d['stamp']>old.get('stamp',-1):setattr(self,key,dict(d,_rx=time.monotonic()))
        except (ValueError,TypeError):pass
    def odom_cb(self,m):self.odom=dict(stamp=seconds(m.header.stamp),_rx=time.monotonic(),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,x=m.pose.pose.position.x,y=m.pose.pose.position.y, yaw=math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.z+m.pose.pose.orientation.x*m.pose.pose.orientation.y),1-2*(m.pose.pose.orientation.y**2+m.pose.pose.orientation.z**2)))
    def start_readiness(self,n,r):
        if self.task.active:return False,'task already active'
        if not fresh(self.safety,n,r):return False,'safety status stale'
        if not self.safety.get('authorized'):return False,'motion permission not received by mission'
        if not self.safety.get('ready'):return False,self.safety.get('reason','safety not ready')
        if not fresh(self.obs,n,r,OBSERVATION_TIMEOUT) or not self.obs.get('ready'):return False,'YOLO/RGB-D observation not ready'
        if not fresh(self.odom,n,r) or not finite(*(self.odom.get(k) for k in ('v','w','x','y','yaw'))):
            return False,'odometry not ready'
        return True,'ready'
    def start_cb(self,req,res):
        n=time.monotonic();r=self.now()
        res.success,res.message=self.start_readiness(n,r)
        if res.success:res.message='started'
        if res.success:self.task.start(n)
        self.tick();return res
    def stop_cb(self,req,res):
        self.task.stop('operator stop');self.tick()
        if self.disarm.service_is_ready():
            request=SetBool.Request();request.data=False;self.disarm.call_async(request)
        res.success=True;res.message='stop requested; inspect state and measured velocity for standstill';return res
    def tick(self):
        now=time.monotonic();ros_now=self.now()
        v,w=self.task.step(now,ros_now,self.obs,self.safety,self.odom)
        start_ready,start_reason=self.start_readiness(now,ros_now)
        transition=(self.task.state,self.task.reason)
        if transition!=self.reported_state:
            self.get_logger().info('Task '+self.task.state+': '+self.task.reason)
            self.reported_state=transition
        packet=dict(schema=1,stamp=ros_now,generation=self.task.generation,active=self.task.active,
                    state=self.task.state,reason=self.task.reason,target=self.task.target,
                    sensor_mode='rgbd_only',stop_gap=.30,goal=self.task.goal,tracking_mode=self.task.tracking_mode,
                    completion_basis=self.task.completion_basis,start_ready=start_ready,start_block_reason=start_reason)
        self.pub.publish(String(data=json.dumps(packet)))
        self.cmd.publish(String(data=json.dumps(dict(packet,v=v,w=w))))


def main():
    rclpy.init();node=MissionNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.task.stop('mission process shutting down')
        if rclpy.ok():node.tick()
        node.destroy_node();rclpy.try_shutdown()
