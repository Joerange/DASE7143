#!/usr/bin/env python3
"""Session-only AGX V2 receive-only odometry. NO CAN writes or cmd_vel topics.

Wire layout is taken from the installed ugv_sdk protocol_v2 implementation:
0x211 system status, 0x221 big-endian int16 linear/angular speed /1000.
Pose integrates measured chassis feedback, as bunker_messenger does; no fake odom.
"""
import json
import math
import socket
import struct
import sys
import time


def decode_motion(data):
    if len(data) != 8:
        raise ValueError('invalid motion frame length')
    v, w, lateral, steering = (x / 1000. for x in struct.unpack('>hhhh', data))
    if abs(v) > 2. or abs(w) > 3. or abs(lateral) > .01:
        raise ValueError('unexpected Bunker motion feedback')
    return v, w


def integrate(x, y, yaw, v, w, dt):
    if abs(w) < 1e-8:
        return x+v*dt*math.cos(yaw), y+v*dt*math.sin(yaw), yaw
    a=yaw+w*dt
    return x+v/w*(math.sin(a)-math.sin(yaw)), y-v/w*(math.cos(a)-math.cos(yaw)), math.atan2(math.sin(a),math.cos(a))


if '--self-test' in sys.argv:
    assert decode_motion(bytes.fromhex('0064ff9c00000000')) == (.1,-.1)
    assert integrate(0,0,0,.1,0,1) == (.1,0.,0)
    x,y,a=integrate(0,0,0,1,1,math.pi/2)
    assert abs(x-1)<1e-9 and abs(y-1)<1e-9 and abs(a-math.pi/2)<1e-9
    assert bytes.fromhex('0003010200000000')[1] == 3
    print('Passive odometry decoder and integration: PASS')
    raise SystemExit(0)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class PassiveOdometry(Node):
    def __init__(self):
        super().__init__('mapping_passive_odometry')
        self.sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,
                             struct.pack('=IIII',0x211,0xC00007ff,0x221,0xC00007ff))
        self.sock.bind(('can0',))
        self.sock.setblocking(False)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.health_pub=self.create_publisher(String,'/mapping/odom_status',10)
        self.broadcaster=TransformBroadcaster(self)
        self.pose=(0.,0.,0.)
        self.previous=None
        self.last_rx=None
        self.system=None
        self.system_rx=0.
        self.frames=0
        self.started=False
        self.fault=''
        self.still_since=None
        self.create_timer(.005,self.poll)
        self.create_timer(.5,self.status)
        self.get_logger().info('Receive-only CAN odometry: no control-mode or velocity transmission')

    def poll(self):
        newest=None
        for _ in range(100):
            try:
                raw=self.sock.recv(16)
            except BlockingIOError:
                break
            except OSError:
                self.fault='can_socket_error'
                return
            if len(raw)!=16:
                continue
            ident,length,data=struct.unpack('=IB3x8s',raw)
            if length!=8:
                continue
            now=time.monotonic()
            if ident==0x211:
                self.system=dict(vehicle_state=data[0],control_mode=data[1],
                                 battery_voltage=int.from_bytes(data[2:4],'big')*.1,
                                 error_code=int.from_bytes(data[4:6],'big'))
                self.system_rx=now
            elif ident==0x221:
                try:
                    newest=(now,*decode_motion(data))
                    self.frames+=1
                except ValueError:
                    self.fault='invalid_motion_frame'
        if newest is None or self.fault:
            return
        now,v,w=newest
        if not self.system or now-self.system_rx>1.:
            if self.started:self.fault='system_feedback_lost'
            return
        if self.system['control_mode']!=3:
            if self.started:self.fault='remote_control_mode_lost'
            return
        if not self.started:
            if abs(v)>.01 or abs(w)>.02:
                self.still_since=None
                return
            if self.still_since is None:self.still_since=now
            if now-self.still_since<.5:return
            self.started=True
            self.get_logger().info('Real feedback confirmed in RC mode; stationary origin initialized')
        if self.previous:
            t0,v0,w0=self.previous
            dt=now-t0
            if not 0<dt<=.25:
                self.fault='motion_feedback_gap_restart_mapping_required'
                return
            self.pose=integrate(*self.pose,(v0+v)/2,(w0+w)/2,dt)
        self.previous=newest
        self.last_rx=now
        stamp=self.get_clock().now().to_msg()
        x,y,a=self.pose
        odom=Odometry()
        odom.header.stamp=stamp
        odom.header.frame_id='odom'
        odom.child_frame_id='base_link'
        odom.pose.pose.position.x=x
        odom.pose.pose.position.y=y
        odom.pose.pose.orientation.z=math.sin(a/2)
        odom.pose.pose.orientation.w=math.cos(a/2)
        odom.twist.twist.linear.x=v
        odom.twist.twist.angular.z=w
        odom.pose.covariance[0]=odom.pose.covariance[7]=.02
        odom.pose.covariance[35]=.03
        odom.twist.covariance[0]=.01
        odom.twist.covariance[35]=.02
        tf=TransformStamped()
        tf.header=odom.header
        tf.child_frame_id=odom.child_frame_id
        tf.transform.translation.x=x
        tf.transform.translation.y=y
        tf.transform.rotation=odom.pose.pose.orientation
        self.odom_pub.publish(odom)
        self.broadcaster.sendTransform(tf)

    def status(self):
        now=time.monotonic()
        age=now-self.last_rx if self.last_rx else None
        if self.started and age is not None and age>.25:
            self.fault=self.fault or 'motion_feedback_stale_restart_mapping_required'
        report=dict(schema=1,stamp=self.get_clock().now().nanoseconds/1e9,
                    receive_only=True,can_writes=0,protocol='AGX_V2',frames=self.frames,
                    initialized=self.started,ready=self.started and not self.fault and age is not None and age<.15,
                    feedback_age=age,system=self.system,pose=list(self.pose),fault=self.fault)
        self.health_pub.publish(String(data=json.dumps(report)))


def main():
    rclpy.init()
    node=PassiveOdometry()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
