#!/usr/bin/env python3
"""No sensors/chassis/CAN/motion: real Humble node construction and zero preview."""
import os
os.environ['ROS_DOMAIN_ID']='182'
os.environ['ROS_LOCALHOST_ONLY']='1'
import time
import json
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from bunker_rgbd.mission import MissionNode
from bunker_rgbd.guard import GuardNode

rclpy.init();guard=GuardNode();mission=MissionNode();audit=rclpy.create_node('rgbd_smoke')
executor=SingleThreadedExecutor()
for node in (guard,mission,audit):executor.add_node(node)
commands=[]
sub=audit.create_subscription(Twist,'/rgbd/cmd_vel_preview',lambda m:commands.append((m.linear.x,m.angular.z)),10)
try:
    end=time.monotonic()+1.5
    while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
    result=dict(preview_messages=len(commands),all_zero=all(v==0 and w==0 for v,w in commands),
                final_publishers=audit.count_publishers('/cmd_vel'),authorized=guard.guard.authorized,
                mission_active=mission.task.active,can_receiver_created=guard.passive is not None)
    print(json.dumps(result,indent=2))
    assert commands and result['all_zero'] and result['final_publishers']==0
    assert not result['authorized'] and not result['mission_active'] and not result['can_receiver_created']
finally:
    guard.stop_event.set()
    for node in (guard,mission,audit):node.destroy_node()
    executor.shutdown();rclpy.try_shutdown()
