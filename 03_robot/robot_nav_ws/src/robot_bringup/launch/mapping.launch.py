"""SLAM map acquisition with real externally supplied odometry; no command output.

Standalone mapping only. Do not run beside task/nav2: each already owns a SLAM instance.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('robot_bringup'))
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        LogInfo(msg='Mapping only: no chassis, CAN setup or cmd_vel publisher. Supply real odometry. Do not run another SLAM or AMCL instance.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensors.launch.py')),
             launch_arguments={'start_sensors':LaunchConfiguration('start_sensors').perform(context),
                               'use_sim_time':str(sim).lower()}.items()),
        Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',output='screen',
             parameters=[str(share/'config/slam.yaml'),{'use_sim_time':sim}],
             remappings=[('pose','/slam_toolbox/pose')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('start_sensors',default_value='false'),
        OpaqueFunction(function=setup)])
