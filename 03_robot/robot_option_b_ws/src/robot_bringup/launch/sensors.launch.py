"""Mounting TF, optional real sensors, body filtering and LaserScan projection."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('robot_bringup'))
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    start = LaunchConfiguration('start_sensors').perform(context).lower() == 'true'
    nodes = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensor_tf.launch.py')),
                                     launch_arguments={'use_sim_time': str(sim).lower()}.items())]
    if start:
        camera = Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py'
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(camera)), launch_arguments={
            'camera_namespace':'camera', 'camera_name':'camera',
            'enable_color':'true', 'enable_depth':'true', 'align_depth.enable':'true',
            'enable_sync':'true', 'publish_tf':'true',
            'rgb_camera.color_profile':'640x480x30', 'depth_module.depth_profile':'640x480x30',
            'use_sim_time':str(sim).lower(),
        }.items()))
        lidar = Path(get_package_share_directory('lslidar_driver'))/'config/lslidar_cx.yaml'
        nodes.append(Node(package='lslidar_driver', executable='lslidar_driver_node',
                          name='lslidar_driver_node', namespace='cx',
                          parameters=[str(lidar), {'use_sim_time':sim}], output='screen'))
    nodes.extend([
        Node(package='robot_bringup', executable='lidar_filter_node', name='lidar_filter_node',
             parameters=[str(share/'config/scan.yaml'),{'use_sim_time':sim}], output='screen')])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('start_sensors',default_value='false',description='Start actual RealSense and C32 drivers'),
        OpaqueFunction(function=setup)])
