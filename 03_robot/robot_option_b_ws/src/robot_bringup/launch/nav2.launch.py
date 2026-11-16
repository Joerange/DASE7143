"""Online SLAM + Humble Nav2. No saved map or AMCL localization route.

SLAM Toolbox alone owns map->odom. Real chassis odometry owns odom->base_link.
All controller outputs remain upstream of the independent command guard.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def setup(context):
    share = Path(get_package_share_directory('robot_bringup'))
    def flag(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(name+' must be true or false')
        return value == 'true'
    sim, active = flag('use_sim_time'), flag('nav_autostart')
    params = RewrittenYaml(source_file=LaunchConfiguration('params_file'),
                           param_rewrites={'use_sim_time': str(sim).lower()}, convert_types=True)
    def node(package, executable, extra=None, remappings=None):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[params] + ([extra] if extra else []), remappings=remappings or [])
    nodes = [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=[str(share/'config/slam.yaml'), {'use_sim_time':sim}],
             remappings=[('pose', '/slam_toolbox/pose')]),
        node('nav2_planner','planner_server'),
        node('nav2_controller','controller_server', remappings=[('cmd_vel','/cmd_vel_nav')]),
        node('nav2_behaviors','behavior_server', remappings=[('cmd_vel','/cmd_vel_nav')]),
        node('nav2_bt_navigator','bt_navigator',{
            'default_nav_to_pose_bt_xml':str(share/'behavior_trees/navigate_safe.xml'),
            'default_nav_through_poses_bt_xml':str(share/'behavior_trees/navigate_through_safe.xml')}),
        node('nav2_velocity_smoother','velocity_smoother', remappings=[
            ('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel_nav_smoothed')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time':sim, 'autostart':active, 'bond_timeout':4.0,
                          'node_names':['planner_server','controller_server','behavior_server',
                                        'bt_navigator','velocity_smoother']}]),
    ]
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('robot_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=str(share/'config/nav2.yaml')),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('nav_autostart',default_value='true'),
        OpaqueFunction(function=setup)])
