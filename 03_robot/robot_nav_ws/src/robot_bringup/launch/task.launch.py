"""Main task with optional online SLAM navigation; disarmed and manual by default."""
from pathlib import Path
import shlex
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('robot_bringup'))
    def value(name): return LaunchConfiguration(name).perform(context)
    def flag(name):
        v = value(name).lower()
        if v not in ('true','false'): raise ValueError(name+' must be true or false')
        return v == 'true'
    sim = flag('use_sim_time')
    navigation, runtime_control = flag('enable_navigation'), flag('allow_runtime_control')
    # Retain these false arguments for diagnostic-script compatibility, but do
    # not let a launch command grant motion or bypass the driver supervisor.
    if flag('motion_enabled'):
        raise ValueError('motion_enabled=true is retired: use allow_runtime_control=true and explicitly authorize through /safety/set_motion_enabled')
    if flag('start_chassis'):
        raise ValueError('start_chassis=true is retired: use the controlled console driver supervisor')
    def include(file, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch'/file)),launch_arguments=args.items())
    nodes = [LogInfo(msg='Runtime control capable: '+str(runtime_control)+'; startup disarmed. CAN is never configured by this launch.'),
             include('sensors.launch.py',{'start_sensors':value('start_sensors'),'use_sim_time':value('use_sim_time')})]
    if navigation:
        nodes.append(include('nav2.launch.py',{'use_sim_time':value('use_sim_time'),
                                              'nav_autostart':value('nav_autostart'),
                                              'params_file':value('nav_params_file')}))
    nodes.extend([
        Node(package='robot_safety',executable='command_guard_node',name='command_guard_node',output='screen',
             parameters=[str(share/'config/mission.yaml'),{'use_sim_time':sim,'motion_enabled':False,'allow_runtime_control':runtime_control,'localization_mode':'slam'}]),
        Node(package='robot_mission',executable='mission_node',name='mission_node',output='screen',
             parameters=[str(share/'config/mission.yaml'),{'use_sim_time':sim,'motion_enabled':False,'allow_runtime_control':runtime_control,
                         'autostart':flag('autostart'),'target_label':value('target_label')}]),
    ])
    if flag('enable_perception'):
        python_path = Path(value('perception_python')).expanduser()
        model_path = Path(value('model_path')).expanduser()
        if not python_path.is_file(): raise ValueError('perception_python does not exist: '+str(python_path))
        if not model_path.is_file(): raise ValueError('model_path does not exist: '+str(model_path))
        nodes.extend([
            Node(package='robot_perception',executable='perception_node',name='perception_node',output='screen',
                 prefix=shlex.quote(str(python_path)),
                 additional_env={'YOLO_CONFIG_DIR':str(Path.home()/'yolo-world-agv/.ultralytics')},
                 parameters=[str(share/'config/perception.yaml'),{'use_sim_time':sim,'model_path':str(model_path),
                             'target_label':value('target_label')}]),
            Node(package='robot_perception',executable='vlm_confirm_node',name='vlm_confirm_node',output='screen',
                 prefix=shlex.quote(value('vlm_python')),
                 parameters=[str(share/'config/perception.yaml'),{'use_sim_time':sim,'target_label':value('target_label')}]),
        ])
    if flag('rviz'):
        nodes.append(Node(package='rviz2',executable='rviz2',name='rviz2',output='screen',
                          arguments=['-d',str(share/'rviz/task.rviz')],parameters=[{'use_sim_time':sim}]))
    return nodes


def generate_launch_description():
    home = Path.home()
    defaults = {
        'use_sim_time':'false', 'motion_enabled':'false', 'allow_runtime_control':'false', 'autostart':'false',
        'start_chassis':'false', 'start_sensors':'false', 'enable_navigation':'false',
        'enable_perception':'true', 'nav_autostart':'true', 'rviz':'false',
        'nav_params_file':str(Path(get_package_share_directory('robot_bringup'))/'config/nav2.yaml'),
        'can_interface':'can0', 'target_label':'trash can',
        'perception_python':str(home/'yolo-world-agv/.venv/bin/python'),
        'vlm_python':'/usr/bin/python3', 'model_path':str(home/'yolo-world-agv/models/yolov8s-worldv2-trash.pt'),
    }
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
