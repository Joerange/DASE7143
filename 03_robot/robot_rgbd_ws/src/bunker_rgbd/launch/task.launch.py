"""One owned launch session; chassis is explicit and CAN configuration untouched."""
import os
import shlex
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,RegisterEventHandler,EmitEvent,TimerAction,SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from bunker_rgbd.timing import LAUNCH_EXIT_GRACE,LAUNCH_SIGNAL_TIMEOUT


def setup(context):
    if os.getenv('ROS_DOMAIN_ID')!='182':
        raise RuntimeError('Use ROS_DOMAIN_ID=182 for this isolated workspace; navigation workspace is not touched')
    def value(k):return LaunchConfiguration(k).perform(context)
    def flag(k):
        v=value(k).lower()
        if v not in ('true','false'):raise ValueError(k+' must be true or false')
        return v=='true'
    share=Path(get_package_share_directory('bunker_rgbd'))
    if flag('demo') and any(flag(k) for k in ('start_camera','start_chassis','allow_runtime_control')):
        raise ValueError('demo must not start sensors/chassis or allow runtime control')
    import yaml
    extrinsics=yaml.safe_load((share/'config/camera.yaml').read_text())
    nodes=[]
    for row in extrinsics['transforms']:
        args=[]
        for name,n in zip(('x','y','z','roll','pitch','yaw'),row['xyz']+row['rpy']):args+=['--'+name,str(n)]
        args+=['--frame-id',row['parent'],'--child-frame-id',row['child']]
        nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name=row['name'],arguments=args))
    if flag('start_camera'):
        camera=Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py'
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(camera)),launch_arguments={
            'camera_namespace':'camera','camera_name':'camera','enable_color':'true','enable_depth':'true',
            'align_depth.enable':'true','enable_sync':'true','publish_tf':'true',
            'rgb_camera.color_profile':'640x480x30','depth_module.depth_profile':'640x480x30'}.items()))
    guard=Node(package='bunker_rgbd',executable='guard',name='rgbd_guard',output='both',
               parameters=[{'allow_runtime_control':flag('allow_runtime_control')}])
    mission=Node(package='bunker_rgbd',executable='mission',name='rgbd_mission',output='both')
    perception=Node(package='bunker_rgbd',executable='perception',name='rgbd_perception',output='both',
             prefix=shlex.quote(value('perception_python')),
             parameters=[{'model_path':value('model_path'),'device':ParameterValue(value('device'),value_type=str),'target_label':'trash can',
                          'show_window':flag('show_window'),'demo':flag('demo')}])
    owned=[guard,mission,perception]
    if flag('start_chassis'):
        owned.append(Node(package='bunker_base',executable='bunker_base_node',name='bunker',output='screen',
                          parameters=[{'port_name':'can0','is_bunker_mini':True,'simulated_robot':False}]))
    for process in owned:
        # On ordinary window close, perception has already disabled/stopped and waited
        # for feedback. On a crash, this grace interval lets the independent guard's
        # freshness timeout output zero before launch tears down the driver.
        shutdown=EmitEvent(event=Shutdown(reason='RGB-D session process exited'))
        actions=[TimerAction(period=LAUNCH_EXIT_GRACE,actions=[shutdown])] if process is perception or process is mission else [shutdown]
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=actions)))
    nodes.extend(owned)
    return nodes


def generate_launch_description():
    home=Path.home()
    defaults=dict(start_camera='false',start_chassis='false',allow_runtime_control='false',show_window='true',demo='false',device='0',
                  perception_python=str(home/'yolo-world-agv/.venv/bin/python'),
                  model_path=str(home/'yolo-world-agv/models/yolov8s-worldv2-trash.pt'))
    return LaunchDescription([SetLaunchConfiguration('sigterm_timeout',str(LAUNCH_SIGNAL_TIMEOUT)),
                              SetLaunchConfiguration('sigkill_timeout',str(LAUNCH_SIGNAL_TIMEOUT))]+
                             [DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
