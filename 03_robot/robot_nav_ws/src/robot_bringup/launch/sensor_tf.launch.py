"""Publish mounting TF only; never start sensors, odometry, navigation or chassis."""
from pathlib import Path
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_transforms(context):
    path = Path(LaunchConfiguration('extrinsics_file').perform(context))
    config = yaml.safe_load(path.read_text())
    if config.get('schema_version') != 1 or config.get('units') != 'metres_and_radians':
        raise ValueError('Expected sensor extrinsics schema 1 in metres and radians')
    transforms = config['transforms']
    children, names, parents = set(), set(), {}
    for transform in transforms:
        parent, child, name = transform['parent'], transform['child'], transform['name']
        if not all(isinstance(s, str) and s and not s.startswith('/') for s in [parent, child, name]):
            raise ValueError('Frame IDs and node names must be nonempty and have no leading slash')
        if child in children or name in names or child == parent:
            raise ValueError('Duplicate child/node or self-parent in mounting transforms')
        if child in {'map', 'odom', 'base_link'} or 'optical' in child:
            raise ValueError('This launch may not own map/odom/base or factory optical transforms')
        children.add(child); names.add(name); parents[child] = parent
        for key in ['xyz', 'rpy']:
            values = transform[key]
            if len(values) != 3 or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values):
                raise ValueError(f'{name}: {key} must contain three finite numeric values')
    for child in parents:
        seen, cursor = set(), child
        while cursor in parents:
            if cursor in seen:
                raise ValueError('Cycle in mounting TF configuration')
            seen.add(cursor); cursor = parents[cursor]
    actions = [LogInfo(msg=f'Loading approximate sensor mounting TF: {path}. No chassis or odometry is launched.')]
    for transform in transforms:
        args = []
        for key, value in zip(['x', 'y', 'z', 'roll', 'pitch', 'yaw'], transform['xyz'] + transform['rpy']):
            args.extend(['--' + key, str(value)])
        args.extend(['--frame-id', transform['parent'], '--child-frame-id', transform['child']])
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name=transform['name'] + '_static_tf', arguments=args,
                            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}], output='screen'))
    return actions


def generate_launch_description():
    default = str(Path(get_package_share_directory('robot_bringup')) / 'config' / 'sensor_extrinsics.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('extrinsics_file', default_value=default,
                              description='Mounting configuration; sensor factory TF remains owned by the camera driver'),
        OpaqueFunction(function=create_transforms),
    ])
