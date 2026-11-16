#!/usr/bin/env python3
"""Validate dependencies in a localhost-only ROS domain; never drive the chassis."""
import os
os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
import math
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
import yaml
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from lifecycle_msgs.srv import GetState, ChangeState
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import StaticTransformBroadcaster

root = Path(__file__).resolve().parent
out = root / ('verification_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
out.mkdir(exist_ok=False)
report = {'started_utc': datetime.now(timezone.utc).isoformat(), 'ros_domain_id': 173,
          'localhost_only': True, 'scope': 'Stationary dependency checks; synthetic odometry and map are test fixtures only.',
          'geometry_assumptions': {'base': 'body centre projected onto floor', 'length_m': .690,
                                   'width_m': .570, 'lidar_xyz_m': [.125, 0., .600],
                                   'lidar_rpy_rad': [0., 0., 0.], 'lateral_offset_and_orientation': 'unverified test assumptions'},
          'packages': {}, 'lifecycle': {}, 'processes': {}}
processes = {}
samples = {'cloud': [], 'scan': []}
last_scan = None
maps = []
cmd_messages = []
node = None

def run(args, timeout=20):
    r = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
    return {'returncode': r.returncode, 'stdout': r.stdout, 'stderr': r.stderr}

def can_state():
    return json.loads(subprocess.check_output(['ip', '-j', 'link', 'show', 'type', 'can'], text=True))

def pump(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)

def wait_future(future, seconds=12):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    if not future.done():
        raise TimeoutError('ROS service/action request timed out')
    return future.result()

def start(key, package, executable, args=()):
    exe = Path(get_package_prefix(package)) / 'lib' / package / executable
    log = (out / (key + '.log')).open('w')
    command = [str(exe), *args]
    p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes[key] = (p, log)
    report['processes'][key] = {'command': command, 'pid': p.pid}

def stop(key):
    p, log = processes.pop(key)
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=3)
    log.close()
    report['processes'][key]['exit_code'] = p.returncode

def transition(name, transition_id, expected):
    c = node.create_client(ChangeState, '/' + name + '/change_state')
    g = node.create_client(GetState, '/' + name + '/get_state')
    try:
        if not c.wait_for_service(timeout_sec=10):
            raise RuntimeError(name + ' lifecycle service unavailable')
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = wait_future(c.call_async(request))
        if not g.wait_for_service(timeout_sec=4):
            raise RuntimeError(name + ' state service unavailable')
        state = wait_future(g.call_async(GetState.Request())).current_state
        report['lifecycle'].setdefault(name, []).append({'transition': transition_id,
                                                       'success': response.success, 'state': state.label})
        if not response.success or state.label != expected:
            raise RuntimeError(name + ' failed lifecycle transition to ' + expected)
    finally:
        node.destroy_client(c)
        node.destroy_client(g)

def tf(parent, child, xyz):
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.w = 1.
    return t

def scan_cb(msg):
    global last_scan
    samples['scan'].append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
    last_scan = msg

try:
    report['can_before'] = can_state()
    if any('UP' in item.get('flags', []) for item in report['can_before']):
        raise RuntimeError('Unexpected active CAN interface; aborting stationary validation')
    pkgs = ['navigation2', 'nav2_bringup', 'nav2_amcl', 'nav2_map_server', 'nav2_navfn_planner',
            'nav2_planner', 'nav2_controller', 'dwb_core', 'nav2_bt_navigator',
            'nav2_velocity_smoother', 'nav2_collision_monitor', 'nav2_simple_commander',
            'slam_toolbox', 'pointcloud_to_laserscan']
    for package in pkgs:
        report['packages'][package] = {'prefix': get_package_prefix(package),
            'dpkg': run(['dpkg-query', '-W', '-f=${db:Status-Abbrev} ${Version}', 'ros-humble-' + package.replace('_', '-')])}
    from nav2_simple_commander.robot_navigator import BasicNavigator
    report['nav2_python_import'] = 'passed'
    report['linker'] = {}
    for package, executable in [('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
                                ('nav2_amcl', 'amcl'), ('nav2_bt_navigator', 'bt_navigator'),
                                ('nav2_map_server', 'map_server'), ('slam_toolbox', 'async_slam_toolbox_node'),
                                ('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node'),
                                ('nav2_velocity_smoother', 'velocity_smoother'), ('nav2_collision_monitor', 'collision_monitor')]:
        result = run(['ldd', str(Path(get_package_prefix(package)) / 'lib' / package / executable)])
        missing = [line for line in result['stdout'].splitlines() if 'not found' in line]
        report['linker'][package] = {'returncode': result['returncode'], 'missing': missing}
        if result['returncode'] or missing:
            raise RuntimeError(package + ' has a linker failure')
    rclpy.init()
    node = rclpy.create_node('dependency_stationary_verifier')
    pump(2)
    report['initial_nodes'] = node.get_node_names_and_namespaces()
    if len(report['initial_nodes']) != 1:
        raise RuntimeError('ROS test domain is already in use')
    subs = [node.create_subscription(LaserScan, '/dependency_check/scan', scan_cb, qos_profile_sensor_data),
            node.create_subscription(PointCloud2, '/cx/lslidar_point_cloud',
                lambda m: samples['cloud'].append((time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec*1e-9)), qos_profile_sensor_data),
            node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(Twist, '/cmd_vel', lambda m: cmd_messages.append(m), 10),
            node.create_subscription(Twist, '/dependency_check/unused_cmd_vel', lambda m: cmd_messages.append(m), 10)]
    broadcaster = StaticTransformBroadcaster(node)
    fixed = [tf('audit_odom', 'audit_base_link', [0., 0., 0.]),
             tf('audit_base_link', 'laser_link', [.125, 0., .600])]
    broadcaster.sendTransform(fixed)
    lidar_config = Path(get_package_share_directory('lslidar_driver')) / 'config/lslidar_cx.yaml'
    start('lidar', 'lslidar_driver', 'lslidar_driver_node',
          ['--ros-args', '-r', '__ns:=/cx', '-r', '__node:=lslidar_driver_node', '--params-file', str(lidar_config)])
    projection = {'pointcloud_to_laserscan': {'ros__parameters': {'target_frame': 'audit_base_link',
        'transform_tolerance': .1, 'min_height': .1, 'max_height': .9, 'angle_min': -math.pi,
        'angle_max': math.pi, 'angle_increment': math.pi/360., 'scan_time': .1,
        'range_min': .4, 'range_max': 10., 'use_inf': True}}}
    projection_path = out / 'projection_test.yaml'
    projection_path.write_text(yaml.safe_dump(projection))
    start('converter', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
          ['--ros-args', '-r', '__node:=pointcloud_to_laserscan', '-r', 'cloud_in:=/cx/lslidar_point_cloud',
           '-r', 'scan:=/dependency_check/scan', '--params-file', str(projection_path)])
    slam_cfg = yaml.safe_load((Path(get_package_share_directory('slam_toolbox'))/'config/mapper_params_online_async.yaml').read_text())
    slam_cfg['slam_toolbox']['ros__parameters'].update(use_sim_time=False, odom_frame='audit_odom',
        map_frame='audit_map', base_frame='audit_base_link', scan_topic='/dependency_check/scan',
        mode='mapping', resolution=.05, min_laser_range=.4, max_laser_range=10., map_update_interval=1.,
        minimum_time_interval=.2, enable_interactive_mode=False)
    slam_path = out / 'slam_stationary_test.yaml'
    slam_path.write_text(yaml.safe_dump(slam_cfg))
    start('slam', 'slam_toolbox', 'async_slam_toolbox_node',
          ['--ros-args', '-r', '__node:=slam_toolbox', '--params-file', str(slam_path)])
    pump(18)
    if last_scan is None:
        raise RuntimeError('No converted LaserScan received')
    finite = np.asarray(last_scan.ranges)
    finite = finite[np.isfinite(finite)]
    report['projection'] = {'scan_messages': len(samples['scan']), 'cloud_messages': len(samples['cloud']),
        'frame': last_scan.header.frame_id, 'bins': len(last_scan.ranges), 'finite_bins': len(finite),
        'range_min_m': float(finite.min()) if len(finite) else None,
        'range_max_m': float(finite.max()) if len(finite) else None}
    for key, rows in samples.items():
        report['projection'][key+'_received_hz'] = (len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else 0.
    if not len(finite):
        raise RuntimeError('LaserScan has no finite ranges')
    if not maps:
        raise RuntimeError('SLAM did not publish an initial map')
    m = maps[-1]
    report['slam'] = {'map_messages': len(maps), 'frame': m.header.frame_id,
        'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution,
        'known_cells': sum(x >= 0 for x in m.data), 'occupied_cells': sum(x >= 65 for x in m.data),
        'limitation': 'One stationary viewpoint with fixed test odometry; not a navigable surveyed map.'}
    if not report['slam']['known_cells']:
        raise RuntimeError('SLAM map has no known cells')
    stop('slam')
    stop('converter')
    stop('lidar')
    pump(1)
    # The next phase uses a synthetic empty map solely to verify the planning API.
    broadcaster.sendTransform(fixed + [tf('audit_map', 'audit_odom', [0., 0., 0.])])
    pixels = bytearray([254] * (100*100))
    for i in range(100):
        pixels[i] = pixels[9900+i] = pixels[100*i] = pixels[100*i+99] = 0
    (out/'synthetic_map.pgm').write_bytes(b'P5\n100 100\n255\n'+pixels)
    synthetic_map = out/'synthetic_map.yaml'
    synthetic_map.write_text(yaml.safe_dump({'image': 'synthetic_map.pgm', 'mode': 'trinary',
        'resolution': .1, 'origin': [-5., -5., 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}))
    cfg = yaml.safe_load((Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml').read_text())
    def real_time(value):
        if isinstance(value, dict):
            for key in value:
                if key == 'use_sim_time': value[key] = False
                else: real_time(value[key])
    real_time(cfg)
    for name in ['map_server', 'planner_server', 'controller_server', 'amcl', 'velocity_smoother']:
        cfg[name]['ros__parameters']['bond_heartbeat_period'] = 0.
    cfg['map_server']['ros__parameters'].update(yaml_filename=str(synthetic_map), frame_id='audit_map')
    cfg['amcl']['ros__parameters'].update(base_frame_id='audit_base_link', odom_frame_id='audit_odom',
        global_frame_id='audit_map', scan_topic='/dependency_check/scan', tf_broadcast=False)
    footprint = '[[0.345,0.285],[0.345,-0.285],[-0.345,-0.285],[-0.345,0.285]]'
    for key, frame in [('global_costmap','audit_map'), ('local_costmap','audit_odom')]:
        cm = cfg[key][key]['ros__parameters']
        cm.update(global_frame=frame, robot_base_frame='audit_base_link', footprint=footprint)
        cm.pop('robot_radius', None)
    cfg['global_costmap']['global_costmap']['ros__parameters']['plugins'] = ['static_layer', 'inflation_layer']
    cfg['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    cfg['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['scan']['topic'] = '/dependency_check/scan'
    cfg['collision_monitor'] = {'ros__parameters': {'use_sim_time': False, 'base_frame_id': 'audit_base_link',
        'odom_frame_id': 'audit_odom', 'cmd_vel_in_topic': '/dependency_check/no_input',
        'cmd_vel_out_topic': '/dependency_check/unused_cmd_vel', 'state_topic': '/dependency_check/collision_state',
        'transform_tolerance': .5, 'source_timeout': 1., 'base_shift_correction': True,
        'stop_pub_timeout': 1., 'polygons': ['StopZone'],
        'StopZone': {'type': 'polygon', 'points': [0.5,0.4,0.5,-0.4,-0.4,-0.4,-0.4,0.4],
                     'action_type': 'stop', 'min_points': 4, 'visualize': False},
        'observation_sources': ['scan'], 'scan': {'type':'scan','topic':'/dependency_check/scan'}}}
    nav_path = out/'nav_dependency_test.yaml'
    nav_path.write_text(yaml.safe_dump(cfg))
    for name, package, executable in [('map_server','nav2_map_server','map_server'),
            ('planner_server','nav2_planner','planner_server'), ('controller_server','nav2_controller','controller_server'),
            ('amcl','nav2_amcl','amcl'), ('velocity_smoother','nav2_velocity_smoother','velocity_smoother'),
            ('collision_monitor','nav2_collision_monitor','collision_monitor')]:
        start(name, package, executable, ['--ros-args', '--params-file', str(nav_path),
            '-r', 'cmd_vel:=/dependency_check/unused_cmd_vel',
            '-r', 'cmd_vel_smoothed:=/dependency_check/unused_cmd_vel_smoothed'])
        transition(name, 1, 'inactive')
    transition('map_server', 3, 'active')
    transition('planner_server', 3, 'active')
    pump(3)
    action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if not action.wait_for_server(timeout_sec=5):
        raise RuntimeError('ComputePathToPose action is unavailable')
    goal = ComputePathToPose.Goal()
    goal.planner_id = 'GridBased'
    goal.use_start = True
    for pose, x in [(goal.start, 0.), (goal.goal, 1.)]:
        pose.header.frame_id = 'audit_map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
    handle = wait_future(action.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError('Planner rejected synthetic test goal')
    result = wait_future(handle.get_result_async(), 15)
    report['planner'] = {'action_status': result.status, 'path_poses': len(result.result.path.poses),
        'path_frame': result.result.path.header.frame_id, 'synthetic_goal_m': [1., 0.],
        'motion_action_sent': False}
    if result.status != 4 or len(result.result.path.poses) < 2:
        raise RuntimeError('Synthetic pure-planning action failed')
    report['cmd_vel_messages_observed'] = len(cmd_messages)
    if cmd_messages:
        raise RuntimeError('Unexpected velocity output during dependency test')
    report['status'] = 'passed'
except Exception as error:
    report['status'] = 'failed'
    report['error'] = type(error).__name__ + ': ' + str(error)
finally:
    for key in list(processes)[::-1]:
        stop(key)
    if node is not None:
        pump(2)
        report['nodes_after_cleanup'] = node.get_node_names_and_namespaces()
        node.destroy_node()
        rclpy.shutdown()
    report['can_after'] = can_state()
    report['dpkg_audit'] = run(['dpkg', '--audit'])
    report['completed_utc'] = datetime.now(timezone.utc).isoformat()
    (out/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({'status': report.get('status'), 'error': report.get('error'), 'report':str(out/'report.json')}, indent=2))
    if report.get('status') != 'passed':
        raise SystemExit(1)
