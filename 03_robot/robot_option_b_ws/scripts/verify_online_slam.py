#!/usr/bin/env python3
"""Isolated live-sensor online-SLAM integration; no chassis and no motion.

Phase 1 has no odometry and must fail closed. Phase 2 uses an explicitly marked
TEST-ONLY zero-odometry/dynamic odom->base_link fixture. CAN is never changed.
An already-UP interface requires --allow-preexisting-can-up plus no running driver.
The default still rejects any UP interface. Console monitoring is read-only.
The real SLAM node alone creates /map and map->odom from live LiDAR. This proves
software integration, NOT real odometry, localization accuracy or navigation.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import argparse
import base64
import hashlib
import json
import math
import signal
import socket
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from robot_mission.core import Grid


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def running_chassis_drivers():
    """Read /proc only; never terminate another process or infer driver state from CAN UP."""
    found=[]
    for item in Path('/proc').glob('[0-9]*'):
        try:
            argv=(item/'cmdline').read_bytes().decode(errors='replace').split('\0')
            if any(Path(arg).name in ('bunker_base_node','bunker_base_ros') for arg in argv[:3]):
                found.append({'pid':int(item.name),'argv':argv})
        except (OSError,ValueError):pass
    return found


def can_configuration(links):
    """Compare administrative configuration, not receive counters or external bus errors."""
    result=[]
    for item in links:
        info=item.get('linkinfo',{}).get('info_data',{})
        result.append({'name':item.get('ifname'),'admin_up':'UP' in item.get('flags',[]),
                       'mtu':item.get('mtu'),'bitrate':info.get('bittiming',{}).get('bitrate'),
                       'restart_ms':info.get('restart_ms'),'ctrlmode':info.get('ctrlmode')})
    return sorted(result,key=lambda x:x['name'])


def console_snapshot(path):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(1.5);client.connect(str(path))
        client.sendall(b'{"op":"snapshot"}\n')
        payload=b''
        while not payload.endswith(b'\n'):
            block=client.recv(65536)
            if not block:raise RuntimeError('Console closed before snapshot completed')
            payload+=block
            if len(payload)>16*1024*1024:raise RuntimeError('Console snapshot exceeds audit limit')
    result=json.loads(payload)
    if not result.get('ok'):raise RuntimeError('Console snapshot failed')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=str(Path.home()/'robot_online_slam_20261109/static_integration'))
    parser.add_argument('--enable-perception', choices=['true','false'], default='true')
    parser.add_argument('--allow-preexisting-can-up', action='store_true',
                        help='Read-only audit of already-UP CAN; requires no driver, never changes link state')
    args = parser.parse_args()
    out = Path(args.output).expanduser()/datetime.now().strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    report = {'started_utc':datetime.now(timezone.utc).isoformat(), 'domain':179,
              'scope':'Live-sensor online SLAM and pure planning; no chassis/navigation action/motion.',
              'test_fixtures':['20Hz zero Odometry and dynamic odom->base_link in domain179 ONLY'],
              'not_fixtures':['live RGB/depth camera images','live LiDAR /scan','SLAM-generated /map','SLAM-generated map->odom'],
              'lifecycle':{}, 'counts':{}, 'samples':{}, 'checks':{}, 'passed':False}
    node = proc = logfile = console_proc = console_log = ui_proc = ui_log = None
    console_socket=Path('/tmp')/('robot-console-audit-'+str(os.getpid())+'-'+str(int(time.time()))+'.sock')
    last_driver_check=0.
    fixture = {'enabled':False}
    latest = {}
    started = time.monotonic()
    def can_state():
        return json.loads(subprocess.check_output(['/usr/sbin/ip','-details','-j','link','show','type','can'],text=True))
    def pump(seconds):
        nonlocal last_driver_check
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if proc is not None and proc.poll() is not None:
                raise RuntimeError('task launch exited before verification completed')
            if console_proc is not None and console_proc.poll() is not None:
                raise RuntimeError('read-only console backend exited')
            if node.count_publishers('/cmd_vel'):
                raise RuntimeError('Unexpected final /cmd_vel publisher in audit domain')
            if time.monotonic()-last_driver_check>1.:
                last_driver_check=time.monotonic()
                if running_chassis_drivers():
                    raise RuntimeError('A chassis driver exists; stationary audit refuses to proceed')
    def until(condition, timeout, reason):
        end = time.monotonic()+timeout
        while not condition():
            if time.monotonic()>=end:
                raise TimeoutError(reason)
            pump(.05)
    def future_result(future, timeout=15.):
        until(future.done,timeout,'ROS request deadline expired')
        return future.result()
    def state(name):
        client=node.create_client(GetState,'/'+name+'/get_state')
        try:
            until(client.service_is_ready,8.,name+' state service missing')
            return future_result(client.call_async(GetState.Request())).current_state.label
        finally:
            node.destroy_client(client)
    def transition(name, code, expected):
        client=node.create_client(ChangeState,'/'+name+'/change_state')
        try:
            until(client.service_is_ready,12.,name+' lifecycle service missing')
            req=ChangeState.Request();req.transition.id=code
            answer=future_result(client.call_async(req),25.)
            observed=state(name)
            report['lifecycle'].setdefault(name,[]).append({'transition':code,'success':answer.success,'state':observed})
            if not answer.success or observed!=expected:
                raise RuntimeError(name+' did not become '+expected)
        finally:
            node.destroy_client(client)
    def count(name):
        report['counts'][name]=report['counts'].get(name,0)+1
    def receive_json(name,msg):
        count(name)
        value=json.loads(msg.data)
        latest[name]=value
        # Candidate images are not needed in this audit report.
        report['samples'][name]={k:v for k,v in value.items() if k not in ('rgb_jpeg','crops')}
    def receive_map(msg):
        count('map');latest['map']=msg
        data=msg.data
        report['samples']['map']={'frame':msg.header.frame_id,'width':msg.info.width,'height':msg.info.height,
            'resolution':msg.info.resolution,'known':sum(v>=0 for v in data),'free':sum(v==0 for v in data),
            'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9}
    def receive_scan(msg):
        count('scan')
        report['samples']['scan']={'frame':msg.header.frame_id,'rays':len(msg.ranges),
                                   'finite':sum(math.isfinite(x) for x in msg.ranges)}
    def receive_pose(msg):
        count('slam_pose')
        report['samples']['slam_pose']={'frame':msg.header.frame_id,
            'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            'xy':[msg.pose.pose.position.x,msg.pose.pose.position.y]}
    try:
        report['can_before']=can_state()
        report['preexisting_can_up']=any('UP' in d.get('flags',[]) for d in report['can_before'])
        report['allow_preexisting_can_up']=args.allow_preexisting_can_up
        if report['preexisting_can_up'] and not args.allow_preexisting_can_up:
            raise RuntimeError('CAN is already UP; default audit refuses it. No interfaces were changed')
        report['drivers_before']=running_chassis_drivers()
        if report['drivers_before']:
            raise RuntimeError('Running chassis driver detected; no processes or interfaces were changed')
        rclpy.init();node=rclpy.create_node('online_slam_stationary_audit')
        pump(2.)
        if len(node.get_node_names_and_namespaces())!=1:
            raise RuntimeError('Isolated ROS domain179 is already in use')
        report['checks']['no_initial_cmd_vel_publishers']=node.count_publishers('/cmd_vel')==0
        if not report['checks']['no_initial_cmd_vel_publishers']:
            raise RuntimeError('Final cmd_vel publisher exists in test domain')
        console_log=(out/'console_monitor.log').open('w')
        console_command=['/usr/bin/python3','-m','robot_console.backend','--socket',str(console_socket),
                         '--export-dir',str(out/'console_exports')]
        report['console_monitor_command']=console_command
        console_proc=subprocess.Popen(console_command,stdout=console_log,stderr=subprocess.STDOUT,start_new_session=True)
        until(console_socket.exists,10.,'read-only console socket startup timeout')
        initial_console=console_snapshot(console_socket)
        if not initial_console.get('monitor_only'):
            raise RuntimeError('Console audit process unexpectedly has control enabled')
        tf=Buffer();listener=TransformListener(tf,node)
        map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        subscriptions=[node.create_subscription(OccupancyGrid,'/map',receive_map,map_qos),
            node.create_subscription(LaserScan,'/scan',receive_scan,qos_profile_sensor_data),
            node.create_subscription(PoseWithCovarianceStamped,'/slam_toolbox/pose',receive_pose,10)]
        for name, topic in [('safety','/safety/status'),('mission','/mission/state'),
                            ('candidates','/perception/candidates'),('estimates','/target/estimates'),
                            ('events','/perception/candidate_event'),('confirmation','/perception/confirmation')]:
            subscriptions.append(node.create_subscription(String,topic,lambda msg,n=name:receive_json(n,msg),10))
        report['cmd_messages']={}
        for topic in ['/cmd_vel','/cmd_vel_search','/cmd_vel_nav','/cmd_vel_nav_smoothed','/cmd_vel_guard_preview']:
            report['cmd_messages'][topic]={'total':0,'nonzero':0}
            def receive_cmd(msg,key=topic):
                counters=report['cmd_messages'][key];counters['total']+=1
                if any(abs(x)>1e-12 for x in [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]):
                    counters['nonzero']+=1
            subscriptions.append(node.create_subscription(Twist,topic,receive_cmd,10))
        # No fixture is emitted until phase1 confirms the missing-odom interlock.
        odom_pub=node.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        broadcaster=TransformBroadcaster(node)
        def zero_odometry_fixture():
            if not fixture['enabled']:
                return
            msg=Odometry();msg.header.stamp=node.get_clock().now().to_msg()
            msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.
            odom_pub.publish(msg)
            count('fixture_odom')
            tr=TransformStamped();tr.header=msg.header;tr.child_frame_id='base_link';tr.transform.rotation.w=1.
            broadcaster.sendTransform(tr)
        timer=node.create_timer(.05,zero_odometry_fixture)
        share=Path(get_package_share_directory('robot_bringup'))
        source=share/'config/nav2.yaml';config=yaml.safe_load(source.read_text())
        managed=['planner_server','controller_server','behavior_server','bt_navigator','velocity_smoother']
        for name in managed:
            config[name]['ros__parameters']['bond_heartbeat_period']=0.0
        params=out/'nav2_test_params.yaml';params.write_text(yaml.safe_dump(config,sort_keys=False))
        report['nav_config_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
        report['test_parameter_override']='bond_heartbeat_period=0 only, for manual lifecycle transitions'
        logfile=(out/'task_launch.log').open('w')
        command=['ros2','launch','robot_bringup','task.launch.py','start_sensors:=true',
                 'enable_navigation:=true','motion_enabled:=false','start_chassis:=false','autostart:=false',
                 'nav_autostart:=false','enable_perception:='+args.enable_perception,'nav_params_file:='+str(params)]
        report['launch_command']=command
        proc=subprocess.Popen(command,stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
        until(lambda:report['counts'].get('scan',0)>=10 and 'safety' in latest,25.,'live sensors/guard startup timeout')
        pump(1.)
        report['phase1_without_odom']={'safety':latest['safety'],'map_count':report['counts'].get('map',0)}
        report['checks']['missing_odom_blocks']=not latest['safety'].get('ready',True)
        report['checks']['no_map_without_odom']=report['counts'].get('map',0)==0
        if not report['checks']['missing_odom_blocks']:
            raise RuntimeError('Safety unexpectedly ready without odometry')
        report['fixture_started_seconds']=time.monotonic()-started
        fixture['enabled']=True
        until(lambda:report['counts'].get('map',0)>=2 and report['counts'].get('slam_pose',0)>0
              and latest.get('safety',{}).get('ready',False),45.,'SLAM map/pose/safety readiness timeout')
        report['healthy_disarmed_sample']=dict(latest['safety'])
        if args.enable_perception=='true':
            until(lambda:latest.get('candidates',{}).get('ready',False),30.,'YOLO warmup timeout')
        for name in managed:transition(name,1,'inactive')
        for name in ['planner_server','bt_navigator']:transition(name,3,'active')
        pump(2.)
        transform=tf.lookup_transform('map','base_link',Time()).transform
        robot=(transform.translation.x,transform.translation.y,yaw_of(transform.rotation))
        occupancy=latest['map'];origin=occupancy.info.origin
        grid=Grid(occupancy.info.width,occupancy.info.height,occupancy.info.resolution,
                  (origin.position.x,origin.position.y,yaw_of(origin.orientation)),list(occupancy.data))
        # Only request a nearby already-known-free path. This is not a driving goal.
        destinations=[]
        for distance in [.25,.15,.10]:
            for angle in [0.,math.pi/2,-math.pi/2,math.pi]:
                x=robot[0]+distance*math.cos(robot[2]+angle);y=robot[1]+distance*math.sin(robot[2]+angle)
                if grid.path_free([robot[:2],(x,y)]):
                    destinations.append((x,y,robot[2]));break
            if destinations:break
        if not destinations:
            raise RuntimeError('No nearby full-footprint known-free path for stationary planning check')
        action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        until(action.server_is_ready,6.,'planner action unavailable')
        def pose(values):
            p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
            p.pose.position.x,p.pose.position.y=values[:2]
            p.pose.orientation.z,p.pose.orientation.w=math.sin(values[2]/2),math.cos(values[2]/2)
            return p
        request=ComputePathToPose.Goal();request.start=pose(robot);request.goal=pose(destinations[0])
        request.use_start=True;request.planner_id='GridBased'
        handle=future_result(action.send_goal_async(request))
        if not handle.accepted:raise RuntimeError('known-free planning request rejected')
        result=future_result(handle.get_result_async(),20.)
        points=[(p.pose.position.x,p.pose.position.y) for p in result.result.path.poses]
        report['planning']={'status':result.status,'path_points':len(points),'start':robot,'goal':destinations[0],
                            'path_in_known_free_space':grid.path_free(points)}
        report['checks']['known_free_planning']=result.status==4 and bool(points) and grid.path_free(points)
        report['controller_final_states']={name:state(name) for name in ['controller_server','behavior_server','velocity_smoother']}
        report['checks']['controllers_never_activated']=all(v=='inactive' for v in report['controller_final_states'].values())
        client=node.create_client(Trigger,'/mission/start')
        until(client.service_is_ready,3.,'mission start service unavailable')
        answer=future_result(client.call_async(Trigger.Request()),3.)
        node.destroy_client(client)
        report['checks']['disarmed_start_rejected']=not answer.success
        report['node_names']=sorted(name for name,namespace in node.get_node_names_and_namespaces())
        report['map_publishers']=[info.node_name for info in node.get_publishers_info_by_topic('/map')]
        report['amcl_pose_publishers']=node.count_publishers('/amcl_pose')
        report['cmd_vel_publishers']=node.count_publishers('/cmd_vel')
        report['checks']['slam_owns_map']=report['map_publishers']==['slam_toolbox']
        report['checks']['no_amcl_or_map_server']=not {'amcl','map_server'} & set(report['node_names']) and report['amcl_pose_publishers']==0
        report['checks']['no_final_velocity_publisher']=report['cmd_vel_publishers']==0
        report['checks']['live_map_and_accepted_pose']=report['counts'].get('map',0)>=2 and report['counts'].get('slam_pose',0)>0
        report['checks']['disarmed_slam_ready']=report['healthy_disarmed_sample'].get('ready',False)
        report['checks']['zero_candidate_api_events']=report['counts'].get('events',0)==0 and report['counts'].get('confirmation',0)==0
        report['checks']['perception_ready']=args.enable_perception=='false' or latest.get('candidates',{}).get('ready',False)
        # Read-only socket snapshots contain actual ROS images/scan/map; no action is requested.
        snapshot={}
        end=time.monotonic()+5.
        while time.monotonic()<end:
            pump(.1)
            snapshot=console_snapshot(console_socket)
            sensors=snapshot.get('sensors',{})
            if all(sensors.get(k,{}).get('state')=='normal' for k in ('rgb','depth','lidar','map')):
                break
        (out/'console_snapshot.json').write_text(json.dumps(snapshot,indent=2))
        report['console_snapshot']={'path':str(out/'console_snapshot.json'),
            'monitor_only':snapshot.get('monitor_only'),'sensors':snapshot.get('sensors',{}),
            'chassis':snapshot.get('chassis',{}),'can':snapshot.get('can',{}),'controls':snapshot.get('controls',{}),
            'fixture_warning':'Displayed odometry/TF are TEST-ONLY fixtures; no actual chassis driver is running.'}
        images={}
        for kind in ('rgb','depth'):
            frame=snapshot.get('frames',{}).get(kind,{})
            if frame.get('jpeg'):
                blob=base64.b64decode(frame['jpeg'],validate=True)
                image=out/('console_'+kind+'.jpg');image.write_bytes(blob)
                images[kind]={'path':str(image),'bytes':len(blob),'sha256':hashlib.sha256(blob).hexdigest(),
                              'stamp':frame.get('stamp'),'width':frame.get('width'),'height':frame.get('height')}
        report['console_snapshot']['images']=images
        report['checks']['console_live_views']=all(snapshot.get('sensors',{}).get(k,{}).get('state')=='normal'
            for k in ('rgb','depth','lidar','map')) and all(k in images for k in ('rgb','depth'))
        report['checks']['console_monitor_only']=snapshot.get('monitor_only') is True
        chassis=snapshot.get('chassis',{})
        report['checks']['console_waits_for_actual_chassis']=chassis.get('driver_running') is False and chassis.get('communication_ok') is False
        report['checks']['console_never_grants_motion']=all(not snapshot.get('controls',{}).get(k,True)
            for k in ('can_enable','can_disable','driver_start','driver_stop','arm','mission_start'))
        # Render the actual Qt UI against the monitor socket. Keep spinning the
        # fixture timer during screenshot capture; never issue a console action.
        ui_image=out/'console_live.png'
        ui_log=(out/'console_ui.log').open('w')
        ui_command=['/usr/bin/python3','-m','robot_console.ui','--socket',str(console_socket),
                    '--screenshot',str(ui_image),'--quit-after','3']
        ui_env=dict(os.environ,QT_QPA_PLATFORM='offscreen')
        fixture_before=report['counts'].get('fixture_odom',0)
        ui_proc=subprocess.Popen(ui_command,env=ui_env,stdout=ui_log,stderr=subprocess.STDOUT,start_new_session=True)
        until(lambda:ui_proc.poll() is not None,12.,'read-only Qt console render timeout')
        image_bytes=ui_image.read_bytes() if ui_image.is_file() else b''
        is_png=len(image_bytes)>=24 and image_bytes[:8]==b'\x89PNG\r\n\x1a\n'
        report['console_render']={'command':ui_command,'platform':'offscreen','path':str(ui_image),
            'exit_code':ui_proc.returncode,'exists':ui_image.is_file(),'bytes':len(image_bytes),
            'sha256':hashlib.sha256(image_bytes).hexdigest() if image_bytes else None,
            'dimensions':[int.from_bytes(image_bytes[16:20],'big'),int.from_bytes(image_bytes[20:24],'big')] if is_png else None,
            'fixture_messages_during_render':report['counts'].get('fixture_odom',0)-fixture_before,
            'scope':'Actual Qt display of live RGB/depth, scan and SLAM map. Odometry/odom TF are TEST-ONLY fixtures; this is not real chassis navigation.'}
        report['checks']['console_actual_qt_render']=ui_proc.returncode==0 and is_png
        report['checks']['fixture_continued_during_ui_render']=report['console_render']['fixture_messages_during_render']>=10
        # Stop BOTH the test odometry and its dynamic TF; no fake refresh remains.
        fixture['enabled']=False
        until(lambda:not latest.get('safety',{}).get('ready',True),3.,'guard failed to detect odometry/TF loss')
        pump(.4)
        report['fault_after_fixture_stopped']=dict(latest['safety'])
        report['checks']['lost_odom_tf_revokes_readiness']=not latest['safety'].get('ready',True)
        report['checks']['all_command_samples_zero']=not any(x['nonzero'] for x in report['cmd_messages'].values())
        report['passed']=all(report['checks'].values())
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
    finally:
        fixture['enabled']=False
        if ui_proc is not None:
            try:
                if ui_proc.poll() is None:ui_proc.send_signal(signal.SIGINT)
                ui_proc.wait(timeout=4.)
            except subprocess.TimeoutExpired:
                os.killpg(ui_proc.pid,signal.SIGTERM)
                try:ui_proc.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(ui_proc.pid,signal.SIGKILL);ui_proc.wait(timeout=2.)
            report['console_ui_exit_code']=ui_proc.returncode
        if ui_log is not None:ui_log.close()
        if console_proc is not None:
            try:
                if console_proc.poll() is None:console_proc.send_signal(signal.SIGINT)
                console_proc.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(console_proc.pid,signal.SIGTERM)
                try:console_proc.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    os.killpg(console_proc.pid,signal.SIGKILL);console_proc.wait(timeout=3.)
            report['console_exit_code']=console_proc.returncode
        if console_log is not None:console_log.close()
        if console_socket.exists():console_socket.unlink()
        if proc is not None:
            try:
                if proc.poll() is None:proc.send_signal(signal.SIGINT)
                proc.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3.)
            report['launch_exit_code']=proc.returncode
        if logfile is not None:logfile.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        try:
            report['can_after']=can_state()
            report['drivers_after']=running_chassis_drivers()
            report['checks']['no_chassis_driver_after']=not report['drivers_after']
            report['checks']['can_configuration_unchanged']=can_configuration(report.get('can_before',[]))==can_configuration(report['can_after'])
            report['passed']=report['passed'] and report['checks']['can_configuration_unchanged'] and report['checks']['no_chassis_driver_after']
        except Exception as error:
            report['passed']=False;report['cleanup_error']=str(error)
        report['elapsed_seconds']=time.monotonic()-started
        (out/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({'passed':report['passed'],'report':str(out/'report.json'),'error':report.get('error','')},indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
