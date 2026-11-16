#!/usr/bin/env python3
"""Independent stop client. No camera, model, GUI or console socket is required.

Default: revoke permission + stop/cancel mission, then verify real standstill.
--shutdown: once disarmed/idle and no chassis driver exists, terminate ONLY the
task launch group owned by start_robot.py. Never kill a driver or cut CAN.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import subprocess
import fcntl

from start_robot import (process_conflicts, process_identity, registered_process,
                         snapshot, control_action, session_status)


def fresh(record, now, ros_now, limit=.8):
    try:
        return bool(record and 0 <= now-record['_received'] <= limit
                    and 0 <= ros_now-record['stamp'] <= limit)
    except (KeyError, TypeError):
        return False


def stopped_control(values, now, ros_now):
    safety, mission = values.get('safety', {}), values.get('mission', {})
    return bool(fresh(safety, now, ros_now) and safety.get('authorized') is False
                and safety.get('output_zero') is True and fresh(mission, now, ros_now)
                and mission.get('active') is False and mission.get('source') == 'none'
                and mission.get('navigation_idle') is True)


def standstill(values, now, ros_now):
    c = values.get('chassis', {})
    v, feedback = c.get('velocity', {}), c.get('can_feedback', {})
    if not isinstance(v, dict) or not isinstance(feedback, dict):
        return False
    speeds = [v.get('linear_x'), v.get('angular_z')]
    elapsed = max(now-c.get('_received', -math.inf), ros_now-c.get('stamp', -math.inf))
    return bool(stopped_control(values, now, ros_now) and fresh(c, now, ros_now, .3)
                and c.get('communication_ok') is True and v.get('valid') is True
                and isinstance(v.get('age'), (float, int)) and 0 <= v['age']+elapsed <= .3
                and all(isinstance(s, (float, int)) and math.isfinite(s) for s in speeds)
                and abs(speeds[0]) < .015 and abs(speeds[1]) < .025
                and feedback.get('live') is True and feedback.get('stopped') is True
                and isinstance(feedback.get('age'), (float, int)) and 0 <= feedback['age']+elapsed <= .3)


def shutdown_permitted(values, now, ros_now):
    chassis = values.get('chassis', {})
    return bool(stopped_control(values, now, ros_now) and fresh(chassis, now, ros_now)
                and chassis.get('driver_running') is False)


def dispatch_stops(clients, futures, requests):
    # An unavailable safety service must never prevent requesting mission stop,
    # and an unavailable mission service must not prevent disarming the guard.
    for name, client in clients.items():
        if name not in futures and client.service_is_ready():
            try:
                futures[name] = client.call_async(requests[name])
            except Exception:
                continue  # Retry within the bounded stop deadline.


def no_chassis_process():
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            actual = process_identity(entry.name)
            if actual and any(Path(word).name == 'bunker_base_node' for word in actual['argv'][:2]):
                return False
    return True


def shutdown_owned_stack(workspace, registry):
    if not registry.is_file():
        raise RuntimeError('无本脚本的任务栈记录，保留所有进程。')
    state = json.loads(registry.read_text())
    task = state.get('task')
    if state.get('workspace') == str(workspace) and not registered_process(task) and not process_conflicts():
        return
    if state.get('workspace') != str(workspace) or not registered_process(task):
        raise RuntimeError('任务栈身份不匹配或已退出，未向其他进程发信号。')
    pid = task['pid']
    if os.getpgid(pid) != pid:
        raise RuntimeError('任务进程组不符合启动记录，保留进程。')
    if not no_chassis_process():
        raise RuntimeError('发现底盘驱动进程，保留最终安全节点和任务栈。')
    os.killpg(pid, signal.SIGINT)
    end = time.monotonic() + 18.
    while time.monotonic() < end:
        if not registered_process(task) and not process_conflicts():
            return
        time.sleep(.2)
    raise RuntimeError('已请求退出，但仍有任务/传感器进程；未强制杀进程，请检查任务日志。')


def shutdown_all(workspace, runtime):
    """Owned driver first, final guard last; then stop only this app's services."""
    with (runtime/'robot-session-shutdown.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        session_status(runtime,'stopping','正在停止任务、底盘和整套后台；CAN配置保留。')
        registry=runtime/'robot-task-start.json'
        state=json.loads(registry.read_text()) if registry.exists() else {}
        endpoint=runtime/'robot-console-control.sock'
        try:
            data=snapshot(endpoint)
        except (OSError,ValueError):
            data=None
        if not no_chassis_process():
            if not data:
                raise RuntimeError('底盘驱动仍在运行但受控后台不可用；保留安全节点，未强行断链。')
            print('正在通过受控后台停止任务与底盘驱动…',flush=True)
            control_action(endpoint,'driver_stop',timeout=30.)
        if registered_process(state.get('task')):
            if main(['--shutdown','--timeout','20'])!=0:
                raise RuntimeError('任务栈退出条件未满足；保留控制服务。')
        elif process_conflicts():
            raise RuntimeError('存在未登记的任务进程；未向未知进程发信号。')
        if not no_chassis_process():
            raise RuntimeError('底盘进程仍存在；未关闭安全通信服务。')
        # A previous read-only viewer may have left its own monitor backend.
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            actual=process_identity(entry.name)
            if not actual:continue
            args=actual['argv']
            if ('-m' in args and 'robot_console.backend' in args and
                    '--control-enabled' not in args and str(runtime/'robot-console.sock') in args):
                os.kill(actual['pid'],signal.SIGINT)
        result=subprocess.run(['sudo','-n','/usr/bin/systemctl','stop',
            'robot-console.service','robot-console-can.service'],check=False)
        if result.returncode:
            raise RuntimeError('任务与底盘已退出，但系统控制服务停止失败；需要安装固定服务管理权限。')
        session_status(runtime,'closed','任务、底盘、传感器、SLAM、Nav2及控制后台已关闭。')
        ui=state.get('ui')
        if state.get('workspace')==str(workspace) and registered_process(ui):
            os.kill(ui['pid'],signal.SIGTERM)
        print('全部任务后台已关闭；CAN接口配置保留。',flush=True)
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shutdown', action='store_true', help='底盘驱动已手动停止后，安全退出本脚本启动的任务栈')
    parser.add_argument('--all', action='store_true', help='自动停止任务和底盘，退出任务栈、控制台及系统控制后台')
    parser.add_argument('--timeout', type=float, default=12., help='请求及停稳验证期限，默认12秒')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 1 <= args.timeout <= 60:
        parser.error('timeout must be 1..60 seconds')
    if args.all:
        workspace=Path(__file__).resolve().parents[1]
        runtime=Path('/run/user')/str(os.getuid())
        try:return shutdown_all(workspace,runtime)
        except (RuntimeError,OSError,ValueError) as error:
            session_status(runtime,'shutdown_failed',str(error))
            print('整套后台退出未完成：'+str(error),file=sys.stderr)
            return 2
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger

    rclpy.init(args=[])
    node = rclpy.create_node('robot_operator_stop_' + str(os.getpid()))
    values, futures, subscriptions = {}, {}, []
    def receive(name, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict) and data.get('schema') == 1 and isinstance(data.get('stamp'), (float, int)):
                values[name] = dict(data, _received=time.monotonic())
        except (ValueError, TypeError):
            pass
    for name, topic in [('safety', '/safety/status'), ('mission', '/mission/control_state'), ('chassis', '/chassis/health')]:
        subscriptions.append(node.create_subscription(String, topic, lambda msg, n=name: receive(n, msg), 1))
    clients = {'disarm': node.create_client(SetBool, '/safety/set_motion_enabled'),
               'stop': node.create_client(Trigger, '/mission/stop')}
    disarm = SetBool.Request(); disarm.data = False
    requests = {'disarm': disarm, 'stop': Trigger.Request()}
    print('正在独立请求撤销运动许可、停止任务和取消导航；不依赖图像或云端。', flush=True)
    end, since = time.monotonic() + args.timeout, None
    try:
        while time.monotonic() < end:
            dispatch_stops(clients, futures, requests)
            rclpy.spin_once(node, timeout_sec=.05)
            now, ros_now = time.monotonic(), node.get_clock().now().nanoseconds/1e9
            if args.shutdown and shutdown_permitted(values, now, ros_now):
                if node.count_publishers('/bunker_status') or not no_chassis_process():
                    continue
                workspace = Path(__file__).resolve().parents[1]
                shutdown_owned_stack(workspace, Path('/run/user')/str(os.getuid())/'robot-task-start.json')
                print('已撤销任务并退出本脚本管理的任务栈；底盘驱动未运行，CAN和控制服务保持原状。')
                return 0
            if not args.shutdown and standstill(values, now, ros_now):
                since = now if since is None else since
                if now-since >= .7:
                    print('已停稳：运动许可撤销、任务停止、导航空闲，实时里程计及CAN反馈持续静止0.7秒。')
                    return 0
            else:
                since = None
        for name in clients:
            future = futures.get(name)
            try:
                accepted = bool(future and future.done() and future.result().success)
            except Exception:
                accepted = False
            print(('撤销运动许可' if name == 'disarm' else '停止任务') + ('：服务已接受。' if accepted else '：接受结果未知。'))
        print('未确认' + ('安全退出条件；先在界面手动停止底盘驱动，再执行 --shutdown。' if args.shutdown else
                         '实际停稳；请检查控制台/实体急停。底盘未启动或反馈缺失时也不会冒报已停稳。'))
        print('未关闭CAN，未强制杀死任务或安全进程。')
        return 2
    except RuntimeError as error:
        print('任务栈退出未完成：' + str(error), file=sys.stderr)
        return 2
    except (OSError, ValueError):
        print('任务栈退出未完成；未强制杀进程，检查现有进程与启动日志。', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('停止验证被中断；请求可能已发送，实际停稳结果未知。', file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
