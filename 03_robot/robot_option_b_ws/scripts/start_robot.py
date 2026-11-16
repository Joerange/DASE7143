#!/usr/bin/env python3
"""Desktop session: automatic preparation, manual mission start, owned cleanup."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time
import signal
from urllib.parse import urlsplit


SERVICES = ('robot-console-can.service', 'robot-console.service')
TASK_ARGUMENTS = ('launch', 'robot_bringup', 'task.launch.py',
                  'start_sensors:=true', 'enable_navigation:=true',
                  'allow_runtime_control:=true', 'motion_enabled:=false',
                  'start_chassis:=false', 'autostart:=false', 'rviz:=false')
PROCESS_NAMES = {'realsense2_camera_node', 'lslidar_driver_node', 'lidar_filter_node',
                 'pointcloud_to_laserscan_node', 'async_slam_toolbox_node',
                 'sync_slam_toolbox_node', 'mission_node', 'command_guard_node',
                 'perception_node', 'vlm_confirm_node', 'view_check_node',
                 'planner_server', 'controller_server', 'bt_navigator',
                 'behavior_server', 'velocity_smoother'}


def credential_environment(environment, home=None):
    """Read existing credentials without prompting, sourcing shell or logging values."""
    result = dict(environment)
    if result.get('DASHSCOPE_API_KEY') or result.get('QWEN_API_KEY'):
        return result, 'environment'
    home = Path.home() if home is None else Path(home)
    explicit = result.get('ROBOT_API_CONFIG')
    candidates = ([Path(explicit).expanduser()] if explicit else
                  [home/'ros2_ws/src/simple_vlm/config.yaml', home/'robot/ros2_ws/src/simple_vlm/config.yaml'])
    import yaml
    for config in candidates:
        try:
            if not config.is_file():
                continue
            if config.stat().st_size > 262144:
                raise RuntimeError('API 配置文件过大；未输出其内容。')
            data = yaml.safe_load(config.read_text())
        except (OSError, UnicodeError, yaml.YAMLError):
            # YAML exceptions include source lines: never propagate the original
            # exception, which may contain an API key or the entire config.
            raise RuntimeError('API 配置无法安全读取或解析；请检查配置文件，未输出其内容。') from None
        key = data.get('api_key') if isinstance(data, dict) else None
        if (not isinstance(key, str) or not key.strip() or any(c.isspace() for c in key.strip())
                or re.search(r'your|placeholder|填[写入]|替换|^x+$|^\$\{|API_KEY', key, re.I)):
            continue
        result['DASHSCOPE_API_KEY'] = key.strip()
        # The existing key is regional; retain its configured Hong Kong endpoint.
        if not result.get('QWEN_BASE_URL') and data.get('base_url'):
            address = data['base_url']
            try:
                url = urlsplit(address) if isinstance(address, str) else None
                valid = bool(url and url.scheme == 'https' and url.hostname and not url.username
                             and not url.password and not url.query and not url.fragment
                             and (url.hostname == 'dashscope.aliyuncs.com'
                                  or url.hostname.endswith('.dashscope.aliyuncs.com')))
            except ValueError:
                valid = False
            if not valid:
                raise RuntimeError('旧配置中的模型地址不是受支持的 HTTPS DashScope 地址；未输出地址或密钥。')
            result['QWEN_BASE_URL'] = address.rstrip('/')
        return result, str(config)
    raise RuntimeError('未找到已有 API 凭据配置；不会要求重复输入，也不会启动缺少确认凭据的任务栈。')


def command(argv, **kwargs):
    return subprocess.run(argv, check=kwargs.pop('check', True), text=True, **kwargs)


def process_identity(pid):
    """Linux PID start ticks prevent a stale registry from matching a reused PID."""
    try:
        base = Path('/proc') / str(int(pid))
        fields = (base / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        argv = (base / 'cmdline').read_bytes().split(b'\0')
        return {'pid': int(pid), 'start_ticks': fields[19],
                'argv': [part.decode(errors='replace') for part in argv if part]}
    except (OSError, ValueError, IndexError, TypeError):
        return None


def registered_process(record):
    if not isinstance(record, dict):
        return False
    actual = process_identity(record.get('pid'))
    return bool(actual and actual == record)


def process_conflicts():
    conflicts = []
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        actual = process_identity(item.name)
        if not actual:
            continue
        # Inspect only executable/script words, not arbitrary parameter strings.
        if any(Path(word).name in PROCESS_NAMES for word in actual['argv'][:2]):
            conflicts.append(actual['pid'])
        elif any(actual['argv'][i:i+3] == ['launch', 'robot_bringup', 'task.launch.py']
                 for i in range(min(4, len(actual['argv'])))):
            conflicts.append(actual['pid'])
    return conflicts


def graph_conflicts():
    """Discover real publishers in domain180 without relying on a CLI daemon."""
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node('robot_oneclick_preflight_' + str(os.getpid()))
    try:
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        topics = ('/cmd_vel', '/mission/state', '/safety/status', '/map', '/scan',
                  '/camera/camera/color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw')
        return [topic for topic in topics if node.count_publishers(topic)]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def control_request(socket_path, payload):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(2.)
        client.connect(str(socket_path))
        client.sendall((json.dumps(payload)+'\n').encode())
        chunks, size = [], 0
        while size < 32 * 1024 * 1024:
            chunk = client.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            size += len(chunk)
            if b'\n' in chunk:
                break
        return json.loads(b''.join(chunks).split(b'\n', 1)[0])


def snapshot(socket_path):
    return control_request(socket_path, {'op':'status'})


def session_status(runtime, phase, message):
    path=runtime/'robot-session.json'
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(phase=phase,message=message,stamp=time.time()),ensure_ascii=False))
    temporary.replace(path)


def control_action(endpoint, action, timeout=25., canceled=lambda:False):
    answer=control_request(endpoint,dict(op='action',action=action))
    if not answer.get('ok'):
        raise RuntimeError(answer.get('error','Control service rejected request'))
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if canceled():raise InterruptedError('正在关闭控制台')
        data=snapshot(endpoint)
        result=data.get('operations',{}).get(answer['operation_id'],{})
        if result.get('phase')=='finished':
            if result.get('result')!='success':raise RuntimeError(action+': '+result.get('message','结果未知'))
            return data
        time.sleep(.1)
    raise RuntimeError(action+' 操作超时；以实际回读为准')


def ensure_services(workspace):
    states = []
    for service in SERVICES:
        result = command(['/usr/bin/systemctl', 'show', '--property=LoadState', '--value', service],
                         capture_output=True, timeout=5, check=False)
        state = result.stdout.strip()
        # Missing units can be installed. A masked/error/bus-unavailable unit
        # is an operator/system state, not permission to overwrite its setup.
        if state not in ('loaded', 'not-found') or (result.returncode and state != 'not-found'):
            raise RuntimeError('无法启动控制服务 ' + service + '，LoadState=' + (state or 'unknown'))
        states.append(state)
    if 'not-found' in states:
        installer = str(workspace / 'console_service/install_services.sh')
        command([installer, '--check'])
        print('首次部署本地控制服务，sudo 可能要求密码；不启用 CAN 或底盘。', flush=True)
        command(['sudo', installer, '--install'])
    # start is idempotent; never restart an already-running control lease.
    command(['sudo', '/usr/bin/systemctl', 'start', *SERVICES])
    for service in SERVICES:
        command(['/usr/bin/systemctl', 'is-active', '--quiet', service], timeout=5)


class Launcher:
    def __init__(self, workspace, runtime):
        self.workspace, self.runtime = workspace, runtime
        self.registry = runtime / 'robot-task-start.json'
        self.control_socket = runtime / 'robot-console-control.sock'
        self.state = {'workspace': str(workspace)}
        self.log_dir = workspace / 'console_exports'
        self.task_child = None

    def load(self):
        if self.registry.exists():
            state = json.loads(self.registry.read_text())
            if state.get('workspace') != str(self.workspace):
                raise RuntimeError('已有其他工作区的启动记录；保留现有进程，请先核对。')
            self.state = state

    def save(self):
        temporary = self.registry.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.state, indent=2))
        temporary.chmod(0o600)
        temporary.replace(self.registry)

    def spawn(self, argv, role, env=None):
        logfile = self.log_dir / (role + '_' + time.strftime('%Y%m%d_%H%M%S') + '.log')
        with logfile.open('ab') as output:
            child = subprocess.Popen(argv, cwd=str(self.workspace), env=env,
                                     stdin=subprocess.DEVNULL, stdout=output,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        # Popen has completed exec; capture both argv and process start identity.
        identity = process_identity(child.pid)
        if identity is None:
            raise RuntimeError('进程启动失败，请查看 ' + str(logfile))
        self.state[role] = identity
        self.state[role + '_log'] = str(logfile)
        self.save()
        return child

    def wait_control(self, require_task=False):
        deadline = time.monotonic() + (45. if require_task else 20.)
        while time.monotonic() < deadline:
            if self.task_child and self.task_child.poll() is not None:
                raise RuntimeError('任务启动进程已退出，请查看 ' + self.state.get('task_log', '日志'))
            try:
                data = snapshot(self.control_socket)
                if data.get('ok') is True and data.get('monitor_only') is False:
                    if not require_task:
                        return data
                    guard, mission = data.get('safety', {}), data.get('mission', {})
                    feeds = data.get('sensors', {})
                    if (guard.get('runtime_control_capable') is True and guard.get('output_topic') == '/cmd_vel'
                            and mission.get('state') and all(feeds.get(k, {}).get('state') == 'normal'
                                                           for k in ('safety', 'mission'))):
                        return data
            except (OSError, ValueError, TypeError):
                pass
            time.sleep(.2)
        raise RuntimeError('等待受控后台/任务状态超时；已有进程保留，未自动重启。'
                           '检查 journalctl -u robot-console.service 和 ' + self.state.get('task_log', '后台日志'))

    def start(self, windowed=False):
        self.load()
        self.state['launcher']=process_identity(os.getpid())
        reused = registered_process(self.state.get('task'))
        if not reused:
            conflicts = process_conflicts()
            topics = graph_conflicts()
            if conflicts or topics:
                raise RuntimeError('检测到非本脚本管理的现有任务/传感器，拒绝重复启动。'
                                   ' PID=' + str(conflicts) + ' 话题=' + str(topics))
        task_env = dict(os.environ)
        if not reused:
            task_env, credential_source = credential_environment(task_env)
            print('已从既有配置加载云端凭据（不显示内容），区域地址随配置使用。', flush=True)
        else:
            print('复用现有任务进程；凭据配置变更需在停车并退出旧任务栈后重新启动才会生效。', flush=True)
        ensure_services(self.workspace)
        self.wait_control()
        self.log_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        if not reused:
            self.task_child = self.spawn([shutil.which('ros2'), *TASK_ARGUMENTS], 'task', task_env)
        state = self.wait_control(require_task=True)
        if not registered_process(self.state.get('ui')):
            ui = ['/usr/bin/python3', '-m', 'robot_console.ui', '--socket', str(self.control_socket), '--managed-session']
            if not windowed:
                ui.append('--fullscreen')
            viewer = self.spawn(ui, 'ui')
            time.sleep(.7)
            if viewer.poll() is not None:
                raise RuntimeError('控制台窗口启动失败；任务栈保留，请查看 ' + self.state['ui_log'])
        print('已' + ('复用' if reused else '启动') + '任务栈与控制台，ROS_DOMAIN_ID=180。', flush=True)
        print('当前授权=' + str(state.get('safety', {}).get('authorized')) +
              '；任务=' + str(state.get('mission', {}).get('state')) + '。', flush=True)
        print('自动准备 CAN、底盘和运动许可；仅“开始任务”需要手动点击。关闭窗口或 Ctrl+C 退出整套后台。', flush=True)
        print('任务日志：' + self.state.get('task_log', '未知'), flush=True)

    def closing(self):
        return ((self.runtime/'robot-session-stop').exists() or
                not registered_process(self.state.get('ui')))

    def prepare(self):
        state=snapshot(self.control_socket)
        session_status(self.runtime,'preparing','正在自动准备 CAN、底盘和运动许可；任务仍等待手动开始。')
        if state.get('mission',{}).get('active'):
            raise RuntimeError('已有任务活动，未重复执行自动准备。')
        if state.get('can',{}).get('state')!='UP':
            control_action(self.control_socket,'can_enable',canceled=self.closing)
        state=snapshot(self.control_socket)
        if not state.get('chassis',{}).get('driver_running'):
            control_action(self.control_socket,'driver_start',canceled=self.closing)
        deadline=time.monotonic()+180.
        stable=None
        while time.monotonic()<deadline:
            if self.closing():raise InterruptedError('正在关闭控制台')
            state=snapshot(self.control_socket)
            safety=state.get('safety',{});mission=state.get('mission',{});feeds=state.get('sensors',{})
            ready=(safety.get('ready') is True and mission.get('nav_ready') is True
                   and mission.get('active') is False and mission.get('navigation_idle') is True
                   and state.get('chassis',{}).get('communication_ok') is True
                   and all(feeds.get(k,{}).get('state')=='normal' for k in ('safety','mission')))
            stable=(stable if stable is not None else time.monotonic()) if ready else None
            if stable is not None and time.monotonic()-stable>=1.:
                if not safety.get('authorized'):
                    # One grant attempt per desktop session. A later revocation
                    # must not be hidden by an automatic rearm loop.
                    control_action(self.control_socket,'arm',canceled=self.closing)
                session_status(self.runtime,'ready','准备完成：请手动点击“开始任务”。')
                return
            session_status(self.runtime,'preparing','等待地图、定位和导航就绪：'+safety.get('reason','waiting'))
            time.sleep(.2)
        raise RuntimeError('自动准备等待超时：'+state.get('safety',{}).get('reason','状态未知'))

    def session(self, windowed=False):
        stop_file=self.runtime/'robot-session-stop'
        stop_file.unlink(missing_ok=True)
        session_status(self.runtime,'starting','正在启动任务栈与控制台；不会自动开始任务。')
        started=False
        try:
            self.start(windowed);started=True
            try:self.prepare()
            except InterruptedError:pass
            except (OSError,ValueError,RuntimeError) as error:
                session_status(self.runtime,'blocked',str(error))
                print('自动准备未完成：'+str(error),flush=True)
            while not self.closing():time.sleep(.2)
        except KeyboardInterrupt:
            print('收到 Ctrl+C，正在关闭整个会话。',flush=True)
        finally:
            # Also run after a partial startup: do not strand an enabled driver.
            if started or self.state.get('task') or self.state.get('ui'):
                session_status(self.runtime,'stopping','正在撤销运动许可、停止底盘并退出后台。')
                result=command(['/usr/bin/python3',str(self.workspace/'scripts/stop_robot.py'),'--all'],check=False)
                if result.returncode:
                    session_status(self.runtime,'shutdown_failed','退出未完成；安全节点保留。请查看启动终端中的具体原因。')
                    raise RuntimeError('后台退出未完成，未强制关闭仍需保持的安全进程。')


def main(argv=None):
    parser = argparse.ArgumentParser(description='自动准备 Jetson；任务手动开始，关闭窗口退出后台。')
    parser.add_argument('--windowed', action='store_true', help='窗口模式（默认全屏，F11可切换）')
    args = parser.parse_args(argv)
    workspace = Path(__file__).resolve().parents[1]
    if (sys.platform != 'linux' or os.getuid() != 1000 or
            workspace != Path('/home/dase-orinnx/robot_option_b_ws')):
        parser.error('此入口对应已配置的 Jetson 普通用户 dase-orinnx 与 ~/robot_option_b_ws。')
    if os.environ.get('ROS_DOMAIN_ID') != '180' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('请通过 scripts/start_robot.sh 加载正确环境。')
    if not shutil.which('ros2'):
        parser.error('缺少 ros2，请先构建并加载 ROS 环境。')
    runtime = Path('/run/user/1000')
    if not runtime.is_dir():
        parser.error('缺少用户运行目录，请从 Jetson 图形桌面登录后执行。')
    def terminate_session(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM,signal.SIGHUP):signal.signal(sig,terminate_session)
    try:
        with (runtime / 'robot-task-start.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('另一个一键启动正在进行，请等待完成。')
            Launcher(workspace, runtime).session(args.windowed)
        return 0
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as error:
        print('启动未完成：' + str(error), file=sys.stderr)
        print('任务未被自动开始；请根据上述错误核对退出结果。', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('退出过程被再次中断；可执行 ./scripts/stop_robot.sh --all 继续关闭后台。', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
