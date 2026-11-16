"""Exercise actual launcher orchestration with no ROS/systemd/hardware execution."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/start_robot.py'
SPEC = importlib.util.spec_from_file_location('oneclick_launcher', SCRIPT)
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.workspace, self.runtime = self.root / 'workspace', self.root / 'runtime'
        self.workspace.mkdir(); self.runtime.mkdir()
        self.app = launcher.Launcher(self.workspace, self.runtime)
        self.ready = {'ok': True, 'monitor_only': False,
                      'safety': {'runtime_control_capable': True, 'output_topic': '/cmd_vel', 'authorized': False},
                      'mission': {'state': 'INIT', 'active': False},
                      'sensors': {'safety': {'state': 'normal'}, 'mission': {'state': 'normal'}}}
        self.processes = {}
        self.next_pid = 400
        def popen(argv, **kwargs):
            self.next_pid += 1
            self.processes[self.next_pid] = {'pid': self.next_pid, 'start_ticks': str(self.next_pid*10), 'argv': list(argv)}
            return NS(pid=self.next_pid, poll=Mock(return_value=None))
        for name, replacement in [('process_identity', lambda pid: self.processes.get(pid)),
                                  ('process_conflicts', lambda: []), ('graph_conflicts', lambda: []),
                                  ('snapshot', Mock(return_value=self.ready)),
                                  ('ensure_services', Mock())]:
            p = patch.object(launcher, name, replacement); p.start(); self.addCleanup(p.stop)
        p = patch.object(launcher.subprocess, 'Popen', side_effect=popen)
        self.popen = p.start(); self.addCleanup(p.stop)
        p = patch.object(launcher.shutil, 'which', return_value='/opt/ros/humble/bin/ros2')
        p.start(); self.addCleanup(p.stop)
        p = patch.object(launcher.time, 'sleep'); p.start(); self.addCleanup(p.stop)
        p = patch.dict(os.environ, {'DASHSCOPE_API_KEY': 'unit-test-only-secret'})
        p.start(); self.addCleanup(p.stop)

    def test_fresh_start_services_full_stack_gui_but_never_motion_actions(self):
        self.app.start()
        launcher.ensure_services.assert_called_once_with(self.workspace)
        self.assertEqual(self.popen.call_count, 2)
        task, ui = self.popen.call_args_list
        self.assertEqual(task.args[0], ['/opt/ros/humble/bin/ros2', *launcher.TASK_ARGUMENTS])
        for arg in ('allow_runtime_control:=true', 'autostart:=false', 'start_chassis:=false',
                    'motion_enabled:=false', 'start_sensors:=true', 'enable_navigation:=true'):
            self.assertIn(arg, task.args[0])
        self.assertIn('--fullscreen', ui.args[0])
        self.assertIn(str(self.app.control_socket), ui.args[0])
        for call in (task, ui):
            self.assertTrue(call.kwargs['start_new_session'])
            self.assertEqual(call.kwargs['stdin'], launcher.subprocess.DEVNULL)
        self.assertNotIn('unit-test-only-secret', self.app.registry.read_text())
        self.assertEqual(task.kwargs['env']['DASHSCOPE_API_KEY'], 'unit-test-only-secret')

    def test_repeat_reuses_task_and_window_without_restarting_services(self):
        self.app.start()
        second = launcher.Launcher(self.workspace, self.runtime)
        second.start()
        self.assertEqual(self.popen.call_count, 2)
        self.assertEqual(len(self.processes), 2)

    def test_closed_window_reopens_without_relaunching_task(self):
        self.app.start()
        del self.processes[self.app.state['ui']['pid']]
        second = launcher.Launcher(self.workspace, self.runtime)
        second.start(windowed=True)
        self.assertEqual(self.popen.call_count, 3)
        last = self.popen.call_args.args[0]
        self.assertIn('robot_console.ui', last)
        self.assertNotIn('--fullscreen', last)

    def test_unmanaged_ros_stack_blocks_before_services_or_process_launch(self):
        with patch.object(launcher, 'graph_conflicts', return_value=['/scan']):
            with self.assertRaisesRegex(RuntimeError, '拒绝重复启动'):
                self.app.start()
        launcher.ensure_services.assert_not_called()
        self.popen.assert_not_called()

    def test_sensor_in_other_domain_also_blocks_duplicate(self):
        with patch.object(launcher, 'process_conflicts', return_value=[123]):
            with self.assertRaisesRegex(RuntimeError, '123'):
                self.app.start()
        self.popen.assert_not_called()

    def test_reused_pid_is_not_trusted(self):
        record = {'pid': 4, 'start_ticks': '10', 'argv': ['ros2']}
        self.processes[4] = dict(record, start_ticks='20')
        self.assertFalse(launcher.registered_process(record))

    def test_active_mission_is_not_automatically_stopped_or_restarted(self):
        self.app.start()
        self.ready['safety']['authorized'] = True
        self.ready['mission'].update(state='NAVIGATE', active=True)
        launcher.Launcher(self.workspace, self.runtime).start()
        self.assertEqual(self.popen.call_count, 2)
        # All backend requests flow through the read-only snapshot function.
        self.assertTrue(all(call.args == (self.app.control_socket,) for call in launcher.snapshot.call_args_list))

    def test_service_failure_does_not_launch_task_or_ui(self):
        launcher.ensure_services.side_effect = RuntimeError('service failed')
        with self.assertRaisesRegex(RuntimeError, 'service failed'):
            self.app.start()
        self.popen.assert_not_called()

    def test_startup_does_not_wait_for_map_before_manual_chassis_start(self):
        self.ready['safety'].update(ready=False, reason='odom_stale')
        self.ready['mission']['nav_ready'] = False
        self.ready['sensors'].update(map={'state': 'waiting'}, odom={'state': 'waiting'})
        self.assertEqual(self.app.wait_control(require_task=True), self.ready)

    def test_stale_guard_does_not_count_as_running_task(self):
        self.ready['sensors']['safety']['state'] = 'stale'
        with patch.object(launcher.time, 'monotonic', side_effect=[0., 1., 50.]):
            with self.assertRaisesRegex(RuntimeError, '超时'):
                self.app.wait_control(require_task=True)
        self.popen.assert_not_called()

    def test_readonly_backend_cannot_satisfy_control_startup(self):
        self.ready['monitor_only'] = True
        with patch.object(launcher.time, 'monotonic', side_effect=[0., 1., 50.]):
            with self.assertRaisesRegex(RuntimeError, '超时'):
                self.app.wait_control()

    def test_ui_failure_keeps_task_record_for_safe_reuse(self):
        spawn = self.app.spawn
        def fail_ui(argv, role, env=None):
            child = spawn(argv, role, env)
            if role == 'ui':
                child.poll.return_value = 1
            return child
        with patch.object(self.app, 'spawn', side_effect=fail_ui):
            with self.assertRaisesRegex(RuntimeError, '任务栈保留'):
                self.app.start()
        self.assertTrue(launcher.registered_process(json.loads(self.app.registry.read_text())['task']))


class ServiceTests(unittest.TestCase):
    def test_existing_python_perception_processes_are_detected(self):
        entries = [Path('/proc/123'), Path('/proc/456')]
        identities = {'123': {'pid': 123, 'argv': ['/usr/bin/python3', '/overlay/lib/robot_perception/perception_node']},
                      '456': {'pid': 456, 'argv': ['/usr/bin/python3', '/overlay/lib/robot_perception/vlm_confirm_node']}}
        with patch.object(launcher, 'Path', side_effect=lambda value: NS(iterdir=lambda:iter(entries)) if value == '/proc' else Path(value)), \
                patch.object(launcher, 'process_identity', side_effect=lambda pid: identities[pid]):
            self.assertEqual(launcher.process_conflicts(), [123, 456])

    def test_existing_services_started_without_restart_enable_or_installer(self):
        with patch.object(launcher, 'command', return_value=NS(stdout='loaded\n', returncode=0)) as run:
            launcher.ensure_services(Path('/workspace'))
        calls = [call.args[0] for call in run.call_args_list]
        self.assertIn(['sudo', '/usr/bin/systemctl', 'start', *launcher.SERVICES], calls)
        self.assertFalse(any('restart' in call or 'enable' in call or '--install' in call for call in calls))

    def test_missing_services_install_then_start_only_fixed_units(self):
        with patch.object(launcher, 'command', return_value=NS(stdout='not-found\n', returncode=1)) as run:
            launcher.ensure_services(Path('/workspace'))
        calls = [call.args[0] for call in run.call_args_list]
        self.assertIn(['sudo', '/workspace/console_service/install_services.sh', '--install'], calls)
        self.assertIn(['sudo', '/usr/bin/systemctl', 'start', *launcher.SERVICES], calls)
        self.assertFalse(any(any('can_enable' in arg or 'ip link' in arg or '/mission/start' in arg for arg in call) for call in calls))

    def test_masked_unit_is_not_overwritten_or_started(self):
        with patch.object(launcher, 'command', return_value=NS(stdout='masked\n', returncode=0)) as run:
            with self.assertRaisesRegex(RuntimeError, 'masked'):
                launcher.ensure_services(Path('/workspace'))
        self.assertEqual(run.call_count, 1)

    def test_systemd_bus_error_does_not_attempt_installation(self):
        with patch.object(launcher, 'command', return_value=NS(stdout='', returncode=1)) as run:
            with self.assertRaisesRegex(RuntimeError, 'unknown'):
                launcher.ensure_services(Path('/workspace'))
        self.assertEqual(run.call_count, 1)

    def test_console_connection_uses_snapshot_only(self):
        fake = Mock()
        fake.recv.return_value = b'{"ok":true,"monitor_only":false}\n'
        with patch.object(launcher.socket, 'socket') as factory:
            factory.return_value.__enter__.return_value = fake
            self.assertTrue(launcher.snapshot('/run/user/1000/robot-console-control.sock')['ok'])
        fake.sendall.assert_called_once_with(b'{"op":"snapshot"}\n')


if __name__ == '__main__':
    unittest.main()
