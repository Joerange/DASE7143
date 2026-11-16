"""Synthetic credentials and fake transports only; never connects to hardware/API."""
import contextlib
import importlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
start = importlib.import_module('start_robot')
stop = importlib.import_module('stop_robot')


class CredentialTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(); self.addCleanup(self.temporary.cleanup)
        self.home = Path(self.temporary.name)
        self.config = self.home/'ros2_ws/src/simple_vlm/config.yaml'
        self.config.parent.mkdir(parents=True)
        self.key = 'synthetic-credential-for-unit-testing'

    def test_saved_key_and_regional_endpoint_loaded_without_output(self):
        self.config.write_text('api_key: '+self.key+'\nbase_url: https://cn-hongkong.dashscope.aliyuncs.com/compatible-mode/v1\n')
        output = io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            env, source = start.credential_environment({}, self.home)
        self.assertEqual(env['DASHSCOPE_API_KEY'], self.key)
        self.assertIn('cn-hongkong.dashscope.aliyuncs.com', env['QWEN_BASE_URL'])
        self.assertEqual(source, str(self.config)); self.assertEqual(output.getvalue(), '')

    def test_explicit_environment_takes_precedence(self):
        env, source = start.credential_environment({'QWEN_API_KEY': self.key}, self.home)
        self.assertEqual(env['QWEN_API_KEY'], self.key); self.assertEqual(source, 'environment')

    def test_missing_configuration_fails_without_prompt(self):
        with patch('builtins.input', side_effect=AssertionError('must not prompt')):
            with self.assertRaisesRegex(RuntimeError, '不会要求重复输入'):
                start.credential_environment({}, self.home)

    def test_malformed_yaml_never_exposes_credential_line(self):
        self.config.write_text('api_key: ['+self.key+'\n')
        with self.assertRaises(RuntimeError) as caught:
            start.credential_environment({}, self.home)
        self.assertNotIn(self.key, str(caught.exception))
        self.assertTrue(caught.exception.__suppress_context__)

    def test_yaml_is_never_executed(self):
        self.config.write_text('api_key: !!python/object/apply:os.system ["echo '+self.key+'"]\n')
        with patch('os.system') as execute:
            with self.assertRaises(RuntimeError):start.credential_environment({}, self.home)
        execute.assert_not_called()

    def test_wrong_endpoint_is_rejected_without_values_in_error(self):
        self.config.write_text('api_key: '+self.key+'\nbase_url: https://untrusted.invalid/'+self.key+'\n')
        with self.assertRaises(RuntimeError) as caught:start.credential_environment({}, self.home)
        self.assertNotIn(self.key, str(caught.exception))

    def test_explicit_endpoint_is_preserved(self):
        self.config.write_text('api_key: '+self.key+'\nbase_url: https://cn-hongkong.dashscope.aliyuncs.com/compatible-mode/v1\n')
        env, _ = start.credential_environment({'QWEN_BASE_URL':'https://dashscope.aliyuncs.com/compatible-mode/v1'}, self.home)
        self.assertEqual(env['QWEN_BASE_URL'], 'https://dashscope.aliyuncs.com/compatible-mode/v1')

    def test_backup_config_supported(self):
        backup = self.home/'robot/ros2_ws/src/simple_vlm/config.yaml'; backup.parent.mkdir(parents=True)
        backup.write_text('api_key: '+self.key+'\n')
        env, source = start.credential_environment({}, self.home)
        self.assertEqual(source, str(backup));self.assertEqual(env['DASHSCOPE_API_KEY'], self.key)


class StopTests(unittest.TestCase):
    def setUp(self):
        self.values = dict(
            safety=dict(_received=10., stamp=100., authorized=False, output_zero=True),
            mission=dict(_received=10., stamp=100., active=False, source='none', navigation_idle=True),
            chassis=dict(_received=10., stamp=100., driver_running=True, communication_ok=True,
                velocity=dict(valid=True, age=.01, linear_x=0., angular_z=0.),
                can_feedback=dict(live=True, stopped=True, age=.01)))

    def test_fresh_actual_stop_evidence_passes(self):
        self.assertTrue(stop.standstill(self.values, 10.1, 100.1))

    def test_freshness_moving_nan_and_missing_feedback_fail(self):
        self.assertFalse(stop.standstill(self.values, 11., 101.))
        self.values['chassis']['velocity']['linear_x'] = .2
        self.assertFalse(stop.standstill(self.values, 10.1, 100.1))
        self.values['chassis']['velocity']['linear_x'] = float('nan')
        self.assertFalse(stop.standstill(self.values, 10.1, 100.1))
        self.values['chassis']['velocity']['linear_x'] = 0.
        self.values['chassis']['can_feedback']['live'] = False
        self.assertFalse(stop.standstill(self.values, 10.1, 100.1))

    def test_guard_and_mission_conditions_both_required(self):
        self.values['safety']['authorized'] = True
        self.assertFalse(stop.standstill(self.values, 10.1, 100.1))
        self.values['safety']['authorized'] = False
        self.values['mission']['navigation_idle'] = False
        self.assertFalse(stop.standstill(self.values, 10.1, 100.1))

    def test_shutdown_forbidden_while_driver_running(self):
        self.assertFalse(stop.shutdown_permitted(self.values, 10.1, 100.1))
        self.values['chassis']['driver_running'] = False
        self.assertTrue(stop.shutdown_permitted(self.values, 10.1, 100.1))

    def test_unavailable_guard_does_not_block_mission_stop(self):
        clients = {'disarm': Mock(), 'stop': Mock()}; futures = {}
        clients['disarm'].service_is_ready.return_value = False
        clients['stop'].service_is_ready.return_value = True
        stop.dispatch_stops(clients, futures, {'disarm': object(), 'stop': object()})
        clients['stop'].call_async.assert_called_once(); clients['disarm'].call_async.assert_not_called()

    def test_unavailable_mission_does_not_block_disarm(self):
        clients = {'disarm': Mock(), 'stop': Mock()}; futures = {}
        clients['disarm'].service_is_ready.return_value = True
        clients['stop'].service_is_ready.return_value = False
        stop.dispatch_stops(clients, futures, {'disarm': object(), 'stop': object()})
        clients['disarm'].call_async.assert_called_once(); clients['stop'].call_async.assert_not_called()

    def test_shutdown_never_signals_reused_or_foreign_pid(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); registry = root/'state.json'
            registry.write_text(json.dumps(dict(workspace=str(root), task=dict(pid=22))))
            with patch.object(stop, 'registered_process', return_value=False), patch.object(stop.os, 'killpg') as signal_group:
                with self.assertRaises(RuntimeError):stop.shutdown_owned_stack(root, registry)
            signal_group.assert_not_called()

    def test_shutdown_rechecks_no_driver_before_signal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); registry = root/'state.json'
            registry.write_text(json.dumps(dict(workspace=str(root), task=dict(pid=22))))
            with patch.object(stop, 'registered_process', return_value=True), patch.object(stop.os, 'getpgid', return_value=22), \
                    patch.object(stop, 'no_chassis_process', return_value=False), patch.object(stop.os, 'killpg') as signal_group:
                with self.assertRaises(RuntimeError):stop.shutdown_owned_stack(root, registry)
            signal_group.assert_not_called()


if __name__ == '__main__':unittest.main()
