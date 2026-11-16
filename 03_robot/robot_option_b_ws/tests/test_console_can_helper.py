"""CAN helper command/authority tests. No networking, privilege or CAN operation."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]/'console_service'
spec=importlib.util.spec_from_file_location('console_can_helper',ROOT/'can_helper.py')
helper=importlib.util.module_from_spec(spec);sys.modules[spec.name]=helper;spec.loader.exec_module(helper)


class FakePlatform:
    def __init__(self):
        self.now=10.;self.up=False;self.bitrate=500000;self.pid=42;self.start='1234';self.calls=[]
        self.fail_enable=False;self.fail_down=False
    def run(self,argv):
        self.calls.append(list(argv))
        if argv[0]==helper.SYSTEMCTL:return str(self.pid)+'\n'
        if argv[:4]==[helper.IP,'-details','-json','link']:
            return json.dumps([{'ifname':'can0','link_type':'can','flags':['UP'] if self.up else [],
                'operstate':'UNKNOWN','linkinfo':{'info_kind':'can','info_data':{
                    'state':'ERROR-ACTIVE','bittiming':{'bitrate':self.bitrate}}}}])
        if argv==[helper.IP,'link','set','dev','can0','type','can','bitrate','500000']:
            self.bitrate=500000;return ''
        if argv==[helper.IP,'link','set','dev','can0','up']:
            if not self.fail_enable:self.up=True
            return ''
        if argv==[helper.IP,'link','set','dev','can0','down']:
            if not self.fail_down:self.up=False
            return ''
        raise AssertionError('Unexpected subprocess argv '+repr(argv))
    def mutations(self):return [x for x in self.calls if x[:3]==[helper.IP,'link','set']]


class CanHelperTests(unittest.TestCase):
    def setUp(self):
        self.p=FakePlatform()
        self.c=helper.CanController(helper.Config(),self.p.run,lambda:self.p.now,lambda pid:self.p.start)
        self.peer=(42,1000,1000)
    def call(self,op,**kw):return self.c.handle(dict(op=op,**kw),self.peer)

    def test_up_does_not_mean_driver_communication_or_motion_permission(self):
        self.p.up=True
        status=self.call('status')['can']
        self.assertTrue(status['admin_up'])
        self.assertEqual(status['chassis_communication'],'unverified')
        self.assertFalse(status['motion_authorized'])
        self.assertFalse(self.p.mutations())

    def test_enable_changes_only_fixed_can_configuration(self):
        result=self.call('enable')
        self.assertTrue(result['ok']);self.assertTrue(result['can']['admin_up'])
        self.assertEqual(self.p.mutations(),[[helper.IP,'link','set','dev','can0','type','can','bitrate','500000'],
                                            [helper.IP,'link','set','dev','can0','up']])
        self.assertFalse(result['can']['motion_authorized'])

    def test_enabled_matching_interface_is_idempotent(self):
        self.p.up=True
        self.assertFalse(self.call('enable')['changed'])
        self.assertFalse(self.p.mutations())

    def test_up_wrong_bitrate_never_cycles_interface(self):
        self.p.up=True;self.p.bitrate=250000
        with self.assertRaises(helper.Rejected):self.call('enable')
        self.assertFalse(self.p.mutations())

    def test_enable_must_read_back_up_before_reporting_success(self):
        self.p.fail_enable=True
        with self.assertRaises(helper.Rejected):self.call('enable')
        self.assertFalse(self.p.up)

    def test_wrong_uid_and_same_uid_untrusted_pid_rejected(self):
        for peer in [(42,1001,1000),(43,1000,1000)]:
            with self.assertRaises(helper.Rejected):self.c.handle({'op':'enable'},peer)
        self.assertFalse(self.p.mutations())

    def test_inactive_systemd_service_rejected(self):
        self.p.pid=0
        with self.assertRaises(helper.Rejected):self.call('status')

    def test_unknown_operation_interface_command_or_stop_boolean_rejected(self):
        for request in [{'op':'disable'}, {'op':'shell','command':'anything'},
                        {'op':'enable','interface':'can1'}, {'op':'enable','bitrate':250000},
                        {'op':'commit_down','token':'x','stopped':True}, {'op':'prepare_down','stopped':True}]:
            with self.assertRaises(helper.Rejected):self.c.handle(request,self.peer)
        self.assertFalse(self.p.mutations())

    def test_prepare_never_changes_can(self):
        self.p.up=True
        lease=self.call('prepare_down')
        self.assertTrue(self.p.up);self.assertTrue(lease['token'])
        self.assertFalse(self.p.mutations())

    def test_commit_requires_lease(self):
        self.p.up=True
        with self.assertRaises(helper.Rejected):self.call('commit_down',token='no-proof')
        self.assertTrue(self.p.up);self.assertFalse(self.p.mutations())

    def test_commit_is_one_use_and_bound_to_control_process_identity(self):
        self.p.up=True
        token=self.call('prepare_down')['token']
        self.assertFalse(self.call('commit_down',token=token)['can']['admin_up'])
        with self.assertRaises(helper.Rejected):self.call('commit_down',token=token)
        self.assertEqual(self.p.mutations(),[[helper.IP,'link','set','dev','can0','down']])
        self.p.up=True;token=self.call('prepare_down')['token'];self.p.start='new_process'
        with self.assertRaises(helper.Rejected):self.call('commit_down',token=token)
        self.assertTrue(self.p.up)

    def test_expired_or_replaced_lease_never_turns_can_off(self):
        self.p.up=True
        token=self.call('prepare_down')['token'];self.p.now+=helper.LEASE_SECONDS+.1
        with self.assertRaises(helper.Rejected):self.call('commit_down',token=token)
        token=self.call('prepare_down')['token'];self.call('prepare_down')
        with self.assertRaises(helper.Rejected):self.call('commit_down',token=token)
        self.assertTrue(self.p.up);self.assertFalse(self.p.mutations())

    def test_enable_revokes_existing_down_lease(self):
        token=self.call('prepare_down')['token'];self.call('enable')
        with self.assertRaises(helper.Rejected):self.call('commit_down',token=token)
        self.assertTrue(self.p.up)

    def test_failed_readback_never_claims_can_off(self):
        self.p.up=True;self.p.fail_down=True;token=self.call('prepare_down')['token']
        with self.assertRaises(helper.Rejected):self.call('commit_down',token=token)
        self.assertTrue(self.p.up)

    def test_root_configuration_interface_whitelist(self):
        for kwargs in [dict(interface='can1'),dict(interface='can0; shutdown'),dict(bitrate=250000),
                       dict(control_uid=0),dict(control_uid=True),dict(control_service='any.service')]:
            with self.assertRaises(helper.Rejected):helper.Config(**kwargs)

    def test_untrusted_json_has_no_duplicate_or_nonfinite_fields(self):
        for body in ['{"op":"status","op":"enable"}','[]','{"op":NaN}']:
            with self.assertRaises((helper.Rejected,ValueError)):helper.json_object(body)


if __name__=='__main__':unittest.main()
