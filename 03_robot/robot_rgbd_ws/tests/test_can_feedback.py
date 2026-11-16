"""Known AGX V2 payloads, including nonzero values hidden by stationary tests."""
import unittest
from bunker_rgbd.can_feedback import Feedback
from bunker_rgbd.core import Safety
from test_core import inputs


class CANFeedbackTests(unittest.TestCase):
    def test_nonzero_signed_velocity_payloads(self):
        for payload,expected in (
            ('0050007800000000',[.08,.12]),
            ('ffb0ff8800000000',[-.08,-.12]),
            ('0001000100000000',[.001,.001]),
            ('0000ff9c00000000',[0.,-.1]),
        ):
            with self.subTest(payload=payload):
                feedback=Feedback()
                feedback.ingest(0x211,bytes.fromhex('000100f000000001'),10.)
                feedback.ingest(0x221,bytes.fromhex(payload),10.)
                self.assertEqual(feedback.snapshot(10.)['velocity'],expected)

    def test_fault_code_uses_the_same_msb_first_word_order(self):
        feedback=Feedback()
        feedback.ingest(0x211,bytes.fromhex('020100f012340001'),10.)
        self.assertEqual(feedback.error_code,0x1234)
        self.assertEqual(feedback.vehicle_state,2)
        self.assertEqual(feedback.control_mode,1)

    def test_turn_feedback_matches_odom_and_real_disagreement_still_stops(self):
        data=inputs();guard=Safety(True)
        self.assertTrue(guard.grant(True,10.,10.,data)[0])
        feedback=Feedback()
        feedback.ingest(0x211,bytes.fromhex('000100f000000001'),10.)
        feedback.ingest(0x221,bytes.fromhex('0000007800000000'),10.)
        data['feedback']=feedback.snapshot(10.)
        data['odom']['w']=.12
        data['mission'].update(active=True,state='SEARCH')
        request=dict(stamp=10.,_rx=10.,generation='g',state='SEARCH',v=0.,w=.12)
        command,ready,reason=guard.tick(10.,10.,data,request)
        self.assertTrue(ready);self.assertTrue(guard.authorized)
        self.assertGreater(command[1],0.)
        data['odom']['w']=0.
        command,ready,reason=guard.tick(10.,10.,data,request)
        self.assertEqual(command,(0.,0.));self.assertFalse(guard.authorized)
        self.assertIn('CAN=(0.000, 0.120), odom=(0.000, 0.000)',reason)


if __name__=='__main__':unittest.main()
