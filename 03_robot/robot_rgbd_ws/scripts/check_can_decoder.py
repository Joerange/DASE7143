#!/usr/bin/env python3
"""Compare receive-only decoding with the actual SDK C parser. No CAN I/O."""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import platform
import subprocess
import tempfile
from bunker_rgbd.can_feedback import Feedback

parser=argparse.ArgumentParser()
parser.add_argument('--sdk',type=Path,default=Path.home()/'ros2_ws/src/ugv_sdk')
parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'validation/jetson/can_decoder_check.json')
args=parser.parse_args()
wrapper=r'''
#include "protocol_v2/agilex_msg_parser_v2.h"
int probe_decode(uint32_t id, const uint8_t *data, double *out) {
    struct can_frame frame = {0};
    AgxMessage message = {0};
    frame.can_id = id; frame.can_dlc = 8;
    memcpy(frame.data, data, 8);
    if (!DecodeCanFrameV2(&frame, &message)) return 0;
    if (id == 0x221) {
        out[0] = message.body.motion_state_msg.linear_velocity;
        out[1] = message.body.motion_state_msg.angular_velocity;
    } else if (id == 0x211) {
        out[0] = message.body.system_state_msg.vehicle_state;
        out[1] = message.body.system_state_msg.control_mode;
        out[2] = message.body.system_state_msg.error_code;
    } else return 0;
    return 1;
}
'''
cases=[]
with tempfile.TemporaryDirectory(prefix='rgbd_can_parser_') as directory:
    folder=Path(directory);source=folder/'probe.c';library=folder/'probe.so'
    source.write_text(wrapper)
    subprocess.run(['cc','-shared','-fPIC','-O2','-I'+str(args.sdk/'src'),'-I'+str(args.sdk/'include'),
                    str(source),str(args.sdk/'src/protocol_v2/agilex_msg_parser_v2.c'),'-lm','-o',str(library)],check=True)
    sdk=C.CDLL(str(library));sdk.probe_decode.argtypes=[C.c_uint32,C.POINTER(C.c_uint8),C.POINTER(C.c_double)]
    sdk.probe_decode.restype=C.c_int
    vectors=[(0x221,p) for p in ('0000000000000000','0050007800000000','ffb0ff8800000000',
                               '0001000100000000','0000ff9c00000000','7fff800000000000')]
    vectors += [(0x211,p) for p in ('000100f000000001','020100f012340001','000100f001000001')]
    for can_id,hex_data in vectors:
        payload=bytes.fromhex(hex_data);buffer=(C.c_uint8*8).from_buffer_copy(payload);decoded=(C.c_double*3)()
        assert sdk.probe_decode(can_id,buffer,decoded)
        feedback=Feedback();feedback.ingest(can_id,payload,10.)
        actual=list(feedback.velocity) if can_id==0x221 else [feedback.vehicle_state,feedback.control_mode,feedback.error_code]
        reference=list(decoded)[:len(actual)]
        match=all(math.isclose(a,b,rel_tol=1e-6,abs_tol=1e-6) for a,b in zip(actual,reference))
        cases.append(dict(can_id=hex(can_id),payload=hex_data,python=actual,sdk=reference,match=match))
        assert match,cases[-1]
result=dict(mode='OFFLINE SDK comparison: no sockets, no driver, no motion',platform=platform.platform(),
            sdk_source=str(args.sdk),passed=len(cases),cases=cases)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
