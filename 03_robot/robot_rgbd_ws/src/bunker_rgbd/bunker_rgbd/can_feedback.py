"""Receive-only SocketCAN evidence; never transmits any frame.

The stock Bunker driver republishes cached SDK state at 50Hz even on bus loss.
Fresh /bunker_status alone is therefore NOT proof of live chassis communication.
AGX V2 payload words are MSB-first, as defined by struct16_t (high_byte,
low_byte) in ugv_sdk/src/protocol_v2/agilex_protocol_v2.h. The SocketCAN
envelope and filter structs below still use the kernel's native byte order.
"""
import math
import socket
import struct
import threading
import time
from .timing import CAN_FEEDBACK_TIMEOUT,CAN_RECEIVE_TIMEOUT


class Feedback:
    def __init__(self):
        self.system_at=self.motion_at=-math.inf;self.velocity=None;self.error='waiting for AGX_V2 0x211/0x221';self.vehicle_state=None;self.control_mode=None;self.error_code=None
    def ingest(self,can_id,payload,now):
        if can_id & 0xe0000000 or len(payload)!=8:return
        if can_id==0x211:
            self.system_at=now;self.vehicle_state=payload[0];self.control_mode=payload[1]
            self.error_code=int.from_bytes(payload[4:6],'big');self.error=''
        elif can_id==0x221:
            self.motion_at=now;v,w=struct.unpack_from('>hh',payload)
            self.velocity=(v/1000.,w/1000.);self.error=''
    def snapshot(self,now):
        age=max(now-self.system_at,now-self.motion_at)
        valid=0<=age<=CAN_FEEDBACK_TIMEOUT and self.velocity is not None
        return {'_received':min(self.system_at,self.motion_at),'_age':0.,'age':age if math.isfinite(age) else 1e9,
                'live':valid,'stopped':bool(valid and abs(self.velocity[0])<.015 and abs(self.velocity[1])<.025),
                'velocity':list(self.velocity) if self.velocity else None,'protocol':'AGX_V2',
                'vehicle_state':self.vehicle_state,'control_mode':self.control_mode,'error_code':self.error_code,
                'reason':self.error or ('' if valid else 'CAN feedback expired')}


class PassiveCAN:
    def __init__(self,interface):
        if interface!='can0':raise ValueError('only reviewed can0 supported')
        self.interface=interface;self.feedback=Feedback();self.stop=threading.Event();self.lock=threading.Lock()
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
    def run(self):
        while not self.stop.is_set():
            try:
                with socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as sock:
                    # Kernel filters accept feedback only; no send method is ever used.
                    sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,struct.pack('=IIII',0x211,0x7ff,0x221,0x7ff))
                    sock.bind((self.interface,));sock.settimeout(CAN_RECEIVE_TIMEOUT)
                    while not self.stop.is_set():
                        try:frame=sock.recv(16)
                        except socket.timeout:continue
                        if len(frame)!=16:continue
                        can_id,dlc,payload=struct.unpack('=IB3x8s',frame)
                        with self.lock:self.feedback.ingest(can_id,payload[:dlc],time.monotonic())
            except (OSError,AttributeError) as exc:
                with self.lock:self.feedback.error='CAN receive unavailable: '+type(exc).__name__
                self.stop.wait(1.)
    def snapshot(self):
        with self.lock:return self.feedback.snapshot(time.monotonic())
