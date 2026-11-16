#!/usr/bin/env python3
"""Small privileged CAN boundary. No ROS, driver launch or arbitrary commands.

Only the configured ordinary control service's systemd MainPID may connect.
The installed control service is the trusted stop-proof authority: it must revoke
motion, verify fresh measured standstill and zero output, then confirm driver
termination before committing a down lease. UI JSON is never stop evidence.
This helper does not claim to measure chassis motion or communication itself.
"""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import socketserver
import stat
import struct
import subprocess
import time

IP = '/usr/sbin/ip'
SYSTEMCTL = '/usr/bin/systemctl'
CONFIG_PATH = '/etc/robot-console/can.json'
SOCKET_PATH = '/run/robot-console/can.sock'
MAX_REQUEST = 4096
LEASE_SECONDS = 8.0


class Rejected(ValueError):
    pass


@dataclass(frozen=True)
class Config:
    interface: str = 'can0'
    bitrate: int = 500000
    control_uid: int = 1000
    control_service: str = 'robot-console.service'

    def __post_init__(self):
        if self.interface != 'can0' or type(self.bitrate) is not int or self.bitrate != 500000:
            raise Rejected('Only configured can0 at 500000 bit/s is supported')
        if type(self.control_uid) is not int or self.control_uid <= 0:
            raise Rejected('Control service must use a non-root UID')
        if self.control_service != 'robot-console.service':
            raise Rejected('Unexpected control service')


def load_config(path=CONFIG_PATH):
    info = os.stat(path, follow_symlinks=False)
    if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
        raise Rejected('CAN configuration must be a root-owned regular file without group/world write')
    with open(path, encoding='utf-8') as stream:
        value = json.load(stream)
    if not isinstance(value, dict) or set(value) != {'interface','bitrate','control_uid','control_service'}:
        raise Rejected('Invalid helper configuration fields')
    return Config(**value)


def run_fixed(argv):
    # All argv entries originate from constants or validated root-owned config.
    result = subprocess.run(argv, text=True, capture_output=True, timeout=3., check=True,
                            env={'PATH':'/usr/sbin:/usr/bin:/sbin:/bin','LANG':'C'}, shell=False)
    return result.stdout


def process_start_time(pid):
    # /proc stat comm is enclosed in parentheses and may contain spaces.
    text = Path('/proc') / str(pid) / 'stat'
    tail = text.read_text().rsplit(')', 1)[1].split()
    return tail[19]  # stat field22; tail starts at field3


def json_object(data):
    def unique(pairs):
        obj={}
        for key,value in pairs:
            if key in obj:
                raise Rejected('Duplicate JSON key')
            obj[key]=value
        return obj
    value=json.loads(data, object_pairs_hook=unique,
                     parse_constant=lambda value: (_ for _ in ()).throw(Rejected('Nonfinite JSON')))
    if not isinstance(value,dict):
        raise Rejected('Request must be an object')
    return value


class CanController:
    def __init__(self, config, run=run_fixed, clock=time.monotonic, identity=process_start_time):
        self.config, self.run, self.clock, self.identity = config, run, clock, identity
        self.lease = None

    def authenticate(self, peer):
        pid, uid, gid = peer
        if uid != self.config.control_uid:
            raise Rejected('Peer UID is not the control service user')
        raw = self.run([SYSTEMCTL,'show','--property=MainPID','--value',self.config.control_service]).strip()
        if not raw.isdigit() or int(raw) <= 0 or int(raw) != pid:
            raise Rejected('Peer PID is not the active control service MainPID')
        return pid, self.identity(pid)

    def status(self):
        raw=self.run([IP,'-details','-json','link','show','dev',self.config.interface])
        links=json.loads(raw)
        if not isinstance(links,list) or len(links)!=1 or links[0].get('ifname')!=self.config.interface:
            raise Rejected('Configured interface status unavailable')
        link=links[0]
        if link.get('link_type') != 'can' and link.get('linkinfo',{}).get('info_kind') != 'can':
            raise Rejected('Configured interface is not CAN')
        info=link.get('linkinfo',{}).get('info_data',{})
        timing=info.get('bittiming',{})
        bitrate=timing.get('bitrate', info.get('bitrate'))
        admin_up='UP' in link.get('flags',[])
        return {'interface':self.config.interface,'admin_up':admin_up,
                'operstate':link.get('operstate','UNKNOWN'),'bitrate':bitrate,
                'bus_state':info.get('state','UNKNOWN'),
                'configured_bitrate':self.config.bitrate,
                'chassis_communication':'unverified','motion_authorized':False}

    def handle(self, request, peer):
        who=self.authenticate(peer)
        op=request.get('op')
        expected={'op','token'} if op=='commit_down' else {'op'}
        if set(request)!=expected or op not in ('status','enable','prepare_down','commit_down'):
            raise Rejected('Operation or fields are not allowed')
        if op=='status':
            return {'ok':True,'can':self.status()}
        if op=='enable':
            self.lease=None
            before=self.status()
            if before['admin_up']:
                if before['bitrate']!=self.config.bitrate:
                    raise Rejected('CAN is already UP with an unexpected bitrate; refusing implicit reconfiguration')
                return {'ok':True,'can':before,'changed':False}
            self.run([IP,'link','set','dev',self.config.interface,'type','can','bitrate',str(self.config.bitrate)])
            self.run([IP,'link','set','dev',self.config.interface,'up'])
            after=self.status()
            if not after['admin_up'] or after['bitrate']!=self.config.bitrate:
                raise Rejected('CAN enable verification failed')
            return {'ok':True,'can':after,'changed':True}
        if op=='prepare_down':
            # Issuance performs no CAN operation. Only MainPID can obtain/use it.
            token=secrets.token_urlsafe(32)
            self.lease=(token,who,self.clock()+LEASE_SECONDS)
            return {'ok':True,'token':token,'expires_in':LEASE_SECONDS,
                    'required_sequence':'guard revoke; mission stop/cancel; fresh stable zero feedback; driver exited; commit'}
        lease,self.lease=self.lease,None  # consume even on failure; never replay
        token=request.get('token')
        if (not lease or not isinstance(token,str) or not secrets.compare_digest(token,lease[0]) or
                who!=lease[1] or self.clock()>lease[2]):
            raise Rejected('Missing, expired, replayed or wrong-peer down lease')
        before=self.status()
        if before['admin_up']:
            self.run([IP,'link','set','dev',self.config.interface,'down'])
        after=self.status()
        if after['admin_up']:
            raise Rejected('CAN down verification failed; stop state is not established')
        return {'ok':True,'can':after,'changed':before['admin_up'],
                'physical_stop':'verified_by_control_service_before_commit'}


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(2.)
        try:
            peer=struct.unpack('3i',self.request.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,struct.calcsize('3i')))
            data=self.rfile.readline(MAX_REQUEST+1)
            if not data.endswith(b'\n') or len(data)>MAX_REQUEST:
                raise Rejected('Request exceeds framing limit')
            request=json_object(data)
            result=self.server.controller.handle(request,peer)
        except (Rejected,ValueError,KeyError,TypeError,OSError,subprocess.SubprocessError) as error:
            # Do not return stderr, shell text or platform-specific sensitive data.
            result={'ok':False,'error':str(error) if isinstance(error,Rejected) else type(error).__name__,
                    'physical_stop':'unconfirmed'}
        self.wfile.write(json.dumps(result,allow_nan=False).encode()+b'\n')


class Server(socketserver.UnixStreamServer):
    allow_reuse_address=False


def main():
    if os.geteuid()!=0:
        raise SystemExit('CAN helper must run in its root systemd service')
    config=load_config()
    path=Path(SOCKET_PATH)
    path.parent.mkdir(mode=0o750,parents=True,exist_ok=True)
    if path.exists() or path.is_symlink():
        if not stat.S_ISSOCK(path.lstat().st_mode):
            raise SystemExit('Refusing to replace a non-socket helper path')
        path.unlink()
    with Server(str(path),Handler) as server:
        os.chmod(path,0o660)
        server.controller=CanController(config)
        server.timeout=.2
        running=True
        def stop(signum,frame):
            nonlocal running
            running=False
        signal.signal(signal.SIGTERM,stop)
        signal.signal(signal.SIGINT,stop)
        try:
            while running:
                server.handle_request()
        finally:
            path.unlink(missing_ok=True)
    # Helper shutdown never toggles CAN or claims that the robot stopped.


if __name__=='__main__':main()
