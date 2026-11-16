"""Client used ONLY by the trusted ordinary control service MainPID."""
import json
import socket

SOCKET_PATH='/run/robot-console/can.sock'


class Client:
    def __init__(self,path=SOCKET_PATH,timeout=4.):
        self.path,self.timeout=path,timeout

    def call(self,op,**fields):
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
            connection.settimeout(self.timeout)
            connection.connect(self.path)
            connection.sendall(json.dumps(dict(op=op,**fields),allow_nan=False).encode()+b'\n')
            data=b''
            while not data.endswith(b'\n'):
                chunk=connection.recv(4096)
                if not chunk:raise RuntimeError('CAN helper closed without a complete response')
                data+=chunk
                if len(data)>16384:raise RuntimeError('CAN helper response exceeds limit')
        result=json.loads(data)
        if not result.get('ok'):
            raise RuntimeError(result.get('error','CAN helper rejected operation'))
        return result

    def status(self):return self.call('status')
    def enable(self):return self.call('enable')
    def prepare_down(self):return self.call('prepare_down')
    def commit_down(self,token):return self.call('commit_down',token=token)

# Compatibility name for direct imports.
CanClient = Client
