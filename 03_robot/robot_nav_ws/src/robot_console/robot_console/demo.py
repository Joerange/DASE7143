"""In-process console fixtures. No ROS imports, Unix sockets, subprocess or CAN."""
import base64
import copy
import math
import time
from .rendering import control_enabled


class FakeClient:
    def __init__(self):
        self.started=time.monotonic()
        self.can_up=True
        self.driver_running=True
        self.communication_ok=False
        self.armed=False
        self.mission_started=None
        self.counter=0
        self.events=[]
        self.operations={}
        self.connected=True
        self.standstill_verified=False
        self.stop_timeout=False
        self._frame_cache=None
        self.add_event('system','SIMULATED：CAN 为 UP，但尚无底盘通信；禁止允许运动。')

    def add_event(self,kind,message,**extra):
        self.counter+=1
        self.events.append(dict(id='demo-'+str(self.counter),stamp=time.time(),kind=kind,message=message,
                                task_generation='simulated-task',geometry_epoch=0,**extra))
        self.events=self.events[-100:]

    def set_connected(self, connected):
        self.connected=bool(connected)

    def inject_fault(self, name):
        if name=='missing_standstill':self.standstill_verified=False
        elif name=='stop_timeout':self.stop_timeout=True
        elif name=='clear':self.stop_timeout=False;self.standstill_verified=self.communication_ok
        else:raise ValueError('unknown demo fault')

    def operation(self, action, result, message):
        self.add_event('local_decision',message,result=result,action=action)
        op_id='simulated-'+str(self.counter)
        self.operations[op_id]=dict(id=op_id,action=action,result=result,phase='finished',message=message)
        return {'ok':True,'operation_id':op_id,'message':'演示请求已提交；查看模拟状态回读。'}

    def controls(self):
        stopped=self.mission_started is None
        return {
            'can_enable':not self.can_up,
            'can_disable':stopped,
            'driver_start':self.can_up and not self.communication_ok,
            'driver_stop':self.driver_running,
            'arm':{'enabled':self.can_up and self.communication_ok and not self.armed,
                   'reason':'仅有 CAN UP 不代表底盘通信正常。'},
            'disarm':self.armed,
            'mission_start':self.armed and self.communication_ok and stopped,
            'mission_stop':True,'export':True,
        }

    def request(self,payload):
        if not self.connected:raise ConnectionError('simulated backend disconnected')
        if payload.get('op')=='snapshot':return self.snapshot()
        if payload.get('op')!='action':return {'ok':False,'error':'unknown_demo_operation'}
        action=payload.get('action','')
        allowed,_=control_enabled(self.controls(),action)
        if not allowed:
            self.add_event('system','演示操作被拒绝：'+action,error='prerequisite_not_met')
            return {'ok':False,'error':'演示前置条件不满足；未操作任何真实设备。'}
        if action in ('mission_stop','disarm','can_disable','driver_stop') and self.stop_timeout:
            self.armed=False
            return self.operation(action,'unknown','模拟停止确认超时：实际停稳未知，CAN 保持原状态。')
        if action in ('can_disable','driver_stop') and not self.standstill_verified:
            return self.operation(action,'failed','模拟缺少实时停稳证据：未关闭 CAN、未停止底盘驱动。')
        if action=='can_enable':self.can_up=True
        elif action=='can_disable':self.can_up=False;self.driver_running=False;self.communication_ok=False;self.armed=False
        elif action=='driver_start':self.driver_running=True;self.communication_ok=True;self.standstill_verified=True
        elif action=='driver_stop':self.driver_running=False;self.communication_ok=False;self.armed=False;self.mission_started=None
        elif action=='arm':self.armed=True
        elif action=='disarm':self.armed=False;self.mission_started=None
        elif action=='mission_start':self.mission_started=time.monotonic();self.standstill_verified=False
        elif action=='mission_stop':self.mission_started=None;self.standstill_verified=self.communication_ok
        elif action=='export':
            return {'ok':True,'message':'演示模式未创建实机日志，也未访问硬件。','operation_id':'simulated-export'}
        return self.operation(action,'success','仅修改演示数据，未调用 ROS / CAN。')

    def image_frames(self,stamp):
        # Generated diagrams deliberately look unlike a real camera feed.
        if self._frame_cache is None:
            from PyQt5 import QtCore,QtGui
            images={}
            for name in ('rgb','depth'):
                image=QtGui.QImage(640,360,QtGui.QImage.Format_RGB888)
                painter=QtGui.QPainter(image)
                if name=='rgb':
                    painter.fillRect(image.rect(),QtGui.QColor('#b2c0cc'))
                    painter.fillRect(0,215,640,145,QtGui.QColor('#697f8c'))
                    painter.setPen(QtGui.QPen(QtGui.QColor('#859ba8'),2))
                    for x in range(-200,1000,100):painter.drawLine(320,210,x,360)
                    for y in (235,270,320):painter.drawLine(0,y,640,y)
                    painter.fillRect(70,60,90,155,QtGui.QColor('#3c5368'))
                    painter.fillRect(415,95,125,125,QtGui.QColor('#8a684f'))
                    painter.fillRect(257,118,117,167,QtGui.QColor('#248d59'))
                    painter.fillRect(247,110,137,22,QtGui.QColor('#195e3d'))
                    painter.fillRect(270,141,90,105,QtGui.QColor('#39a86a'))
                    painter.setPen(QtGui.QColor('#def4e9'));painter.setFont(QtGui.QFont('Sans',28))
                    painter.drawText(289,204,'BIN')
                else:
                    for y in range(360):
                        color=QtGui.QColor.fromHsv(int(230-y*.45),210,220)
                        painter.fillRect(0,y,640,1,color)
                    painter.fillRect(257,118,117,167,QtGui.QColor('#ffce55'))
                    painter.setPen(QtGui.QColor('white'));painter.setFont(QtGui.QFont('Sans',18))
                    painter.drawText(14,337,'SIMULATED DEPTH   0.25 m  —  4.0 m')
                painter.setPen(QtGui.QColor('#fff7cf'));painter.setFont(QtGui.QFont('Sans',15))
                painter.drawText(15,26,'SIMULATED / NOT A LIVE CAMERA')
                painter.end()
                buffer=QtCore.QBuffer();buffer.open(QtCore.QIODevice.WriteOnly);image.save(buffer,'JPEG',85)
                images[name]=base64.b64encode(bytes(buffer.data())).decode()
            self._frame_cache=images
        return {name:dict(jpeg=encoded,stamp=stamp,width=640,height=360,frame_id='simulated_optical')
                for name,encoded in self._frame_cache.items()}

    def snapshot(self):
        stamp=time.time()
        elapsed=0. if self.mission_started is None else time.monotonic()-self.mission_started
        if self.mission_started is None:state='INIT'
        elif elapsed<3:state='SEARCH_CONTINUOUS'
        elif elapsed<6:state='CONFIRM_PENDING'
        elif elapsed<9:state='WAIT_PATH'
        elif elapsed<18:state='NAVIGATE'
        elif elapsed<23:state='APPROACH'
        else:state='DONE'
        reason={'INIT':'等待人工开始；演示初始通信尚未验证','SEARCH_CONTINUOUS':'持续本地检测，不等待转满一圈',
                'CONFIRM_PENDING':'候选已停稳；等待模拟 VLM 结果','WAIT_PATH':'停车等待已知自由空间路径',
                'NAVIGATE':'已找到安全路径，模拟靠近','APPROACH':'当前深度监督，模拟限速',
                'DONE':'模拟净距 0.60 m，实际速度稳定为零'}[state]
        if not self.events or not any(e.get('state')==state for e in self.events):
            self.add_event('mission_state',reason,state=state)
            if state=='CONFIRM_PENDING':
                self.add_event('detector','trash can 候选稳定：3帧中2帧',observation_id='sim-obs-1')
                self.add_event('vlm_confirmation','模拟批量确认请求：候选1 → match',observation_id='sim-obs-1',request_id='sim-req-1',latency=.83)
        moving=state in ('SEARCH_CONTINUOUS','NAVIGATE','APPROACH') and self.armed
        v=.04 if moving and state!='SEARCH_CONTINUOUS' else 0.
        w=.10 if moving and state=='SEARCH_CONTINUOUS' else 0.
        width,height,resolution=120,100,.05
        data=[]
        for y in range(height):
            for x in range(width):
                data.append(-1 if x>101 or y>88 else 100 if x in (0,100) or y in (0,87) else 0)
        for y in range(58,66):
            for x in range(63,69):data[y*width+x]=100
        sensors={key:dict(state='normal',age=.06,hz=rate,stamp=stamp-.06) for key,rate in
                 [('rgb',15.),('depth',30.),('lidar',9.),('map',.5),('tf',50.),('odom',50.),
                  ('safety',20.),('mission',20.),('localization',20.),('nav2',20.)]}
        if not self.communication_ok:
            sensors['odom']=dict(state='disconnected',age=None,hz=0.,reason='尚无已验证底盘里程计')
        gap=.60 if state=='DONE' else .78 if state=='APPROACH' else 1.15
        target=dict(track_id='simulated-task:0:1',valid=True,map_point=[1.3,1.1,.35],
                    base_point=[gap+.345,.03,.35],optical_point=[-.03,.02,gap+.14],gap=gap,bearing=.02,stamp=stamp)
        return dict(schema=1,demo=True,stamp=stamp,
                    mission=dict(state=state,reason=reason,stamp=stamp,task_generation='simulated-task',geometry_epoch=0,
                                 motion_enabled=self.armed,target_track_id=target['track_id']),
                    safety=dict(stamp=stamp,authorized=self.armed,authorization_reason='simulation permission' if self.armed else 'simulation disarmed',ready=self.communication_ok,reason='authorized_simulation' if self.armed else 'disarmed_simulation'),
                    chassis=dict(driver_running=self.driver_running,communication_ok=self.communication_ok,
                                 velocity=dict(linear_x=v,angular_z=w,age=.05 if self.communication_ok else None,valid=self.communication_ok)),
                    can=dict(interface='can0',state='UP' if self.can_up else 'DOWN',bus_state='ERROR-ACTIVE' if self.can_up else 'STOPPED'),controls=self.controls(),sensors=sensors,
                    frames=self.image_frames(stamp),candidates=dict(stamp=stamp,frame_id='simulated_optical',observation_id='sim-obs-1',
                    candidates=[dict(id=1,track_id=target['track_id'],label='trash can',score=.84,bbox=[247,109,385,287])]),
                    scan=dict(stamp=stamp,frame='base_link',angle_min=-math.pi,angle_increment=2*math.pi/720,
                              ranges=[2.7+.4*math.sin(i*.017) for i in range(720)],display_range=4.),
                    map=dict(stamp=int(elapsed/2),version=int(elapsed/2),width=width,height=height,resolution=resolution,
                             origin=[-2.,-2.,0.],data=data),pose=[0.,0.,.58],targets=[target],goal=[.78,.66,.58],
                    path=[[0.,0.],[.2,.12],[.4,.30],[.6,.5],[.78,.66]],events=copy.deepcopy(self.events),operations=copy.deepcopy(self.operations))
