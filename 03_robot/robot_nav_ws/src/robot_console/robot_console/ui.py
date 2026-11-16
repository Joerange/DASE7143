"""Native, unprivileged console. Only IPC actions; never publishes velocity or uses CAN."""
import argparse
import base64
import collections
import json
import math
import os
import socket
import threading
import time
from pathlib import Path
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from .rendering import (matching_candidates, velocity_text, world_to_grid, body_polygon,
                        normalize_pose, control_enabled, sensor_text, timeline_category, finite_number,
                        feed_fresh, authorization_value, current_mission_state, LatestSlot)


class UnixClient:
    def __init__(self, path=None):
        self.path = path or '/run/user/%d/robot-console.sock' % os.getuid()

    def request(self, payload):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(.8)
            connection.connect(self.path)
            connection.sendall(json.dumps(payload,separators=(',',':')).encode()+b'\n')
            chunks, length = [], 0
            while True:
                chunk = connection.recv(65536)
                if not chunk:
                    break
                chunks.append(chunk)
                length += len(chunk)
                if length > 32*1024*1024:
                    raise ValueError('console response too large')
                if b'\n' in chunk:
                    break
            answer = json.loads(b''.join(chunks).split(b'\n',1)[0])
            if not isinstance(answer,dict):
                raise ValueError('invalid backend response')
            return answer


class BackendWorker(QtCore.QThread):
    snapshot = QtCore.pyqtSignal()
    response = QtCore.pyqtSignal(str,dict)
    disconnected = QtCore.pyqtSignal(str)

    def __init__(self, client):
        super().__init__()
        self.client, self.actions = client, collections.deque()
        self.lock, self.quit_event = threading.Lock(), threading.Event()
        self.latest_snapshot = LatestSlot()

    def action(self, name):
        with self.lock:
            if name in ('mission_stop','disarm'):
                # A stop takes precedence over queued enable/start requests.
                self.actions.clear()
                self.actions.appendleft(name)
            elif name not in self.actions:
                self.actions.append(name)

    def run(self):
        deadline = 0.
        while not self.quit_event.is_set():
            action = None
            with self.lock:
                if self.actions:
                    action = self.actions.popleft()
            try:
                if action:
                    self.response.emit(action,self.client.request({'op':'action','action':action}))
                    action = None  # A later snapshot failure cannot undo an acknowledged reply.
                if time.monotonic() >= deadline:
                    result = self.client.request({'op':'snapshot'})
                    if result.get('ok') is False:
                        raise ValueError('snapshot unavailable')
                    value=(time.monotonic(),result.get('snapshot',result))
                    if self.latest_snapshot.offer(value):self.snapshot.emit()
                    deadline = time.monotonic()+.2
            except (OSError,ValueError,KeyError) as error:
                if action:
                    self.response.emit(action,{'ok':False,'outcome':'unknown','error':type(error).__name__,
                                               'message':'请求可能已被接受，但回复不可用；结果未知，正在重新查询实际状态。'})
                self.disconnected.emit(type(error).__name__)
                deadline = time.monotonic()+.5
            self.quit_event.wait(.025)

    def stop(self):
        self.quit_event.set()
        self.wait(1500)


PALETTE = {'bg':'#101923','panel':'#172431','line':'#334555','text':'#e6edf3',
           'muted':'#96a9bb','ok':'#60d49c','warn':'#f3c36c','bad':'#ff7b82','cyan':'#67d6ed'}
STATE_NAMES = {'INIT':'等待就绪','SEARCH_CONTINUOUS':'旋转搜索','ACQUIRE':'候选采集 / 减速',
               'CONFIRM_PENDING':'等待语义确认','WAIT_PATH':'停车等待路径','READY_TO_NAV':'准备导航',
               'NAVIGATE':'导航靠近','APPROACH':'近距靠近','REOBSERVE_RECOVERY':'停车重新观察',
               'VERIFY_STOP':'核验停稳','DONE':'任务完成','HOLD':'保持停车'}
SENSOR_NAMES = {'rgb':'RGB','depth':'深度','lidar':'LiDAR','scan':'LiDAR','odom':'里程计','tf':'TF','map':'地图','localization':'SLAM 定位','nav2':'Nav2','safety':'安全节点'}
HEALTH_NAMES = {'normal':'正常','healthy':'正常','ok':'正常','waiting':'等待','stale':'过期','disconnected':'断流','invalid':'无效'}
ACTIONS = [('can_enable','启用 CAN'),('can_disable','关闭 CAN'),('driver_start','启动底盘驱动'),
           ('driver_stop','停止底盘驱动'),('arm','允许任务运动'),('disarm','撤销运动'),
           ('mission_start','开始任务'),('mission_stop','停止任务'),('export','导出日志')]


def QColor(value):
    return QtGui.QColor(value)


def target_rows(snapshot):
    value = snapshot.get('targets',[])
    return value.get('targets',[]) if isinstance(value,dict) else value if isinstance(value,list) else []


def selected_target(snapshot):
    mission = snapshot.get('mission',{})
    track = mission.get('observed_target_track_id') or mission.get('target_track_id','')
    return next((row for row in target_rows(snapshot) if row.get('track_id') == track), {})


class SceneView(QtWidgets.QWidget):
    def __init__(self, kind):
        super().__init__()
        self.kind, self.snapshot, self.image = kind, {}, QtGui.QImage()
        self.image_key, self.map_image, self.map_key = None, QtGui.QImage(), None
        self.setMinimumSize(240,170)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,QtWidgets.QSizePolicy.Expanding)

    def update_data(self, snapshot):
        self.snapshot = snapshot
        if self.kind in ('rgb','depth'):
            frame = snapshot.get('frames',{}).get(self.kind,{})
            if isinstance(frame,str):
                frame = {'jpeg':frame}
            encoded = frame.get('jpeg',frame.get('data',''))
            key = (frame.get('stamp'),len(encoded),encoded[:30])
            if key != self.image_key:
                self.image_key = key
                self.image = QtGui.QImage()
                try:
                    self.image.loadFromData(base64.b64decode(encoded))
                except (ValueError,TypeError):
                    pass
        if self.kind == 'map':
            metadata = (snapshot.get('map') or {})
            data = metadata.get('data',[])
            if not data:
                self.map_image, self.map_key = QtGui.QImage(), None
            key = (metadata.get('version'),metadata.get('stamp'),metadata.get('width'),metadata.get('height'),tuple(metadata.get('origin',[])))
            if key != self.map_key and data:
                self.map_key = key
                try:
                    width,height=int(metadata['width']),int(metadata['height'])
                    if width*height != len(data) or not 0 < width*height <= 4_000_000:
                        raise ValueError('invalid map')
                    pixels = bytearray()
                    for y in range(height-1,-1,-1):
                        for cell in data[y*width:(y+1)*width]:
                            pixels.extend((60,76,89) if cell<0 else (31,42,51) if cell>=50 else (200,215,219))
                    self.map_image=QtGui.QImage(bytes(pixels),width,height,width*3,QtGui.QImage.Format_RGB888).copy()
                except (ValueError,KeyError,TypeError):
                    self.map_image=QtGui.QImage()
        self.update()

    def message(self,painter,text,rect=None):
        painter.setPen(QColor(PALETTE['muted']))
        painter.drawText(rect or self.rect(),QtCore.Qt.AlignCenter|QtCore.Qt.TextWordWrap,text)

    def image_rect(self,image):
        if image.isNull():
            return QtCore.QRectF()
        scale=min(self.width()/image.width(),self.height()/image.height())
        w,h=image.width()*scale,image.height()*scale
        return QtCore.QRectF((self.width()-w)/2,(self.height()-h)/2,w,h)

    def paintEvent(self,event):
        painter=QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(),QColor('#0b121a'))
        if self.kind in ('rgb','depth'):
            if self.image.isNull():
                self.message(painter,'等待图像')
            else:
                rect=self.image_rect(self.image)
                painter.drawImage(rect,self.image)
                if self.kind=='rgb':
                    frame=self.snapshot.get('frames',{}).get('rgb',{})
                    candidates=self.snapshot.get('candidates',{})
                    rows=matching_candidates(frame,candidates)
                    mission=self.snapshot.get('mission',{})
                    selected=mission.get('observed_target_track_id') or mission.get('target_track_id','')
                    for row in rows:
                        x1,y1,x2,y2=row['bbox']
                        sx,sy=rect.width()/self.image.width(),rect.height()/self.image.height()
                        box=QtCore.QRectF(rect.x()+x1*sx,rect.y()+y1*sy,(x2-x1)*sx,(y2-y1)*sy)
                        chosen=bool(selected) and row.get('track_id')==selected
                        painter.setPen(QtGui.QPen(QColor('#ffd36a' if chosen else '#67efab'),3 if chosen else 2))
                        painter.drawRect(box)
                        text=('已选择  ' if chosen else '')+str(row.get('label','候选'))+' '+str(row.get('id',''))
                        score=row.get('score')
                        if finite_number(score): text+='  %.2f' % score
                        painter.fillRect(QtCore.QRectF(box.left(),max(rect.top(),box.top()-23),min(box.width()+70,rect.width()),23),QColor('#c0101923'))
                        painter.drawText(QtCore.QPointF(box.left()+4,max(rect.top()+17,box.top()-6)),text)
                    if isinstance(candidates,dict) and candidates.get('candidates') and not rows:
                        self.caption(painter,'检测结果等待对应原始帧；未叠加异步框')
        elif self.kind=='lidar':
            self.paint_lidar(painter)
        else:
            self.paint_map(painter)
        if self.snapshot.get('demo'):
            painter.setPen(QColor('#ffd36a'))
            painter.drawText(self.rect().adjusted(10,8,-10,-8),QtCore.Qt.AlignTop|QtCore.Qt.AlignRight,'SIMULATED / 演示数据')

    def caption(self,painter,text):
        painter.fillRect(QtCore.QRectF(0,self.height()-29,self.width(),29),QColor('#d0101923'))
        painter.setPen(QColor(PALETTE['text']))
        painter.drawText(self.rect().adjusted(9,0,-9,-5),QtCore.Qt.AlignBottom|QtCore.Qt.AlignLeft,text)

    def paint_lidar(self,painter):
        scan=self.snapshot.get('scan',{})
        frame=scan.get('frame',scan.get('frame_id','base_link'))
        if frame!='base_link':
            self.message(painter,'等待 base_link 扫描；当前 '+str(frame)+' 不与车体轮廓叠加')
            return
        ranges=scan.get('ranges',[])
        radius=max(1.,float(scan.get('display_range',4.)))
        scale=min(self.width(),self.height())/(2*radius)
        cx,cy=self.width()/2,self.height()/2
        point=lambda x,y: QtCore.QPointF(cx-y*scale,cy-x*scale)
        painter.setPen(QtGui.QPen(QColor('#263a4b'),1))
        for distance in range(1,int(radius)+1):
            painter.drawEllipse(QtCore.QPointF(cx,cy),distance*scale,distance*scale)
        painter.drawLine(point(-radius,0),point(radius,0));painter.drawLine(point(0,-radius),point(0,radius))
        painter.setPen(QtGui.QPen(QColor('#72d5e8'),2))
        amin,inc=scan.get('angle_min',-math.pi),scan.get('angle_increment',scan.get('increment',0))
        for i,r in enumerate(ranges):
            if finite_number(r) and 0 < r <= radius:
                a=amin+i*inc
                painter.drawPoint(point(r*math.cos(a),r*math.sin(a)))
        polygon=QtGui.QPolygonF([point(x,y) for x,y in body_polygon((0.,0.,0.))])
        painter.setPen(QtGui.QPen(QColor('#ffffff'),2));painter.setBrush(QColor('#5274a0'))
        painter.drawPolygon(polygon)
        painter.setPen(QtGui.QPen(QColor('#ffd36a'),2));painter.drawLine(point(0,0),point(.75,0))
        self.caption(painter,'+X 车头 ↑   +Y 车体左侧 ←    圆环 1 m；显示半径 %.1f m' % radius)
        if not ranges: self.message(painter,'等待 /scan')

    def paint_map(self,painter):
        if self.map_image.isNull():
            self.message(painter,'等待 SLAM 初始地图\n未知区域不会被当作可通行空间')
            return
        metadata=(self.snapshot.get('map') or {})
        rect=self.image_rect(self.map_image)
        painter.drawImage(rect,self.map_image)
        def point(x,y):
            gx,gy=world_to_grid(x,y,metadata)
            return QtCore.QPointF(rect.x()+gx*rect.width()/self.map_image.width(),rect.y()+gy*rect.height()/self.map_image.height())
        pose=normalize_pose(self.snapshot.get('pose'))
        path=self.snapshot.get('path',[])
        if isinstance(path,dict):path=path.get('points',path.get('poses',[]))
        line=QtGui.QPolygonF()
        for p in path:
            xy=normalize_pose(p)
            if xy:line.append(point(*xy[:2]))
        painter.setPen(QtGui.QPen(QColor('#46a0ff'),3));painter.setBrush(QtCore.Qt.NoBrush)
        if len(line)>1:painter.drawPolyline(line)
        if pose:
            painter.setPen(QtGui.QPen(QColor('#10344f'),2));painter.setBrush(QColor('#56a8e8'))
            painter.drawPolygon(QtGui.QPolygonF([point(x,y) for x,y in body_polygon(pose)]))
            painter.setPen(QtGui.QPen(QColor('#e85e65'),2))
            painter.drawLine(point(*pose[:2]),point(pose[0]+.6*math.cos(pose[2]),pose[1]+.6*math.sin(pose[2])))
        selected=self.snapshot.get('mission',{}).get('target_track_id','')
        locked=self.snapshot.get('mission',{}).get('locked_target')
        targets=target_rows(self.snapshot)
        if isinstance(locked,dict):
            targets=[t for t in targets if t.get('track_id')!=selected]+[locked]
        for target in targets:
            xyz=target.get('map_point')
            if isinstance(xyz,list) and len(xyz)>=2 and all(finite_number(x) for x in xyz[:2]):
                painter.setPen(QtGui.QPen(QColor('#ef9b24'),2));painter.setBrush(QColor('#ffd36a'))
                painter.drawEllipse(point(*xyz[:2]),7 if target.get('track_id')==selected else 4,7 if target.get('track_id')==selected else 4)
                if target is locked:
                    adaptive=self.snapshot.get('mission',{}).get('navigation_mode')=='adaptive_map'
                    painter.drawText(point(*xyz[:2])+QtCore.QPointF(10,-10),'视觉修正目标' if adaptive else '已锁定')
        goal=normalize_pose(self.snapshot.get('goal',self.snapshot.get('mission',{}).get('goal')))
        if goal:
            gp=point(*goal[:2]);painter.setPen(QtGui.QPen(QColor('#25a56a'),3));painter.setBrush(QtCore.Qt.NoBrush)
            painter.drawEllipse(gp,8,8);painter.drawLine(gp+QtCore.QPointF(-11,0),gp+QtCore.QPointF(11,0))
            painter.drawLine(gp+QtCore.QPointF(0,-11),gp+QtCore.QPointF(0,11))
        self.caption(painter,'蓝：车体 / 路径   黄：垃圾桶   绿：停靠点   灰：未知')


class Panel(QtWidgets.QFrame):
    def __init__(self,title,kind):
        super().__init__()
        self.setObjectName('panel')
        layout=QtWidgets.QVBoxLayout(self);layout.setContentsMargins(10,8,10,8);layout.setSpacing(5)
        self.title=QtWidgets.QLabel(title);self.title.setObjectName('panelTitle')
        self.health=QtWidgets.QLabel('等待');self.health.setObjectName('muted')
        line=QtWidgets.QHBoxLayout();line.addWidget(self.title);line.addStretch();line.addWidget(self.health)
        layout.addLayout(line)
        self.view=SceneView(kind);layout.addWidget(self.view,1)
        self.detail=QtWidgets.QLabel(' ');self.detail.setObjectName('muted');self.detail.setWordWrap(True)
        layout.addWidget(self.detail)


class ConsoleWindow(QtWidgets.QMainWindow):
    def __init__(self, client, demo=False, managed_session=False):
        super().__init__()
        self.setWindowTitle('Bunker Mini · 垃圾桶搜索与停靠'+(' · SIMULATED' if demo else ''))
        self.resize(1440,960);self.setMinimumSize(1000,740)
        self.demo,self.last_snapshot,self.connected,self.last_received=demo,{},False,0.
        self.managed_session=managed_session and not demo
        self.closing=False;self.allow_close=False;self.seen_events=set();self.operation_id=None;self.ambiguous_action=None
        self.worker=BackendWorker(client)
        self.worker.snapshot.connect(self.receive_latest);self.worker.response.connect(self.action_result);self.worker.disconnected.connect(self.offline)
        central=QtWidgets.QWidget();self.setCentralWidget(central)
        layout=QtWidgets.QVBoxLayout(central);layout.setContentsMargins(14,12,14,10);layout.setSpacing(8)
        title=QtWidgets.QHBoxLayout()
        self.headline=QtWidgets.QLabel('BUNKER MINI  /  任务控制台');self.headline.setObjectName('headline')
        self.connection=QtWidgets.QLabel('SIMULATED · 无 ROS / 无 CAN' if demo else '正在连接本机服务');self.connection.setObjectName('muted')
        title.addWidget(self.headline);title.addStretch();title.addWidget(self.connection);layout.addLayout(title)
        status=QtWidgets.QHBoxLayout()
        self.state=QtWidgets.QLabel('等待就绪');self.state.setObjectName('state')
        self.speed=QtWidgets.QLabel('实际速度：等待里程计');self.can=QtWidgets.QLabel('CAN：未知');self.driver=QtWidgets.QLabel('底盘通信：未知')
        self.authorization=QtWidgets.QLabel('运动权限：未知')
        for label in (self.state,self.speed,self.can,self.driver,self.authorization):status.addWidget(label)
        status.addStretch();layout.addLayout(status)
        self.reason=QtWidgets.QLabel('安全状态：等待');self.reason.setWordWrap(True);layout.addWidget(self.reason)
        buttons=QtWidgets.QHBoxLayout();buttons.setSpacing(6);self.buttons={}
        for action,text in ACTIONS:
            button=QtWidgets.QPushButton(text);button.setMinimumHeight(37);button.setEnabled(action=='mission_stop')
            if action=='mission_stop':button.setObjectName('stopButton');button.setMinimumWidth(150)
            if action=='arm':button.setObjectName('armButton')
            button.clicked.connect(lambda checked=False,a=action:self.request_action(a))
            self.buttons[action]=button;buttons.addWidget(button)
        layout.addLayout(buttons)
        self.operation=QtWidgets.QLabel('手动开始任务；F11 全屏 / Esc 退出全屏。'+('关闭窗口会停止任务并退出全部后台。' if self.managed_session else '此监视窗口关闭不停止任务。'));self.operation.setObjectName('muted');self.operation.setWordWrap(True);layout.addWidget(self.operation)
        self.session_line=QtWidgets.QLabel('自动准备中…' if self.managed_session else '')
        self.session_line.setWordWrap(True);layout.addWidget(self.session_line)
        self.health_line=QtWidgets.QLabel('传感器：等待数据');self.health_line.setWordWrap(True);layout.addWidget(self.health_line)
        splitter=QtWidgets.QSplitter(QtCore.Qt.Vertical)
        views=QtWidgets.QWidget();grid=QtWidgets.QGridLayout(views);grid.setContentsMargins(0,0,0,0);grid.setSpacing(8)
        self.panels={}
        for i,(key,title_text) in enumerate([('rgb','RGB · 同帧候选'),('depth','对齐深度 · 距离依据'),('lidar','LiDAR · 车体俯视'),('map','在线 SLAM · 已知空间导航')]):
            panel=Panel(title_text,key);self.panels[key]=panel;grid.addWidget(panel,i//2,i%2)
        splitter.addWidget(views)
        timeline_box=QtWidgets.QWidget();tl=QtWidgets.QVBoxLayout(timeline_box);tl.setContentsMargins(0,4,0,0)
        tl.addWidget(QtWidgets.QLabel('事件时间线 · 检测 / VLM / 本地决策'))
        self.timeline=QtWidgets.QTreeWidget();self.timeline.setHeaderLabels(['时间','来源','事件 / 判断依据','关联 ID / 延迟'])
        self.timeline.setRootIsDecorated(False);self.timeline.setAlternatingRowColors(True);self.timeline.setMinimumHeight(100)
        self.timeline.setColumnWidth(0,85);self.timeline.setColumnWidth(1,90);self.timeline.setColumnWidth(2,650)
        tl.addWidget(self.timeline);splitter.addWidget(timeline_box);splitter.setSizes([630,170]);layout.addWidget(splitter,1)
        QtWidgets.QShortcut(QtGui.QKeySequence('F11'),self,activated=self.toggle_fullscreen)
        QtWidgets.QShortcut(QtGui.QKeySequence('Escape'),self,activated=self.exit_fullscreen)
        self.watchdog=QtCore.QTimer(self);self.watchdog.timeout.connect(self.check_connection);self.watchdog.start(250)
        self.worker.start()

    def toggle_fullscreen(self):
        self.showNormal() if self.isFullScreen() else self.showFullScreen()

    def exit_fullscreen(self):
        if self.isFullScreen():self.showNormal()

    def request_action(self,action):
        if action!='mission_stop' and not self.buttons[action].isEnabled():return
        self.ambiguous_action=None
        self.worker.action(action)
        self.operation.setText('正在请求：'+dict(ACTIONS).get(action,action)+'；以后台结果为准。')
        if action!='mission_stop':self.buttons[action].setEnabled(False)

    def action_result(self,action,result):
        ok=result.get('ok',result.get('accepted',False)) is True
        unknown=result.get('outcome')=='unknown'
        self.ambiguous_action=action if unknown else None
        detail=result.get('message',result.get('error',result.get('reason','')))
        path=result.get('path',result.get('export_path',''))
        op=result.get('operation_id',result.get('id',''))
        if op:self.operation_id=str(op)
        self.operation.setText(('结果未知' if unknown else '已提交' if ok else '未执行')+'：'+dict(ACTIONS).get(action,action)+'  '+str(detail)+('  操作 '+str(op) if op else '')+('  '+str(path) if path else ''))
        self.append_event({'stamp':time.time(),'kind':'system','message':self.operation.text()})

    def offline(self,reason):
        self.connected=False
        self.connection.setText('本机服务断开 · '+reason)
        self.connection.setStyleSheet('color:'+PALETTE['bad'])
        self.state.setText('状态未知 / 服务断开')
        self.can.setText('CAN：未知')
        self.driver.setText('底盘通信：未知')
        self.authorization.setText('运动权限：未知')
        self.authorization.setStyleSheet('color:'+PALETTE['warn'])
        self.health_line.setText('传感器：连接中断，保留图像仅供查看，不代表当前观测。')
        self.speed.setText('实际速度：不可用（连接中断）')
        self.reason.setText('安全状态不可用；停止请求仍可提交。请检查本机后台。')
        for key,button in self.buttons.items():button.setEnabled(key=='mission_stop')
        for panel in self.panels.values():panel.health.setText('断流 / 数据已过期')

    def check_connection(self):
        if self.connected and time.monotonic()-self.last_received>1.2:self.offline('快照超时')
        if self.managed_session:
            try:
                value=json.loads((Path('/run/user')/str(os.getuid())/'robot-session.json').read_text())
                self.session_line.setText(value.get('message',''))
                if value.get('phase')=='closed':
                    self.allow_close=True;self.close()
                elif value.get('phase')=='shutdown_failed':self.closing=False
            except (OSError,ValueError):pass

    def receive_latest(self):
        queued=self.worker.latest_snapshot.take()
        if queued is None:return
        received,snapshot=queued
        if time.monotonic()-received>1.2:
            self.offline('界面待处理快照已过期')
            return
        self.receive(snapshot)

    def receive(self,snapshot):
        if not isinstance(snapshot,dict):return
        self.last_snapshot=snapshot;self.connected=True;self.last_received=time.monotonic()
        self.connection.setText('SIMULATED · 无 ROS / 无 CAN' if self.demo else ('本机服务已连接 · '+('仅监视' if snapshot.get('monitor_only') else '受控服务')+' · 5 Hz'))
        self.connection.setStyleSheet('color:'+PALETTE['warn' if self.demo else 'ok'])
        mission=snapshot.get('mission',{});state=current_mission_state(snapshot)
        self.state.setText(STATE_NAMES.get(state,state) if state else '任务状态：过期 / 未知')
        self.state.setToolTip('任务状态 ROS 更新时间：'+str(mission.get('stamp','未知')))
        velocity=velocity_text(snapshot.get('chassis',{}))
        self.speed.setText('实测 %.3f m/s  |  %.3f rad/s  · %.0f ms' % (velocity[0],velocity[1],velocity[2]*1000) if velocity else '实际速度：等待 / 过期（未知）')
        can=snapshot.get('can',{})
        if not isinstance(can,dict):can={'state':'UNKNOWN'}
        can_state=str(can.get('state','UNKNOWN'));interface=str(can.get('interface','接口未知'))
        bus=str(can.get('bus_state') or '总线状态未知')
        self.can.setText(interface+'：'+can_state+' / '+bus)
        chassis=snapshot.get('chassis',{})
        self.driver.setText('底盘通信：'+('正常' if chassis.get('communication_ok') is True else '未连接 / 未验证'))
        safety=snapshot.get('safety',{})
        authorization=authorization_value(snapshot)
        self.authorization.setText('运动权限：'+('已允许' if authorization is True else '已撤销' if authorization is False else '未知 / 过期'))
        self.authorization.setStyleSheet('color:'+PALETTE['ok' if authorization is True else 'warn'])
        safety_current=feed_fresh(snapshot.get('sensors',{}),'safety')
        authorization_reason=str(safety.get('authorization_reason','未提供')) if safety_current else '安全状态已过期，不能确认授权'
        protection=str(safety.get('reason','等待')) if safety_current else '安全节点状态未知 / 过期'
        if safety_current and safety.get('motion_blocked'):
            hit=safety.get('collision') or {}
            protection='障碍保护：零速等待可通行空间，运动许可保留（'+str(hit.get('sensor','传感器'))+'）'
        mission_reason=str(mission.get('reason','等待手动开始')) if state else '任务状态已过期，不能显示旧状态为当前状态'
        planning=mission.get('planning_status',{}) if state else {}
        if planning:
            descriptions={'missing_map_tf':'等待该图像时刻的地图坐标变换',
                          'target_not_observed':'等待已确认目标的当前观测',
                          'target_observation_stale':'目标观测已过期',
                          'docking_unknown_cell':'停靠车体范围仍有未知栅格',
                          'docking_occupied_cell':'停靠车体范围与地图占用栅格重叠',
                          'path_unknown_cell':'规划路径经过未知栅格',
                          'path_occupied_cell':'规划路径与地图占用栅格重叠',
                          'planner_requested':'正在请求导航路径',
                          'planner_response_timeout':'规划响应超时，请查看代价地图传感器状态',
                          'planner_no_path':'规划器未返回可通行路径',
                          'planner_rejected':'规划请求被拒绝','path_valid':'路径验证通过'}
            reasons=list(dict.fromkeys(descriptions.get(v.get('reason'),v.get('reason','等待')) for v in planning.values()))
            mission_reason+=' · '+' / '.join(reasons)
        self.authorization.setToolTip('安全状态 ROS 更新时间：'+str(safety.get('stamp','未知'))+'\n'+authorization_reason)
        self.reason.setText('保护：'+protection+'   |   授权原因：'+authorization_reason+'   |   任务：'+mission_reason)
        controls=snapshot.get('controls',{})
        for action,button in self.buttons.items():
            enabled,reason=control_enabled(controls,action)
            if action in ('arm','mission_start') and (not safety_current or not current_mission_state(snapshot)):
                enabled,reason=False,'任务或安全状态过期，等待实时状态回读。'
            button.setEnabled(enabled or action=='mission_stop');button.setToolTip(reason)
        sensors=snapshot.get('sensors',{})
        health=[];updated=[]
        for key in ('rgb','depth','lidar','odom','tf','map','localization','nav2','safety'):
            raw=sensors.get(key,sensors.get('scan',{}) if key=='lidar' else {})
            state,age,hz,reason=sensor_text(raw)
            age_text='--' if age is None else '%.0f ms' % (age*1000) if age<1 else '%.1f s' % age
            hz_text='--' if hz is None else '%.1f Hz' % hz
            text=SENSOR_NAMES.get(key,key)+' '+HEALTH_NAMES.get(state,state)+' · '+age_text+' / '+hz_text
            health.append(text)
            source_key='safety' if key=='localization' else 'mission' if key=='nav2' else key
            stamp=raw.get('stamp',sensors.get(source_key,{}).get('stamp')) if isinstance(raw,dict) else None
            if not finite_number(stamp):stamp=snapshot.get(source_key,{}).get('stamp') if isinstance(snapshot.get(source_key,{}),dict) else None
            update_text=('%.6f' % stamp) if finite_number(stamp) else '尚无更新时间'
            tooltip=SENSOR_NAMES.get(key,key)+' · ROS 更新时间 '+update_text+' · '+reason
            updated.append(tooltip)
            if key in self.panels:
                self.panels[key].health.setText(HEALTH_NAMES.get(state,state)+' · '+age_text+' / '+hz_text)
                self.panels[key].health.setToolTip(tooltip)
        self.health_line.setText('   |   '.join(health))
        self.health_line.setToolTip('\n'.join(updated))
        for panel in self.panels.values():panel.view.update_data(snapshot)
        frame=snapshot.get('frames',{}).get('rgb',{})
        rows=matching_candidates(frame,snapshot.get('candidates',{}))
        self.panels['rgb'].detail.setText('同帧候选 %d  ·  目标轨迹 %s' % (len(rows),mission.get('target_track_id') or '尚未选定'))
        target=selected_target(snapshot)
        optical=target.get('optical_point',[])
        z=optical[2] if len(optical)>=3 and finite_number(optical[2]) else None
        gap=target.get('gap');bearing=target.get('bearing')
        gap_text='%.3f m' % gap if finite_number(gap) else '--'
        z_text='%.3f m' % z if z is not None else '--'
        target_age=target.get('age')
        if not finite_number(target_age) and finite_number(snapshot.get('stamp')) and finite_number(target.get('stamp')):
            target_age=snapshot['stamp']-target['stamp']
        target_timeout=snapshot.get('safety',{}).get('freshness_limits',{}).get('target_timeout',.5)
        stale=not finite_number(target_age) or not 0<=target_age<=target_timeout
        validity=('距离观测已过期 / 时间不可用' if stale else '当前定位有效' if target.get('valid') is True
                  else str(target.get('reason','尚无选定目标距离')))
        if stale and finite_number(gap):gap_text+='（过期）'
        desired_gap=mission.get('docking_gap',.10)
        desired_text='%.0f cm' % (desired_gap*100) if finite_number(desired_gap) else '--'
        depth_detail='光轴深度 Z：'+z_text+'  |  相机测得车头净距：'+gap_text+'\n物理车头 x=0.345 m；停车目标 '+desired_text+'。'+validity
        locked_geometry=mission.get('locked_geometry') or {}
        if finite_number(locked_geometry.get('gap')):
            depth_detail+='\n导航地图估计净距：%.3f m（不代表当前相机测量）' % locked_geometry['gap']
        if mission.get('completion_evidence')=='locked_map_only':
            depth_detail+='\n已到停靠点并停稳；实时净距未复核。'
        if mission.get('navigation_mode')=='adaptive_map':
            statuses={'waiting_for_depth':'等待有效深度，保持上次目标','waiting_for_second_observation':'等待连续观测',
                      'ambiguous_target':'目标关联不唯一，保持上次目标','correction_below_deadband':'目标已稳定',
                      'updated_goal_not_known_free':'修正点不可通行，保持上次目标',
                      'update_requested':'正在更新导航目标','update_accepted':'导航目标已更新',
                      'update_rejected':'目标更新被拒绝','update_transport_error':'目标更新通信异常'}
            depth_detail+='\n目标修正 #%s：%s' % (mission.get('target_revision',0),statuses.get(mission.get('refinement_status'),'等待目标'))
        self.panels['depth'].detail.setText(depth_detail)
        lidar_detail='物理轮廓 690 × 570 mm；目标物仍参与碰撞保护。'
        pipeline=snapshot.get('lidar_health',{})
        counters=pipeline.get('counters',{})
        if counters:
            names={'receiving':'正在接收','raw_cloud_waiting':'等待原始点云','projection_delayed':'点云转换延迟'}
            lidar_detail+='\n'+names.get(pipeline.get('reason'),str(pipeline.get('reason','')))
            lidar_detail+=' · 输入 %s / 扫描 %s · 过期丢帧 %s · TF 等待 %s' % (
                counters.get('received',0),counters.get('published',0),counters.get('expired',0),counters.get('tf_unavailable',0))
            lidar_detail+='\n累计孤立点过滤 %s · 连续观测保留稀疏点 %s' % (
                counters.get('noise_removed',0),counters.get('sparse_repeated',0))
        self.panels['lidar'].detail.setText(lidar_detail)
        metadata=(snapshot.get('map') or {})
        self.panels['map'].detail.setText('地图 %s × %s  ·  %.3f m/格  ·  geometry_epoch %s' % (metadata.get('width','--'),metadata.get('height','--'),metadata.get('resolution',0.),mission.get('geometry_epoch',0)))
        for event in snapshot.get('events',[]):
            if isinstance(event,dict):self.append_event(event)
        operation=snapshot.get('operations')
        if isinstance(operation,dict) and operation and self.ambiguous_action is None:
            current=operation.get(self.operation_id) if self.operation_id else None
            if not isinstance(current,dict):
                current=operation.get('current')
            if not isinstance(current,dict):
                current=operation if 'message' in operation else next(reversed(operation.values()),{})
            if isinstance(current,dict) and current.get('message'):
                outcome={'pending':'处理中','success':'已确认','failed':'失败','unknown':'结果未知'}.get(current.get('result'),current.get('phase','状态'))
                action=dict(ACTIONS).get(current.get('action'),current.get('action','操作'))
                self.operation.setText(action+' · '+outcome+' · '+str(current['message']))

    def append_event(self,event):
        key=str(event.get('id',event.get('sequence','')))+json.dumps(event,sort_keys=True,default=str)
        if key in self.seen_events:return
        self.seen_events.add(key)
        stamp=event.get('stamp',event.get('time',time.time()))
        timestamp=time.strftime('%H:%M:%S',time.localtime(stamp)) if finite_number(stamp) else str(stamp)
        category=timeline_category(event);label={'DETECTION':'检测','VLM':'VLM','LOCAL':'本地决策','SYSTEM':'系统'}[category]
        message=str(event.get('message',event.get('reason',event.get('event',event.get('kind','')))))
        ids=[]
        for k,title in [('task_generation','task'),('geometry_epoch','geo'),('observation_id','obs'),('request_id','req')]:
            if event.get(k) is not None and event.get(k)!='':ids.append(title+'='+str(event[k]))
        latency=event.get('latency',event.get('elapsed'))
        if finite_number(latency):ids.append('%.0f ms' % (latency*1000))
        statuses=['候选 '+str(row.get('id',''))+' '+str(row['status']) for row in event.get('candidates',[]) if isinstance(row,dict) and row.get('status')]
        if statuses:message+=' · '+', '.join(statuses)
        if event.get('error'):message+=' · '+str(event['error'])
        item=QtWidgets.QTreeWidgetItem([timestamp,label,message,'  '.join(ids)])
        item.setToolTip(2,json.dumps(event,ensure_ascii=False,indent=2,default=str));item.setToolTip(3,'\n'.join(ids))
        if event.get('error') or event.get('level')=='error':item.setForeground(2,QColor(PALETTE['bad']))
        self.timeline.insertTopLevelItem(0,item)
        while self.timeline.topLevelItemCount()>300:self.timeline.takeTopLevelItem(self.timeline.topLevelItemCount()-1)
        if len(self.seen_events)>3000:self.seen_events={key}

    def closeEvent(self,event):
        if not self.managed_session or self.allow_close:
            event.accept();return
        event.ignore()
        if self.closing:return
        self.closing=True
        self.session_line.setText('正在停止任务和底盘，关闭全部后台；请稍候。')
        runtime=Path('/run/user')/str(os.getuid())
        (runtime/'robot-session-stop').write_text('close\n')
        # Foreground launcher normally owns cleanup. If its terminal was killed,
        # the window can still invoke the same fixed, serialized shutdown path.
        alive=False
        try:
            record=json.loads((runtime/'robot-task-start.json').read_text()).get('launcher',{})
            fields=(Path('/proc')/str(record['pid'])/'stat').read_text().rsplit(')',1)[1].split()
            alive=fields[0]!='Z' and fields[19]==record['start_ticks']
        except (OSError,ValueError,KeyError):pass
        if not alive:
            workspace=Path.home()/'robot_nav_ws'
            with (workspace/'console_exports/shutdown.log').open('a') as log:
                subprocess.Popen([str(workspace/'scripts/stop_robot.sh'),'--all'],
                    stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)


def style():
    return '''
    QWidget { background:#101923; color:#e6edf3; font-family:Sans; font-size:13px; }
    QLabel#headline { font-size:21px; font-weight:600; }
    QLabel#state { color:#67d6ed; font-size:20px; font-weight:600; min-width:170px; }
    QLabel#muted { color:#96a9bb; font-size:12px; }
    QFrame#panel { background:#172431; border:1px solid #334555; border-radius:7px; }
    QFrame#panel QLabel { background:transparent; border:0; }
    QLabel#panelTitle { font-size:14px; font-weight:600; }
    QPushButton { border:1px solid #4a6276; border-radius:5px; padding:5px 8px; background:#263a4d; }
    QPushButton:hover { background:#36516b; }
    QPushButton:disabled { color:#667887; border-color:#2b3b48; background:#1b2936; }
    QPushButton#stopButton { background:#b52f40; border-color:#ef6677; color:white; font-weight:700; }
    QPushButton#stopButton:hover { background:#d43c4e; }
    QPushButton#armButton { border-color:#be9350; }
    QTreeWidget { background:#14212d; alternate-background-color:#192a39; border:1px solid #334555; }
    QHeaderView::section { background:#203344; color:#c4d3df; padding:5px; border:0; }
    QSplitter::handle { background:#2b3e50; height:5px; }
    QToolTip { background:#25394c; color:white; border:1px solid #668298; }
    '''


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket');parser.add_argument('--demo',action='store_true')
    parser.add_argument('--managed-session',action='store_true')
    parser.add_argument('--fullscreen',action='store_true');parser.add_argument('--screenshot')
    parser.add_argument('--quit-after',type=float,default=0.)
    options=parser.parse_args(args)
    app=QtWidgets.QApplication([]);app.setStyleSheet(style())
    if options.demo:
        from .demo import FakeClient
        client=FakeClient()
    else:client=UnixClient(options.socket)
    window=ConsoleWindow(client,demo=options.demo,managed_session=options.managed_session)
    window.showFullScreen() if options.fullscreen else window.show()
    if options.screenshot:
        QtCore.QTimer.singleShot(1200,lambda:window.grab().save(options.screenshot))
    if options.quit_after>0:QtCore.QTimer.singleShot(int(options.quit_after*1000),window.close)
    try:return app.exec_()
    finally:window.worker.stop()


if __name__=='__main__':
    raise SystemExit(main())
