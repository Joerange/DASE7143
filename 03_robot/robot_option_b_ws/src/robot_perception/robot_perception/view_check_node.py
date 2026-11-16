"""On-demand stationary whole-view checks, independent of the control loop."""
import base64
import json
import os
import queue
import threading
import time
import urllib.request
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from .view_protocol import build_view_request,parse_view

class ViewCheckNode(Node):
    def __init__(self):
        super().__init__('view_check_node')
        self.cfg={k:self.declare_parameter(k,v).value for k,v in dict(
            target_label='trash can',model='qwen3.8-flash',request_timeout=8.,
            base_url='https://dashscope.aliyuncs.com/compatible-mode/v1',
            rgb_topic='/camera/camera/color/image_raw',log_directory='~/robot_option_b_ws/mission_logs/views').items()}
        self.request=None;self.context=None;self.busy=False;self.done=queue.SimpleQueue();self.bridge=CvBridge()
        self.publisher=self.create_publisher(String,'/exploration/view_result',10)
        self.create_subscription(String,'/exploration/view_request',self.request_cb,10)
        self.create_subscription(String,'/mission/state',self.state_cb,1)
        self.create_subscription(Image,self.cfg['rgb_topic'],self.image_cb,qos_profile_sensor_data)
        self.create_timer(.02,self.poll)

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def state_cb(self,msg):
        try:self.context=json.loads(msg.data)
        except ValueError:pass

    def request_cb(self,msg):
        try:
            req=json.loads(msg.data)
            if req.get('schema')==1 and 0<=self.now()-req['stamp']<2:
                self.request=req
        except (ValueError,KeyError,TypeError):pass

    def image_cb(self,msg):
        req=self.request;ctx=self.context
        if self.busy or not req or not ctx or ctx.get('state')!='VIEW_OBSERVE':return
        if any(req.get(k)!=ctx.get(k) for k in ('task_generation','geometry_epoch')):return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp<=req['after_stamp'] or not 0<=self.now()-stamp<.5:return
        bgr=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        if cv2.Laplacian(cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var()<12:return
        ok,jpg=cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,85])
        if not ok:return
        self.request=None;self.busy=True
        threading.Thread(target=self.infer,args=(dict(req),stamp,jpg.tobytes()),daemon=True).start()

    def infer(self,req,stamp,jpg):
        out=dict(schema=1,**{k:req[k] for k in ('request_id','task_generation','geometry_epoch')},
                 observation_stamp=stamp,status='error',error='')
        try:
            key=os.getenv('DASHSCOPE_API_KEY') or os.getenv('QWEN_API_KEY')
            if not key:raise ValueError('missing_api_key')
            endpoint=(os.getenv('QWEN_BASE_URL') or self.cfg['base_url']).rstrip('/')+'/chat/completions'
            payload=build_view_request(base64.b64encode(jpg).decode(),self.cfg['target_label'],self.cfg['model'])
            request=urllib.request.Request(endpoint,data=json.dumps(payload).encode(),method='POST',
                headers={'Authorization':'Bearer '+key,'Content-Type':'application/json'})
            with urllib.request.urlopen(request,timeout=self.cfg['request_timeout']) as response:
                body=json.loads(response.read(262144))
            choice=body['choices'][0]
            if choice.get('finish_reason')!='stop':raise ValueError('incomplete_response')
            out['status']=parse_view(choice['message']['content'])
        except Exception as exc:
            out['error']=str(exc) if isinstance(exc,ValueError) else type(exc).__name__
        try:
            directory=Path(self.cfg['log_directory']).expanduser();directory.mkdir(parents=True,exist_ok=True)
            # Local-generated UUID only, never a remote path.
            rid=''.join(c for c in req['request_id'] if c in '0123456789abcdef')[:32]
            if rid:
                (directory/(rid+'.jpg')).write_bytes(jpg)
                (directory/(rid+'.json')).write_text(json.dumps(out))
        except OSError:pass
        self.done.put(out)

    def poll(self):
        while not self.done.empty():
            out=self.done.get_nowait();self.busy=False
            out['stamp']=self.now()
            self.publisher.publish(String(data=json.dumps(out)))

def main(args=None):
    rclpy.init(args=args);node=ViewCheckNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
