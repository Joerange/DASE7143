"""Continuous latest-only inference; callbacks never wait for YOLO or network."""
import base64
import json
import math
import os
import queue
import threading
import time
import traceback
import uuid
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener

from .core import CandidateTracker, depth_surface, transform_matrix
from .context import ObservationContext, can_request


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        defaults = dict(model_path='~/yolo-world-agv/models/yolov8s-worldv2-trash.pt',
                        target_label='trash can', device='0', image_size=640, half=True,
                        rgb_topic='/camera/camera/color/image_raw',
                        depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
                        camera_info_topic='/camera/camera/aligned_depth_to_color/camera_info',
                        sync_slop=.05, max_frame_age=.5, capture_tf_wait=.15, depth_scale=.001, event_window=.2,
                        min_image_sharpness=12., log_directory='~/robot_nav_ws/mission_logs/perception')
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        if not 0 <= self.cfg['capture_tf_wait'] <= .3:
            raise ValueError('capture_tf_wait must be within 0..0.3 seconds')
        self.bridge, self.tf = CvBridge(), Buffer()
        self.listener = TransformListener(self.tf, self)
        self.lock = threading.RLock()
        self.latest = None
        self.observation_context = ObservationContext()
        self.generation = ''
        self.mission_received = 0.
        self.mission_state = 'INIT'
        self.tracker = CandidateTracker()
        self.confirmations = queue.SimpleQueue()
        self.results = queue.Queue(maxsize=2)
        self.events = queue.Queue(maxsize=1)
        self.log_queue = queue.Queue(maxsize=2)
        self.stop = threading.Event()
        self.model_ready, self.worker_error = False, ''
        self.data_publishers = {k: self.create_publisher(String, topic, 10) for k, topic in
                           dict(candidates='/perception/candidates', estimates='/target/estimates',
                                event='/perception/candidate_event').items()}
        self.create_subscription(String, '/mission/state', self.mission, 10)
        self.create_subscription(String, '/perception/confirmation', self.confirmation, 10)
        self.rgb_sub = Subscriber(self, Image, self.cfg['rgb_topic'], qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, self.cfg['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.info_sub = Subscriber(self, CameraInfo, self.cfg['camera_info_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub],
                                               queue_size=5, slop=self.cfg['sync_slop'])
        self.sync.registerCallback(self.observe)
        self.worker = threading.Thread(target=self.infer, daemon=True, name='latest_frame_yolo')
        self.worker.start()
        self.log_worker = threading.Thread(target=self.save_snapshots, daemon=True, name='snapshot_log')
        self.log_worker.start()
        self.create_timer(.02, self.publish_results)
        self.create_timer(1., self.health)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def mission(self, msg):
        try:
            d = json.loads(msg.data)
            with self.lock:
                previous = self.observation_context.key
                if not self.observation_context.update(d, self.now(), time.monotonic()):
                    return
                if self.observation_context.key != previous:
                    self.latest = None
                self.generation = d['task_generation']
                self.mission_state = d['state']
                self.mission_received = time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def confirmation(self, msg):
        try:
            d = json.loads(msg.data)
            with self.lock:
                if d.get('schema') == 1 and self.observation_context.matches(d):
                    self.confirmations.put(d)
        except ValueError:
            pass

    def observe(self, rgb, depth, info):
        t = stamp_seconds(rgb.header.stamp)
        age = self.now()-t
        if age < -.05 or age > self.cfg['max_frame_age']:
            return
        if (rgb.width, rgb.height) != (depth.width, depth.height) or (info.width, info.height) != (depth.width, depth.height):
            return
        # Aligned depth must declare the same optical frame/intrinsics as RGB.
        if not rgb.header.frame_id or info.header.frame_id != rgb.header.frame_id or depth.header.frame_id != rgb.header.frame_id:
            return
        with self.lock:
            capture_context = self.observation_context.key
            if not self.observation_context.accepts_capture(t):
                return
        base_tf = map_tf = None
        try:
            base_tf = matrix(self.tf.lookup_transform('base_link', rgb.header.frame_id, Time.from_msg(rgb.header.stamp)))
        except Exception:
            pass
        try:
            map_tf = matrix(self.tf.lookup_transform('map', rgb.header.frame_id, Time.from_msg(rgb.header.stamp)))
        except Exception:
            pass
        # Messages are immutable by ownership here; workers copy arrays before processing.
        with self.lock:
            if self.observation_context.key != capture_context:
                return
            self.latest = (rgb, depth, info, base_tf, map_tf, t, time.monotonic(), capture_context)

    def put_result(self, value):
        try:
            self.results.put_nowait(value)
        except queue.Full:
            try:
                self.results.get_nowait()
            except queue.Empty:
                pass
            self.results.put_nowait(value)

    def put_event(self, event):
        with self.lock:
            if (not self.observation_context.matches(event) or not self.observation_context.accepts_capture(event['stamp']) or
                    not self.observation_context.permits_event(self.now(), time.monotonic())):
                return False
            try:
                self.events.put_nowait(event)
            except queue.Full:
                # A just-invalidated geometry context may still occupy the slot.
                # Keep the new candidate; the old snapshot cannot be sent now.
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    pass
                self.events.put_nowait(event)
            return True

    def infer(self):
        try:
            os.environ.setdefault('YOLO_CONFIG_DIR', str(Path.home()/'yolo-world-agv/.ultralytics'))
            from ultralytics import YOLOWorld
            path = Path(self.cfg['model_path']).expanduser()
            if not path.is_file():
                raise RuntimeError('Configured YOLO model file does not exist')
            model = YOLOWorld(str(path))
            labels = list(dict.fromkeys([self.cfg['target_label'], 'person', 'chair', 'potted plant', 'bottle', 'backpack']))
            model.set_classes(labels)
            predict = dict(device=self.cfg['device'], imgsz=self.cfg['image_size'], conf=.20,
                           half=self.cfg['half'], agnostic_nms=True, verbose=False)
            model.predict(np.zeros((480, 640, 3), np.uint8), **predict)
            self.model_ready = True
            last_stamp, last_finish, hz = None, None, 0.
            while not self.stop.is_set():
                with self.lock:
                    item, self.latest = self.latest, None
                    generation, geometry_epoch = self.observation_context.key
                    state, reconfirm = self.mission_state, self.observation_context.reconfirm_required
                    if time.monotonic()-self.mission_received > 1.:
                        state = 'HOLD'
                if (self.tracker.generation, self.tracker.geometry_epoch) != (generation, geometry_epoch):
                    self.tracker.reset(generation, geometry_epoch)
                while not self.confirmations.empty():
                    result = self.confirmations.get_nowait()
                    if (result.get('task_generation'), result.get('geometry_epoch', 0)) == (generation, geometry_epoch):
                        statuses = {x['id']: x['status'] for x in result.get('candidates', [])}
                        self.tracker.apply(result.get('request_id'), statuses, time.monotonic(), result.get('valid') is True,
                                           result.get('error', ''), result.get('retry_after', 0.))
                if item is None:
                    self.stop.wait(.01)
                    continue
                rgb, depth_msg, info, base_tf, map_tf, stamp, received, item_context = item
                if item_context != (generation, geometry_epoch) or stamp == last_stamp:
                    continue
                if time.monotonic()-received > self.cfg['max_frame_age'] or self.now()-stamp > self.cfg['max_frame_age']:
                    continue
                last_stamp = stamp
                bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8').copy()
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').copy()
                result = model.predict(bgr, **predict)[0]
                with self.lock:
                    if self.observation_context.key != item_context:
                        continue
                # A camera callback can precede odom/SLAM TF delivery by a few
                # milliseconds. Retry the ORIGINAL timestamp after inference,
                # never substitute latest TF for the capture-time transform.
                if base_tf is None:
                    try:
                        base_tf = matrix(self.tf.lookup_transform('base_link', rgb.header.frame_id, Time.from_msg(rgb.header.stamp)))
                    except Exception:
                        pass
                # Await capture-time TF on this worker only. ROS callbacks and
                # the independent guard keep running; no latest-TF substitution.
                tf_deadline=min(time.monotonic()+self.cfg['capture_tf_wait'],
                                received+self.cfg['max_frame_age']-.02,
                                time.monotonic()+self.cfg['max_frame_age']-(self.now()-stamp)-.02)
                while map_tf is None and not self.stop.is_set():
                    try:
                        map_tf = matrix(self.tf.lookup_transform('map', rgb.header.frame_id, Time.from_msg(rgb.header.stamp)))
                    except Exception:
                        if time.monotonic()>=tf_deadline:
                            break
                        self.stop.wait(min(.005,max(0.,tf_deadline-time.monotonic())))
                now = time.monotonic()
                if self.now()-stamp > self.cfg['max_frame_age'] or now-received > self.cfg['max_frame_age']:
                    continue
                hz = (1./(now-last_finish) if last_finish else 0.) * .3 + hz*.7
                last_finish = now
                detections = []
                for box in result.boxes:
                    label = result.names[int(box.cls.item())]
                    if label != self.cfg['target_label']:
                        continue
                    bbox, score = box.xyxy[0].cpu().tolist(), float(box.conf.item())
                    # Ultralytics xyxy is already scaled to the original image.
                    x1, y1, x2, y2 = [int(v) for v in bbox]
                    crop = bgr[max(0,y1):min(bgr.shape[0],y2), max(0,x1):min(bgr.shape[1],x2)]
                    sharp = float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()) if crop.size else 0.
                    edge = x1 <= 3 or y1 <= 3 or x2 >= bgr.shape[1]-3 or y2 >= bgr.shape[0]-3
                    geom = depth_surface(depth, depth_msg.encoding, bbox, info.k, info.d,
                                         info.distortion_model, base_tf, map_tf, self.cfg['depth_scale'])
                    detections.append(dict(bbox=bbox, score=score, label=label,
                                           image_clipped=edge,
                                           image_quality=sharp, image_usable=not edge and sharp >= self.cfg['min_image_sharpness'],
                                           map_point=geom.get('map_point'), geometry=geom))
                rows = self.tracker.update(detections, now)
                observation = uuid.uuid4().hex
                envelope = dict(schema=1, stamp=stamp, depth_stamp=stamp_seconds(depth_msg.header.stamp),
                                task_generation=generation, geometry_epoch=geometry_epoch, observation_id=observation)
                candidates = []
                targets = []
                for i, row in enumerate(rows, 1):
                    candidates.append({k: row[k] for k in ('track_id','bbox','score','label','stable')})
                    candidates[-1]['id'] = i
                    targets.append(dict(row['geometry'], track_id=row['track_id'], stamp=stamp,
                                        image_clipped=row.get('image_clipped',False)))
                output = dict(candidates=dict(envelope, candidates=candidates, ready=True, processing_hz=hz),
                              estimates=dict(envelope, targets=targets))
                if generation and can_request(state, reconfirm):
                    batch = self.tracker.event(rows, now, self.cfg['event_window'])
                    if batch:
                        rid, chosen = batch
                        annotated, crops, cs = bgr.copy(), [], []
                        for row in chosen:
                            x1, y1, x2, y2 = [int(v) for v in row['bbox']]
                            cv2.rectangle(annotated, (x1,y1), (x2,y2), (0,255,0), 2)
                            cv2.putText(annotated, str(row['id']), (x1,max(y1,22)), cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
                            dx, dy = int((x2-x1)*.08), int((y2-y1)*.08)
                            crop = bgr[max(0,y1-dy):min(bgr.shape[0],y2+dy),max(0,x1-dx):min(bgr.shape[1],x2+dx)]
                            crops.append(dict(id=row['id'], jpeg=self.jpeg(crop)))
                            cs.append({k: row[k] for k in ('id','track_id','bbox','score','label','stable')})
                        event = dict(envelope, request_id=rid, candidates=cs,
                                     rgb_jpeg=self.jpeg(annotated), crops=crops,
                                     camera_frame=rgb.header.frame_id, depth_encoding=depth_msg.encoding,
                                     depth_scale_m=self.cfg['depth_scale'] if depth_msg.encoding == '16UC1' else 1.,
                                     camera_info=dict(width=info.width, height=info.height, k=list(info.k),
                                                      d=list(info.d), distortion_model=info.distortion_model),
                                     base_from_camera=base_tf.tolist() if base_tf is not None else None,
                                     map_from_camera=map_tf.tolist() if map_tf is not None else None)
                        if not self.put_event(event):
                            continue
                        try:
                            self.log_queue.put_nowait((event, bgr, depth))
                        except queue.Full:
                            self.get_logger().warning('Snapshot log queue full; record skipped')
                self.put_result(output)
        except Exception as exc:
            # Avoid printing library/network internals or credentials.
            site = traceback.extract_tb(exc.__traceback__)[-1]
            self.worker_error = f'{type(exc).__name__} at {Path(site.filename).name}:{site.lineno} ({site.name})'
            self.model_ready = False
            self.get_logger().error('Perception worker stopped: '+self.worker_error)

    def save_snapshots(self):
        directory = Path(self.cfg['log_directory']).expanduser()
        while not self.stop.is_set() or not self.log_queue.empty():
            try:
                event, rgb, depth = self.log_queue.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                directory.mkdir(parents=True, exist_ok=True)
                prefix = directory/event['request_id']
                # Exact frozen acquisition data, not newer frames from the latest slot.
                np.savez_compressed(str(prefix)+'.npz', depth=depth, rgb=rgb)
                Path(str(prefix)+'.json').write_text(json.dumps(event, allow_nan=False))
            except Exception as exc:
                self.get_logger().warning('Snapshot logging failed: '+type(exc).__name__)

    @staticmethod
    def jpeg(image):
        good, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY,85])
        if not good:
            raise ValueError('JPEG encode failure')
        return base64.b64encode(data).decode('ascii')

    def publish_results(self):
        while not self.events.empty():
            event = self.events.get_nowait()
            if (self.observation_context.matches(event) and self.observation_context.accepts_capture(event['stamp']) and
                    self.observation_context.permits_event(self.now(), time.monotonic()) and 0 <= self.now()-event['stamp'] <= 1.):
                self.data_publishers['event'].publish(String(data=json.dumps(event, allow_nan=False)))
        while not self.results.empty():
            result = self.results.get_nowait()
            for topic, data in result.items():
                if not self.observation_context.matches(data) or not self.observation_context.accepts_capture(data['stamp']):
                    continue
                if self.now()-data['stamp'] > self.cfg['max_frame_age']:
                    continue
                self.data_publishers[topic].publish(String(data=json.dumps(data, allow_nan=False)))

    def health(self):
        if not self.model_ready:
            self.data_publishers['candidates'].publish(String(data=json.dumps(dict(
                schema=1, stamp=self.now(), task_generation=self.generation, geometry_epoch=self.observation_context.geometry_epoch, observation_id='',
                candidates=[], ready=False, processing_hz=0., error=self.worker_error or 'warming_up'))))

    def destroy_node(self):
        self.stop.set()
        self.worker.join(timeout=2.)
        self.log_worker.join(timeout=2.)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
