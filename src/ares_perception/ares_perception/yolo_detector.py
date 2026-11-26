#!/usr/bin/env python3
"""ARES object detector: YOLOv8 on the robot camera, fused with LiDAR for distance.

Subscribes : /camera/image_raw, /scan
Publishes  : /detections/objects   std_msgs/String JSON
               {"inference_ms": 38.2, "objects": [{"cls": "person", "conf": 0.81,
                 "bearing_deg": 12.4, "dist_m": 2.35}, ...]}
             /detections           vision_msgs/Detection2DArray (standard ROS message)
             /camera/annotated     sensor_msgs/Image (boxes + class + distance drawn on)
Bearing is + to the left of the camera axis. Distance is the nearest LiDAR return
inside the middle of the bounding box, so it is measured, not guessed from box size.
"""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
except ImportError:
    Detection2DArray = None


# ---------- pure helpers (no ROS, unit-testable) ----------
def pixel_to_bearing(px, width, hfov):
    """Horizontal pixel -> bearing in radians (+ left), pinhole camera."""
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return -math.atan((px - width / 2.0) / fx)


def range_between(ranges, angle_min, inc, rmin, rmax, b0, b1):
    """Nearest valid LiDAR return between bearings b0 and b1 (radians), or None."""
    if ranges is None or len(ranges) == 0:
        return None
    lo, hi = min(b0, b1), max(b0, b1)
    i0 = int(math.floor((lo - angle_min) / inc))
    i1 = int(math.ceil((hi - angle_min) / inc))
    idx = np.arange(i0, i1 + 1) % len(ranges)
    r = ranges[idx]
    ok = np.isfinite(r) & (r >= rmin) & (r <= rmax)
    return float(r[ok].min()) if ok.any() else None


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        default_model = os.path.expanduser('~/ares_ws/models/yolov8n.pt')
        for k, v in {'model_path': default_model, 'conf': 0.40, 'rate': 5.0, 'imgsz': 640,
                     'device': 'auto', 'hfov': 1.3963, 'box_core': 0.5}.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.conf, self.imgsz, self.hfov = g('conf'), int(g('imgsz')), g('hfov')
        self.core = g('box_core')

        from ultralytics import YOLO          # imported here so a missing package gives a clear log
        import torch
        dev = g('device')
        self.device = (0 if torch.cuda.is_available() else 'cpu') if dev == 'auto' else dev
        path = g('model_path')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.get_logger().info(f'loading {path} on {"GPU" if self.device == 0 else self.device} ...')
        self.model = YOLO(path)
        self.model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=self.imgsz,
                           device=self.device, verbose=False)        # warm-up

        self.cv = CvBridge()
        self.img_msg, self.done_stamp = None, None
        self.scan = None
        self.seen_classes = set()
        self.ms_hist = []
        self.vm_ok = Detection2DArray is not None

        q = qos_profile_sensor_data
        self.create_subscription(Image, 'camera/image_raw', lambda m: setattr(self, 'img_msg', m), q)
        self.create_subscription(LaserScan, 'scan', self.on_scan, q)
        self.pub_json = self.create_publisher(String, 'detections/objects', 10)
        self.pub_img = self.create_publisher(Image, 'camera/annotated', 5)
        self.pub_vm = self.create_publisher(Detection2DArray, 'detections', 10) if self.vm_ok else None
        self.create_timer(1.0 / g('rate'), self.step)
        self.get_logger().info('YOLO detector ready')

    def on_scan(self, m):
        self.scan = (np.asarray(m.ranges, dtype=float), m.angle_min, m.angle_increment,
                     m.range_min, m.range_max)

    def step(self):
        msg = self.img_msg
        if msg is None or msg.header.stamp == self.done_stamp:
            return
        self.done_stamp = msg.header.stamp
        img = self.cv.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        t0 = time.monotonic()
        res = self.model.predict(img, conf=self.conf, imgsz=self.imgsz,
                                 device=self.device, verbose=False)[0]
        ms = (time.monotonic() - t0) * 1000
        self.ms_hist = (self.ms_hist + [ms])[-50:]

        objects = []
        annotated = res.plot()
        vm = Detection2DArray() if self.vm_ok else None
        if vm is not None:
            vm.header = msg.header
        for box in res.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            name = res.names[int(box.cls[0])]
            conf = float(box.conf[0])
            cx = (x1 + x2) / 2
            bearing = pixel_to_bearing(cx, w, self.hfov)
            half = (x2 - x1) * self.core / 2               # middle of the box only
            dist = None
            if self.scan is not None:
                b_left = pixel_to_bearing(cx - half, w, self.hfov)
                b_right = pixel_to_bearing(cx + half, w, self.hfov)
                dist = range_between(*self.scan, b_right, b_left)
            objects.append({'cls': name, 'conf': round(conf, 2),
                            'bearing_deg': round(math.degrees(bearing), 1),
                            'dist_m': None if dist is None else round(dist, 2)})
            if dist is not None:
                cv2.putText(annotated, f'{dist:.1f} m', (int(x1), int(y2) + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if name not in self.seen_classes:
                self.seen_classes.add(name)
                where = 'unknown distance' if dist is None else f'{dist:.1f} m'
                self.get_logger().info(f'first {name} detected ({conf:.2f}) at {where}, '
                                       f'{math.degrees(bearing):+.0f} deg')
            if vm is not None:
                try:
                    d = Detection2D()
                    d.header = msg.header
                    d.bbox.center.position.x, d.bbox.center.position.y = cx, (y1 + y2) / 2
                    d.bbox.size_x, d.bbox.size_y = x2 - x1, y2 - y1
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id, hyp.hypothesis.score = name, conf
                    d.results.append(hyp)
                    vm.detections.append(d)
                except AttributeError:
                    self.vm_ok, vm = False, None
                    self.get_logger().warn('vision_msgs layout differs; publishing JSON only')

        cv2.putText(annotated, f'YOLOv8n {ms:.0f} ms', (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        self.pub_json.publish(String(data=json.dumps(
            {'inference_ms': round(float(np.mean(self.ms_hist)), 1), 'objects': objects})))
        out = self.cv.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = msg.header
        self.pub_img.publish(out)
        if vm is not None:
            self.pub_vm.publish(vm)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
