#!/usr/bin/env python3
"""
Stone perception — Eurobot 2027
===============================

Finds the stones (cardboard boxes, 320 x 110 x 110 mm) from the ArUco marker
(DICT_4X4_50, ID 13, 80 mm) printed on each of their 4 long faces, and works out
where each stone is on the table.

For every marker in a camera frame:
  1. detect it (at several image scales, so close-up markers are not missed),
  2. estimate its 3D pose relative to the camera (solvePnP),
  3. move from the marker to the stone centre (55 mm behind the face),
  4. transform into the `table` frame with TF (camera mount comes from the URDF),
  5. derive the stack level, the stone's yaw, and distance/bearing from the robot.

Publishes
  /stones/detections  eurobot_interfaces/StoneDetectionArray (frame: table)
  /stones/debug       sensor_msgs/Image  camera image with the detections drawn
  /stones/markers     visualization_msgs/MarkerArray  detected stones for RViz
"""
import math

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from eurobot_interfaces.msg import StoneDetection, StoneDetectionArray

# OpenCV optical frame (x right, y down, z forward) -> camera_link (x forward, y left, z up)
OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])

GREEN = (0, 220, 0)
ORANGE = (0, 165, 255)
WHITE = (255, 255, 255)
RED = (0, 0, 255)


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class StonePerception(Node):
    def __init__(self):
        super().__init__('stone_perception')

        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('table_frame', 'table')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stone_marker_id', 13)
        self.declare_parameter('marker_size', 0.08)           # black square side (m)
        self.declare_parameter('stone_size', [0.32, 0.11, 0.11])
        self.declare_parameter('detection_scales', [1.0, 0.5, 0.25])
        self.declare_parameter('horizontal_fov', 1.047)       # used until camera_info arrives
        self.declare_parameter('max_distance', 2.5)            # ignore stones further than this (m)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.camera_frame = p('camera_frame')
        self.table_frame = p('table_frame')
        self.base_frame = p('base_frame')
        self.stone_id = p('stone_marker_id')
        self.marker_size = p('marker_size')
        self.stone_length, self.stone_width, self.stone_height = p('stone_size')
        self.scales = list(p('detection_scales'))
        self.horizontal_fov = p('horizontal_fov')
        self.max_distance = p('max_distance')

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.aruco_params = aruco.DetectorParameters_create()   # OpenCV < 4.7 (Ubuntu 22.04)
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters()
        half = self.marker_size / 2
        # Corner order returned by ArUco: top-left, top-right, bottom-right, bottom-left
        self.marker_points = np.array([[-half, half, 0], [half, half, 0],
                                       [half, -half, 0], [-half, -half, 0]], dtype=np.float64)

        self.camera_matrix = None
        self.dist_coeffs = np.zeros(5)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(Image, p('camera_topic'), self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('camera_info_topic'), self.on_camera_info, 10)
        self.pub_detections = self.create_publisher(StoneDetectionArray, '/stones/detections', 10)
        self.pub_debug = self.create_publisher(Image, '/stones/debug', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '/stones/markers', 1)

        self.get_logger().info(
            f'Stone perception: ArUco ID {self.stone_id}, {self.marker_size * 1000:.0f} mm, '
            f'scales {self.scales}, positions in frame "{self.table_frame}"')

    # ------------------------------------------------------------------ camera model

    def on_camera_info(self, msg):
        if self.camera_matrix is None or msg.k[0] != self.camera_matrix[0, 0]:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
            self.get_logger().info(f'Camera intrinsics from camera_info: fx={msg.k[0]:.1f}')

    def ensure_camera_matrix(self, width, height):
        if self.camera_matrix is None:
            f = (width / 2) / math.tan(self.horizontal_fov / 2)
            self.camera_matrix = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]])

    # ------------------------------------------------------------------ detection

    def detect(self, gray):
        """Detect markers at several scales and merge them (close markers need small scales)."""
        found = []  # (marker_id, corners 4x2 in full-resolution pixels)
        for scale in self.scales:
            img = gray if scale == 1.0 else cv2.resize(gray, None, fx=scale, fy=scale,
                                                       interpolation=cv2.INTER_AREA)
            corners, ids, _ = aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
            if ids is None:
                continue
            for c, marker_id in zip(corners, ids.flatten()):
                c = c.reshape(4, 2) / scale
                size = np.linalg.norm(c[0] - c[2])
                if all(np.linalg.norm(c.mean(axis=0) - f[1].mean(axis=0)) > 0.3 * size for f in found):
                    found.append((int(marker_id), c))
        # Refine corners on the full-resolution image for an accurate pose
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        for _, c in found:
            win = int(max(2, min(10, np.linalg.norm(c[0] - c[1]) / 10)))
            refined = cv2.cornerSubPix(gray, c.astype(np.float32).reshape(-1, 1, 2), (win, win),
                                       (-1, -1), criteria)
            c[:] = refined.reshape(4, 2)
        return found

    def marker_pose(self, corners):
        ok, rvec, tvec = cv2.solvePnP(self.marker_points, corners.astype(np.float64),
                                      self.camera_matrix, self.dist_coeffs,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return (rvec, tvec) if ok else (None, None)

    def lookup(self, target, stamp):
        """Rotation and translation of camera_link in `target`, or None if TF is not ready."""
        try:
            t = self.tf_buffer.lookup_transform(target, self.camera_frame, stamp,
                                                timeout=Duration(seconds=0.05))
        except TransformException:
            try:
                t = self.tf_buffer.lookup_transform(target, self.camera_frame, Time())
            except TransformException:
                return None
        tr = t.transform.translation
        return quaternion_to_matrix(t.transform.rotation), np.array([tr.x, tr.y, tr.z])

    def to_stone(self, rvec, tvec, cam_in_table, cam_in_base):
        """Marker pose (camera optical frame) -> StoneDetection in the table frame."""
        R_marker, _ = cv2.Rodrigues(rvec)
        t_marker = tvec.reshape(3)
        normal = R_marker[:, 2]          # out of the marker face, towards the camera
        long_axis = R_marker[:, 0]       # marker x axis runs along the stone's long side
        centre_opt = t_marker - normal * (self.stone_width / 2)

        R_tc, t_tc = cam_in_table
        centre = R_tc @ (OPTICAL_TO_LINK @ centre_opt) + t_tc
        normal_t = R_tc @ (OPTICAL_TO_LINK @ normal)
        axis_t = R_tc @ (OPTICAL_TO_LINK @ long_axis)

        det = StoneDetection()
        det.position.x, det.position.y, det.position.z = (float(v) for v in centre)
        yaw = math.atan2(axis_t[1], axis_t[0])
        det.yaw = float((yaw + math.pi / 2) % math.pi - math.pi / 2)   # stone is symmetric
        det.level = max(1, int(round((centre[2] - self.stone_height / 2) / self.stone_height)) + 1)
        det.top_z = float(centre[2] + self.stone_height / 2)
        det.seen_from_top = bool(normal_t[2] > 0.7)

        R_bc, t_bc = cam_in_base
        rel = R_bc @ (OPTICAL_TO_LINK @ centre_opt) + t_bc
        det.distance = float(math.hypot(rel[0], rel[1]))
        det.bearing = float(math.degrees(math.atan2(rel[1], rel[0])))
        return det

    # ------------------------------------------------------------------ main loop

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.ensure_camera_matrix(frame.shape[1], frame.shape[0])

        stamp = Time.from_msg(msg.header.stamp)
        cam_in_table = self.lookup(self.table_frame, stamp)
        cam_in_base = self.lookup(self.base_frame, stamp)
        tf_ok = cam_in_table is not None and cam_in_base is not None

        out = StoneDetectionArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.table_frame
        debug = frame.copy()

        for marker_id, corners in self.detect(gray):
            pts = corners.astype(np.int32)
            cx, cy = (int(v) for v in corners.mean(axis=0))
            if marker_id != self.stone_id:
                cv2.polylines(debug, [pts], True, ORANGE, 2)
                self.label(debug, (cx, cy), [f'table marker {marker_id}'], ORANGE)
                continue

            rvec, tvec = self.marker_pose(corners)
            if rvec is None:
                continue
            cv2.polylines(debug, [pts], True, GREEN, 3)
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs, rvec, tvec,
                              self.marker_size * 0.5, 2)
            if not tf_ok:
                self.label(debug, (cx, cy), ['stone', 'no TF yet'], GREEN)
                continue

            det = self.to_stone(rvec, tvec, cam_in_table, cam_in_base)
            if det.distance > self.max_distance:
                continue
            det.pixel_x, det.pixel_y = float(cx), float(cy)
            out.stones.append(det)
            where = 'top' if det.seen_from_top else f'L{det.level}'
            self.label(debug, (cx, cy), [
                f'stone {where}',
                f'x={det.position.x:.3f} y={det.position.y:.3f}',
                f'z={det.position.z:.3f} top={det.top_z:.3f}',
                f'{det.distance:.2f} m  {det.bearing:+.0f} deg',
            ], GREEN)

        self.mark_stack_tops(out.stones)
        for det in out.stones:
            if det.is_top_of_stack:
                cv2.circle(debug, (int(det.pixel_x), int(det.pixel_y)), 10, (255, 0, 255), -1)

        status = f'stones: {len(out.stones)}   frame: {self.table_frame}   (pink dot = top of stack)'
        if not tf_ok:
            status += '   (waiting for TF table -> camera_link)'
        cv2.rectangle(debug, (0, 0), (debug.shape[1], 40), (0, 0, 0), -1)
        cv2.putText(debug, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    WHITE if tf_ok else RED, 2)

        self.pub_detections.publish(out)
        debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)
        self.publish_markers(out)

    def mark_stack_tops(self, stones):
        """A stone is the top of its stack if no detected stone sits on it (same x, y, higher)."""
        for s in stones:
            s.is_top_of_stack = not any(
                o is not s and o.position.z > s.position.z + self.stone_height / 2
                and math.hypot(o.position.x - s.position.x, o.position.y - s.position.y) < self.stone_width
                for o in stones)

    @staticmethod
    def label(img, origin, lines, colour):
        x, y = origin
        scale = max(0.5, img.shape[1] / 1600)
        for i, text in enumerate(lines):
            pos = (x - 70, y - 10 + i * int(28 * scale))
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 5)
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 2)

    def publish_markers(self, detections):
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, s in enumerate(detections.stones):
            m = Marker(type=Marker.CUBE, ns='detected_stones', id=i)
            m.header = detections.header
            m.pose.position = s.position
            m.pose.orientation.z = math.sin(s.yaw / 2)
            m.pose.orientation.w = math.cos(s.yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = self.stone_length, self.stone_width, self.stone_height
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.1, 0.8
            m.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(m)
        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = StonePerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
