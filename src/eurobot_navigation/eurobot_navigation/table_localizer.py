#!/usr/bin/env python3
"""
Table localizer — corrects the robot's position with the fixed ArUco markers
printed on the playmat (IDs 20-23, see field.TABLE_MARKERS).

    table ──(this node)──> map ──(SLAM)──> odom ──(wheels)──> base_footprint

SLAM keeps the motion smooth but drifts over a match. Whenever the camera sees a
table marker, this node measures where the robot really is on the table and
nudges the table -> map transform towards it. Nav2 works in `map`, so it never
sees a jump; only the link between the rule-book frame and SLAM's frame moves.

How a measurement works: the markers lie flat on the table (z = 0) and the camera
mounting is known from TF, so each marker corner seen in the image can be
projected onto the table surface. That gives the corners in the robot frame.
Matching them to the corners' known table positions (a 2D rigid fit) gives the
robot's x, y and heading directly, which is more robust than ArUco's own 3D pose
for flat markers seen at a shallow angle.

Publishes
  TF  table -> map
  /localization/marker_pose  geometry_msgs/PoseStamped  each accepted measurement
"""
import math

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from eurobot_navigation import field

# OpenCV optical frame (x right, y down, z forward) -> camera_link (x forward, y left, z up)
OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    """2D pose composition a * b, poses as (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def fit_rigid_2d(robot_pts, table_pts):
    """Robot pose (x, y, yaw) in the table such that table = R(yaw) * robot + (x, y)."""
    P, Q = np.asarray(robot_pts), np.asarray(table_pts)
    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    H = (P - pc).T @ (Q - qc)
    yaw = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    t = qc - R @ pc
    residual = float(np.sqrt(np.mean(np.sum((P @ R.T + t - Q) ** 2, axis=1))))
    return (float(t[0]), float(t[1]), yaw), residual


class TableLocalizer(Node):
    def __init__(self):
        super().__init__('table_localizer')
        self.declare_parameter('start_x', 0.25)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('rate', 5.0)                 # measurements per second (max)
        self.declare_parameter('max_distance', 1.3)         # ignore markers further than this (m)
        self.declare_parameter('max_residual', 0.02)        # reject fits worse than this (m)
        self.declare_parameter('max_jump', 0.30)            # reject corrections bigger than this (m)
        self.declare_parameter('max_jump_yaw', 0.35)        # ... or this (rad, ~20 deg)
        self.declare_parameter('blend', 0.3)                # 0..1, how much one measurement moves the estimate
        self.declare_parameter('detection_scales', [0.5, 0.25])

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.camera_frame, self.robot_frame = p('camera_frame'), p('robot_frame')
        self.period = 1.0 / p('rate')
        self.max_distance, self.max_residual = p('max_distance'), p('max_residual')
        self.max_jump, self.max_jump_yaw = p('max_jump'), p('max_jump_yaw')
        self.blend, self.scales = p('blend'), list(p('detection_scales'))

        # SLAM's map frame starts at the robot's start pose
        self.table_to_map = (p('start_x'), p('start_y'), p('start_yaw'))
        self.accepted = 0
        self.last_processed = None

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.aruco_params = aruco.DetectorParameters_create()   # OpenCV < 4.7 (Ubuntu 22.04)
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters()
        self.camera_matrix, self.dist_coeffs = None, np.zeros(5)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Image, '/camera', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera_info', self.on_camera_info, 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/localization/marker_pose', 10)
        self.create_timer(0.05, self.publish_tf)
        self.get_logger().info(
            f'Table localizer: markers {sorted(field.TABLE_MARKERS)}, start pose {self.table_to_map}')

    def on_camera_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'table', 'map'
        t.transform.translation.x, t.transform.translation.y = self.table_to_map[:2]
        t.transform.rotation.z = math.sin(self.table_to_map[2] / 2)
        t.transform.rotation.w = math.cos(self.table_to_map[2] / 2)
        self.tf_broadcaster.sendTransform(t)

    def lookup(self, target, source, stamp):
        return self.tf_buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=0.1))

    # ------------------------------------------------------------------ measurement

    def detect(self, gray):
        found = {}
        for scale in self.scales:
            img = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            corners, ids, _ = aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
            if ids is None:
                continue
            for c, marker_id in zip(corners, ids.flatten()):
                if int(marker_id) in field.TABLE_MARKERS and int(marker_id) not in found:
                    found[int(marker_id)] = c.reshape(4, 2) / scale
        return found

    def project_to_table_surface(self, pixels, R, t):
        """Pixels -> points on the table surface (z = 0), in the robot frame."""
        undist = cv2.undistortPoints(pixels.reshape(-1, 1, 2).astype(np.float64),
                                     self.camera_matrix, self.dist_coeffs).reshape(-1, 2)
        points = []
        for u, v in undist:
            ray = R @ (OPTICAL_TO_LINK @ np.array([u, v, 1.0]))
            if ray[2] >= -1e-6:
                return None          # ray does not hit the floor
            s = -t[2] / ray[2]
            points.append((t + s * ray)[:2])
        return np.array(points)

    def on_image(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.camera_matrix is None or (
                self.last_processed is not None
                and (stamp - self.last_processed).nanoseconds < self.period * 1e9):
            return
        self.last_processed = stamp

        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        markers = self.detect(gray)
        if not markers:
            return
        try:
            cam = self.lookup(self.robot_frame, self.camera_frame, stamp)
            map_robot = self.lookup('map', self.robot_frame, stamp)
        except TransformException as e:
            self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=5.0)
            return
        R = quaternion_to_matrix(cam.transform.rotation)
        t = np.array([cam.transform.translation.x, cam.transform.translation.y,
                      cam.transform.translation.z])

        robot_pts, table_pts = [], []
        for marker_id, corners in markers.items():
            pts = self.project_to_table_surface(corners, R, t)
            if pts is None or np.linalg.norm(pts.mean(axis=0)) > self.max_distance:
                continue
            robot_pts.extend(pts)
            table_pts.extend(field.table_marker_corners(marker_id))
        if not robot_pts:
            return

        measured, residual = fit_rigid_2d(robot_pts, table_pts)   # robot pose in table
        if residual > self.max_residual:
            self.get_logger().debug(f'Rejected marker fit (residual {residual * 1000:.0f} mm)')
            return

        q = map_robot.transform.rotation
        in_map = (map_robot.transform.translation.x, map_robot.transform.translation.y,
                  2 * math.atan2(q.z, q.w))
        target = compose(measured, inverse(in_map))                # table -> map that fits it
        current_robot = compose(self.table_to_map, in_map)
        jump = math.hypot(measured[0] - current_robot[0], measured[1] - current_robot[1])
        jump_yaw = abs(wrap(measured[2] - current_robot[2]))
        if jump > self.max_jump or jump_yaw > self.max_jump_yaw:
            self.get_logger().warn(
                f'Ignored marker fix: {jump * 1000:.0f} mm / {math.degrees(jump_yaw):.1f} deg '
                f'away from the current estimate', throttle_duration_sec=2.0)
            return

        a = self.blend
        x, y, yaw = self.table_to_map
        self.table_to_map = (x + a * (target[0] - x), y + a * (target[1] - y),
                             wrap(yaw + a * wrap(target[2] - yaw)))
        self.accepted += 1
        self.get_logger().info(
            f'Marker fix #{self.accepted} from {sorted(markers)}: correction {jump * 1000:.0f} mm, '
            f'{math.degrees(jump_yaw):.1f} deg (fit residual {residual * 1000:.1f} mm)',
            throttle_duration_sec=1.0)

        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = msg.header.stamp, 'table'
        pose.pose.position.x, pose.pose.position.y = measured[0], measured[1]
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(measured[2] / 2), math.cos(measured[2] / 2)
        self.pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = TableLocalizer()
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
