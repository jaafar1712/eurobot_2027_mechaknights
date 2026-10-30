#!/usr/bin/env python3
"""
Stone accuracy check (simulation only)
======================================

Compares what stone_perception reports with the true positions Gazebo knows, and
splits the error into its two sources:

  camera error        stone position relative to the robot: measured vs true.
                      Comes from marker detection, pose estimation, camera model.
  localisation error  where the robot thinks it is on the table vs where it is.
                      Comes from odometry / SLAM, and adds to every stone.
  total error         stone position on the table (x, y): reported vs true.
                      What the strategy code actually uses.
  height error        stone centre height (z): reported vs true. Matters for
                      grabbing from above and for picking the right stack level.

Run it next to the simulation:
    ros2 run eurobot_perception stone_accuracy
"""
import math
import re
import subprocess

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from eurobot_interfaces.msg import StoneDetectionArray


def gazebo_poses(world):
    """{model name: (x, y, z, yaw)} for every model, read from Gazebo."""
    out = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', f'/world/{world}/pose/info'],
                         capture_output=True, text=True, timeout=10).stdout
    poses = {}
    for block in re.findall(r'pose \{(.*?)\n\}', out, re.S):
        name = re.search(r'name: "([^"]+)"', block)
        pos = re.search(r'position \{(.*?)\}', block, re.S)
        if not name or not pos:
            continue
        p = dict(re.findall(r'(\w): ([-\d.e+]+)', pos.group(1)))
        rot = re.search(r'orientation \{(.*?)\}', block, re.S)
        q = dict(re.findall(r'(\w): ([-\d.e+]+)', rot.group(1))) if rot else {}
        qz, qw = float(q.get('z', 0)), float(q.get('w', 1))
        poses[name.group(1)] = (float(p.get('x', 0)), float(p.get('y', 0)),
                                float(p.get('z', 0)), 2 * math.atan2(qz, qw))
    return poses


def to_robot(point, robot):
    """Express a table-frame (x, y) point in the robot frame (x forward, y left)."""
    rx, ry, ryaw = robot
    dx, dy = point[0] - rx, point[1] - ry
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    return np.array([c * dx - s * dy, s * dx + c * dy])


class StoneAccuracy(Node):
    def __init__(self):
        super().__init__('stone_accuracy')
        self.declare_parameter('world', 'eurobot_2027_arena')
        self.declare_parameter('robot_model', 'simple_robot')
        self.declare_parameter('period', 3.0)
        self.world = self.get_parameter('world').value
        self.robot_model = self.get_parameter('robot_model').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.latest = None
        self.history = {'camera': [], 'localisation': [], 'total': [], 'height': []}
        self.create_subscription(StoneDetectionArray, '/stones/detections', self.on_stones, 10)
        self.create_timer(self.get_parameter('period').value, self.check)
        self.get_logger().info('Comparing /stones/detections with Gazebo ground truth...')

    def on_stones(self, msg):
        self.latest = msg

    def estimated_robot_pose(self):
        t = self.tf_buffer.lookup_transform('table', 'base_link', Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def check(self):
        if self.latest is None:
            self.get_logger().info('Waiting for /stones/detections ...')
            return
        try:
            est = self.estimated_robot_pose()
        except TransformException as e:
            self.get_logger().warn(f'No TF table -> base_link yet: {e}')
            return
        truth = gazebo_poses(self.world)
        if self.robot_model not in truth:
            self.get_logger().warn('Robot not found in Gazebo (is the simulation running?)')
            return
        robot = truth[self.robot_model][0], truth[self.robot_model][1], truth[self.robot_model][3]
        stones = {n: p for n, p in truth.items() if n.startswith('stone_')}

        loc_err = math.hypot(est[0] - robot[0], est[1] - robot[1]) * 1000
        loc_yaw = math.degrees(math.atan2(math.sin(est[2] - robot[2]), math.cos(est[2] - robot[2])))
        lines = [f'robot  true ({robot[0]:.3f}, {robot[1]:.3f}, {math.degrees(robot[2]):+.1f} deg)'
                 f'  estimated ({est[0]:.3f}, {est[1]:.3f}, {math.degrees(est[2]):+.1f} deg)'
                 f'  -> localisation error {loc_err:.0f} mm, {loc_yaw:+.1f} deg']
        self.history['localisation'].append(loc_err)

        if not self.latest.stones:
            lines.append('no stones in view')
        for s in self.latest.stones:
            p = np.array([s.position.x, s.position.y, s.position.z])
            name, t = min(stones.items(), key=lambda kv: np.linalg.norm(p - np.array(kv[1][:3])))
            total = np.linalg.norm(p[:2] - np.array(t[:2])) * 1000
            # Same stone seen from the robot: measured (via estimated pose) vs true (via true pose)
            camera = np.linalg.norm(to_robot(p, est) - to_robot(t, robot)) * 1000
            height = abs(p[2] - t[2]) * 1000
            true_level = int(round((t[2] - 0.055) / 0.11)) + 1
            level = 'ok' if s.level == true_level else f'WRONG (true L{true_level})'
            lines.append(f'{name:12s} at {s.distance:.2f} m: camera {camera:4.0f} mm | '
                         f'total {total:4.0f} mm | height {height:3.0f} mm '
                         f'(z {p[2]:.3f} vs {t[2]:.3f}) | level L{s.level} {level}')
            self.history['camera'].append(camera)
            self.history['total'].append(total)
            self.history['height'].append(height)

        def stats(key):
            v = self.history[key]
            return f'mean {np.mean(v):.0f} / max {np.max(v):.0f} mm' if v else '-'
        lines.append(f'since start: camera {stats("camera")} | localisation {stats("localisation")}'
                     f' | total {stats("total")} | height {stats("height")}')
        self.get_logger().info('\n  ' + '\n  '.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = StoneAccuracy()
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
