#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')

        # The lidar is mounted low (below the 70 mm table borders), so some beams
        # hit the robot's own wheels. Readings closer than this are dropped.
        self.declare_parameter('self_filter_range', 0.20)
        self.self_filter_range = self.get_parameter('self_filter_range').value

        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)

        self.get_logger().info(
            f'Fixing /scan frame_id to lidar_link -> /scan_fixed '
            f'(dropping readings under {self.self_filter_range:.2f} m)')

    def callback(self, msg):
        msg.header.frame_id = 'lidar_link'
        msg.ranges = [float('inf') if r < self.self_filter_range else r for r in msg.ranges]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()