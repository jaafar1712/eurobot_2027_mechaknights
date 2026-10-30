#!/usr/bin/env python3
"""
Draws the Eurobot 2027 table in RViz (frame `table`): outline, throne rooms,
stables and the stone quarries' start positions. Everything comes from field.py.

Published once on /field/markers (latched), so RViz shows it whenever it connects.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from eurobot_navigation import field

YELLOW = (0.97, 0.71, 0.0)   # RAL 1023
BLUE = (0.0, 0.36, 0.55)     # RAL 5017
STONE = (0.75, 0.72, 0.65)


def outline(marker_id, ns, rect, rgb, width=0.01):
    x_min, y_min, x_max, y_max = rect
    m = Marker(type=Marker.LINE_STRIP, ns=ns, id=marker_id)
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
    m.points = [Point(x=x, y=y) for x, y in
                ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max), (x_min, y_min))]
    m.pose.orientation.w = 1.0
    return m


class FieldMarkers(Node):
    def __init__(self):
        super().__init__('field_markers')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/field/markers', qos)
        self.pub.publish(self.build())
        self.get_logger().info('Table markers published on /field/markers (frame: table)')

    def build(self):
        markers = [outline(0, 'table', (0.0, 0.0, field.TABLE_LENGTH, field.TABLE_WIDTH),
                           (0.9, 0.9, 0.9), width=0.022)]
        for team, rgb in (('yellow', YELLOW), ('blue', BLUE)):
            markers.append(outline(len(markers), f'{team}_throne_room',
                                   field.own(team, field.YELLOW_THRONE_ROOM), rgb))
            for stable in field.YELLOW_STABLES:
                markers.append(outline(len(markers), f'{team}_stables', field.own(team, stable), rgb))

        length, width, height = field.STONE_SIZE
        for i, (x, y, yaw) in enumerate(field.QUARRIES):
            m = Marker(type=Marker.CUBE, ns='quarries', id=len(markers))
            m.pose.position.x, m.pose.position.y = x, y
            m.pose.position.z = height * field.STONES_PER_QUARRY / 2
            m.pose.orientation.z, m.pose.orientation.w = _yaw_to_quaternion_zw(yaw)
            m.scale.x, m.scale.y, m.scale.z = length, width, height * field.STONES_PER_QUARRY
            m.color.r, m.color.g, m.color.b, m.color.a = (*STONE, 0.5)
            markers.append(m)

        for m in markers:
            m.header.frame_id = 'table'
            m.frame_locked = True
        return MarkerArray(markers=markers)


def _yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def main():
    rclpy.init()
    node = FieldMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
