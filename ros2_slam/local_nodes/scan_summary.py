#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSummary(Node):

    def __init__(self):
        super().__init__('scan_summary')
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def sector_min(self, msg, start_deg, end_deg):
        values = []

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue

            angle_deg = math.degrees(
                msg.angle_min + i * msg.angle_increment
            ) % 360.0

            if start_deg <= end_deg:
                inside = start_deg <= angle_deg <= end_deg
            else:
                inside = angle_deg >= start_deg or angle_deg <= end_deg

            if inside:
                values.append(distance)

        if not values:
            return None

        return min(values)

    def fmt(self, value):
        if value is None:
            return '---'
        return '{:.2f} m'.format(value)

    def scan_callback(self, msg):
        front = self.sector_min(msg, 345, 15)
        left = self.sector_min(msg, 75, 105)
        back = self.sector_min(msg, 165, 195)
        right = self.sector_min(msg, 255, 285)

        print(
            '\rFront: {:>7} | Left: {:>7} | Back: {:>7} | Right: {:>7}'.format(
                self.fmt(front),
                self.fmt(left),
                self.fmt(back),
                self.fmt(right)
            ),
            end='',
            flush=True
        )


def main(args=None):
    rclpy.init(args=args)

    node = ScanSummary()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
