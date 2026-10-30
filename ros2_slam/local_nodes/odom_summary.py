#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomSummary(Node):

    def __init__(self):
        super().__init__('odom_summary')

        self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.odom_callback,
            10
        )

    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion to yaw angle in radians.
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return yaw_rad

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        yaw_rad = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        yaw_deg = math.degrees(yaw_rad)

        print(
            '\r'
            'x: {:+.3f} m | '
            'y: {:+.3f} m | '
            'yaw: {:+.1f} deg'.format(
                x,
                y,
                yaw_deg
            ),
            end='',
            flush=True
        )


def main(args=None):
    rclpy.init(args=args)

    node = OdomSummary()

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
