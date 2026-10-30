#!/usr/bin/env python3

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


DELAY_SECONDS = 0.30


class ScanDelayNode(Node):

    def __init__(self):
        super().__init__("scan_delay_node")

        self.queue = deque()

        self.publisher = self.create_publisher(
            LaserScan,
            "/scan_slam",
            qos_profile_sensor_data
        )

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.005,
            self.publish_ready
        )

        self.received = 0
        self.published = 0

        self.get_logger().info(
            "Delaying /scan -> /scan_slam by {:.0f} ms".format(
                DELAY_SECONDS * 1000.0
            )
        )

    def scan_callback(self, msg):

        self.received += 1

        self.queue.append(
            (
                time.monotonic(),
                msg
            )
        )

    def publish_ready(self):

        now = time.monotonic()

        while self.queue:

            received_time, msg = self.queue[0]

            if (
                now - received_time
                <
                DELAY_SECONDS
            ):
                break

            self.queue.popleft()

            # Preserve the ORIGINAL LaserScan timestamp.
            self.publisher.publish(msg)

            self.published += 1


def main():

    rclpy.init()

    node = ScanDelayNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
