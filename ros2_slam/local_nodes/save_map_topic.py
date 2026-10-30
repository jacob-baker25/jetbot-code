#!/usr/bin/env python3

import os
import sys
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapSaver(Node):

    def __init__(self, output_base):
        super().__init__('simple_map_saver')

        self.output_base = os.path.expanduser(output_base)
        self.saved = False

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        print("Waiting for /map ...")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg):
        if self.saved:
            return

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        if width == 0 or height == 0:
            print("Received an empty map. Waiting for another update...")
            return

        print()
        print("Received map:")
        print("  width:      {}".format(width))
        print("  height:     {}".format(height))
        print("  resolution: {:.6f} m/pixel".format(resolution))
        print(
            "  size:       {:.2f} m x {:.2f} m".format(
                width * resolution,
                height * resolution
            )
        )

        pgm_path = self.output_base + ".pgm"
        yaml_path = self.output_base + ".yaml"

        output_directory = os.path.dirname(
            os.path.abspath(self.output_base)
        )

        os.makedirs(output_directory, exist_ok=True)

        #
        # OccupancyGrid values:
        #
        #   -1   unknown
        #    0   free
        #  100   occupied
        #
        # Standard ROS-style map image:
        #
        #    0   black / occupied
        #  205   gray / unknown
        #  254   white / free
        #

        with open(pgm_path, "wb") as pgm_file:

            header = (
                "P5\n"
                "# CREATOR: save_map_topic.py\n"
                "{} {}\n"
                "255\n"
            ).format(width, height)

            pgm_file.write(header.encode("ascii"))

            #
            # OccupancyGrid starts at the lower-left.
            # Image files are written top-to-bottom.
            #
            for y in reversed(range(height)):
                for x in range(width):

                    value = msg.data[y * width + x]

                    if value < 0:
                        pixel = 205

                    elif value >= 65:
                        pixel = 0

                    elif value <= 19:
                        pixel = 254

                    else:
                        #
                        # Values between clearly free and clearly occupied
                        # are represented as gray.
                        #
                        pixel = 205

                    pgm_file.write(bytes([pixel]))

        origin = msg.info.origin
        origin_yaw = self.quaternion_to_yaw(
            origin.orientation
        )

        #
        # IMPORTANT:
        # These are real newline characters, not literal "\n" text.
        #
        with open(yaml_path, "w") as yaml_file:

            yaml_file.write(
                "image: {}\n".format(
                    os.path.basename(pgm_path)
                )
            )

            yaml_file.write(
                "resolution: {}\n".format(
                    resolution
                )
            )

            yaml_file.write(
                "origin: [{}, {}, {}]\n".format(
                    origin.position.x,
                    origin.position.y,
                    origin_yaw
                )
            )

            yaml_file.write(
                "negate: 0\n"
            )

            yaml_file.write(
                "occupied_thresh: 0.65\n"
            )

            yaml_file.write(
                "free_thresh: 0.196\n"
            )

        print()
        print("Saved occupancy map:")
        print("  {}".format(pgm_path))
        print("  {}".format(yaml_path))

        self.saved = True


def main():

    if len(sys.argv) != 2:

        print()
        print("Usage:")
        print(
            "  python3 save_map_topic.py "
            "/path/to/output_base"
        )
        print()
        print("Example:")
        print(
            "  python3 save_map_topic.py "
            "~/autochair_ros2_ws/maps/room_test_01"
        )

        sys.exit(1)

    output_base = sys.argv[1]

    rclpy.init()

    node = MapSaver(output_base)

    try:

        while rclpy.ok() and not node.saved:

            rclpy.spin_once(
                node,
                timeout_sec=1.0
            )

    except KeyboardInterrupt:
        print("\nCancelled.")

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
