#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rplidar import RPLidar, RPLidarException


PORT = '/dev/ttyUSB0'
BAUDRATE = 256000

NUM_BINS = 360
RANGE_MIN_M = 0.15
RANGE_MAX_M = 12.0


class RPLidarBridge(Node):

    def __init__(self):
        super().__init__('rplidar_python_bridge')

        self.publisher = self.create_publisher(
            LaserScan,
            'scan',
            10
        )

        self.lidar = RPLidar(
            PORT,
            baudrate=BAUDRATE,
            timeout=2
        )

        self.recover_lidar_state()

        info = self.lidar.get_info()
        health = self.lidar.get_health()

        self.get_logger().info(
            'RPLidar connected: {}'.format(info)
        )
        self.get_logger().info(
            'RPLidar health: {}'.format(health)
        )

        self.last_scan_time = time.time()

    def recover_lidar_state(self):
        self.get_logger().info('Resetting RPLidar to a clean state')

        try:
            self.lidar.stop()
        except Exception:
            pass

        try:
            self.lidar.stop_motor()
        except Exception:
            pass

        time.sleep(0.5)

        try:
            self.lidar.reset()
            time.sleep(1.0)
        except Exception as exc:
            self.get_logger().warn(
                'RPLidar reset warning: {}'.format(exc)
            )

        try:
            self.lidar.clean_input()
        except Exception as exc:
            self.get_logger().warn(
                'Could not clean RPLidar input buffer: {}'.format(exc)
            )

    def publish_scan(self, scan):
        now = time.time()
        scan_time = now - self.last_scan_time
        self.last_scan_time = now

        ranges = [float('inf')] * NUM_BINS
        intensities = [0.0] * NUM_BINS

        for quality, angle_deg, distance_mm in scan:
            index = int(angle_deg) % NUM_BINS
            distance_m = distance_mm / 1000.0

            if RANGE_MIN_M <= distance_m <= RANGE_MAX_M:
                ranges[index] = distance_m
                intensities[index] = float(quality)

        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / NUM_BINS

        msg.scan_time = float(scan_time)
        msg.time_increment = float(scan_time / NUM_BINS)

        msg.range_min = RANGE_MIN_M
        msg.range_max = RANGE_MAX_M

        msg.ranges = ranges
        msg.intensities = intensities

        self.publisher.publish(msg)

    def run(self):
        self.get_logger().info('Starting RPLidar scan loop')

        try:
            self.lidar.start_motor()
            time.sleep(1.5)

            self.lidar.clean_input()

            for scan in self.lidar.iter_scans():
                if not rclpy.ok():
                    break

                self.publish_scan(scan)

        except RPLidarException as exc:
            self.get_logger().error(
                'RPLidar error: {}'.format(exc)
            )

        except Exception as exc:
            self.get_logger().error(
                'Unexpected error: {}'.format(exc)
            )

        finally:
            self.shutdown_lidar()

    def shutdown_lidar(self):
        self.get_logger().info('Stopping RPLidar')

        try:
            self.lidar.stop()
        except Exception:
            pass

        try:
            self.lidar.stop_motor()
        except Exception:
            pass

        try:
            self.lidar.clean_input()
        except Exception:
            pass

        try:
            self.lidar.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = RPLidarBridge()
        node.run()

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print('Fatal error: {}'.format(exc))

    finally:
        if node is not None:
            try:
                node.shutdown_lidar()
            except Exception:
                pass

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
