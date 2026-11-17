#!/usr/bin/env python3

import time
import json
import os
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rplidar import RPLidarException
from lidar_serial_reader import BoundedRPLidar, BatchedRPLidar

from lidar_scan_geometry import (
    NUM_BINS,
    laser_scan_angles,
    ros_bin_for_hardware_angle,
)


PORT = '/dev/ttyUSB0'
BAUDRATE = 256000

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

        reader_mode = os.environ.get('JETBOT_LIDAR_READER', 'blocking')
        if reader_mode not in ('blocking', 'batched'):
            raise ValueError('Unknown lidar reader: '+reader_mode)
        reader = BatchedRPLidar if reader_mode == 'batched' else BoundedRPLidar
        self.lidar = reader(
            PORT,
            baudrate=BAUDRATE,
            timeout=2
        )
        print('LIDAR_ACQUISITION version=1 mode={} batch_bytes={} max_buffer_bytes=3000'.format(
            reader_mode, 200 if reader_mode == 'batched' else 5), flush=True)

        self.recover_lidar_state()

        info = self.lidar.get_info()
        health = self.lidar.get_health()

        self.get_logger().info(
            'RPLidar connected: {}'.format(info)
        )
        self.get_logger().info(
            'RPLidar health: {}'.format(health)
        )
        self.get_logger().info(
            'Angle conversion: hardware clockwise -> ROS counterclockwise'
        )

        self.last_scan_time = time.time()
        self._shutdown_done = False
        self.scan_delivery_max_s = 0.0
        self.publish_max_s = 0.0
        self.scan_count = 0
        self.slow_events = deque(maxlen=32)

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
            index = ros_bin_for_hardware_angle(angle_deg)
            distance_m = distance_mm / 1000.0

            if RANGE_MIN_M <= distance_m <= RANGE_MAX_M:
                ranges[index] = distance_m
                intensities[index] = float(quality)

        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        angle_min, angle_max, angle_increment = laser_scan_angles()
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment

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

            self.lidar.capture_active = True
            previous_scan = None
            for scan in self.lidar.iter_scans():
                if not rclpy.ok():
                    break
                received = time.monotonic()
                if previous_scan is not None:
                    gap = received - previous_scan
                    self.scan_delivery_max_s = max(self.scan_delivery_max_s, gap)
                    if gap > .35:
                        self.slow_events.append({"kind": "scan_delivery_gap", "seconds": gap,
                                                 "end_mono_s": received, "end_unix_s": time.time()})
                previous_scan = received
                self.publish_scan(scan)
                publish_s = time.monotonic() - received
                self.publish_max_s = max(self.publish_max_s, publish_s)
                self.scan_count += 1
                if publish_s > .1:
                    self.slow_events.append({"kind": "conversion_and_publish", "seconds": publish_s,
                                             "end_mono_s": time.monotonic(), "end_unix_s": time.time()})

        except RPLidarException as exc:
            self.get_logger().error(
                'RPLidar error: {}'.format(exc)
            )
            raise

        except Exception as exc:
            self.get_logger().error(
                'Unexpected error: {}'.format(exc)
            )
            raise

        finally:
            self.shutdown_lidar()

    def shutdown_lidar(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self.lidar.capture_active = False
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

        summary = self.lidar.timing_summary()
        summary.update(scan_count=self.scan_count, scan_delivery_max_s=self.scan_delivery_max_s,
                       conversion_publish_max_s=self.publish_max_s, slow_events=list(self.slow_events))
        print('LIDAR_TIMING_SUMMARY ' + json.dumps(summary, allow_nan=False), flush=True)


def main(args=None):
    rclpy.init(args=args)

    node = None
    exit_code = 0

    try:
        node = RPLidarBridge()
        node.run()

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print('Fatal error: {}'.format(exc))
        exit_code = 1

    finally:
        if node is not None:
            try:
                node.shutdown_lidar()
            except Exception:
                pass

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
