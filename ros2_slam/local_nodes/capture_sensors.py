#!/usr/bin/env python3
"""Bounded, passive ROS2 sensor capture. Python 3.6; no motor/TF publishers.

JSONL is diagnostic data, not a rosbag. Reception can drop messages; this tool
does not claim lossless recording or implement replay. Run in a second SSH
terminal before starting the mapping session. Ctrl-C stops only the recorder.
"""
import argparse
import collections
import json
import math
import os
import shutil
import time


def plain(value):
    """Convert ROS messages recursively without depending on rosidl_runtime_py."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)  # Preserve inf/-inf/nan distinctly in valid JSON.
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, 'get_fields_and_field_types'):
        return {key: plain(getattr(value, key))
                for key in value.get_fields_and_field_types()}
    return [plain(item) for item in value]


class BoundedWriter:
    def __init__(self, path, max_bytes, reserve_bytes):
        self.path = os.path.abspath(path)
        self.max_bytes = max_bytes
        self.reserve_bytes = reserve_bytes
        self.bytes_written = 0
        self.counts = collections.Counter()
        self.reason = None
        self.last_space_check = 0.0
        if shutil.disk_usage(os.path.dirname(self.path)).free < reserve_bytes:
            raise RuntimeError('Insufficient free space for requested reserve')
        self.stream = open(self.path, 'x', encoding='utf-8')

    def write(self, record):
        if self.reason:
            return False
        payload = json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n'
        size = len(payload.encode('utf-8'))
        if self.bytes_written + size > self.max_bytes:
            self.reason = 'size_limit'
            return False
        if time.monotonic() - self.last_space_check >= 1:
            self.last_space_check = time.monotonic()
            if shutil.disk_usage(os.path.dirname(self.path)).free < self.reserve_bytes:
                self.reason = 'free_space_reserve'
                return False
            self.stream.flush()
        self.stream.write(payload)
        self.bytes_written += size
        self.counts[record['topic']] += 1
        return True

    def close(self):
        self.stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='New .jsonl file; never overwrites')
    parser.add_argument('--imu-topic', default='/imu/data_raw')
    parser.add_argument('--seconds', type=float, default=300)
    parser.add_argument('--max-mb', type=float, default=64)
    parser.add_argument('--reserve-mb', type=float, default=150)
    parser.add_argument('--lean', action='store_true',
                        help='Record IMU plus raw/final odometry only; omit scans and TF')
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in
           (args.seconds, args.max_mb, args.reserve_mb)):
        parser.error('duration and size limits must be finite and positive')

    # ROS imports remain lazy so the data writer can be tested without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                          QoSDurabilityPolicy, QoSReliabilityPolicy)
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    writer = BoundedWriter(args.output, int(args.max_mb * 1024**2),
                           int(args.reserve_mb * 1024**2))
    rclpy.init()
    node = Node('passive_sensor_capture')
    subscriptions = []
    start = time.monotonic()
    reason = 'duration'
    def receive(topic, msg):
        writer.write({'topic': topic, 'receipt_unix_s': time.time(),
                      'receipt_ros_s': node.get_clock().now().nanoseconds / 1e9,
                      'elapsed_s': time.monotonic() - start, 'msg': plain(msg)})

    static_qos = QoSProfile(depth=100)
    static_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    static_qos.reliability = QoSReliabilityPolicy.RELIABLE
    if args.lean:
        specs = [(Odometry, '/odom_rf2o_raw', qos_profile_sensor_data),
                 (Odometry, '/odom_rf2o', qos_profile_sensor_data),
                 (Imu, args.imu_topic, qos_profile_sensor_data)]
    else:
        specs = [(LaserScan, '/scan', qos_profile_sensor_data),
                 (LaserScan, '/scan_slam', qos_profile_sensor_data),
                 (Odometry, '/odom_rf2o_raw', qos_profile_sensor_data),
                 (Odometry, '/odom_rf2o', qos_profile_sensor_data),
                 (Odometry, '/odometry/filtered', qos_profile_sensor_data),
                 (Imu, args.imu_topic, qos_profile_sensor_data),
                 (TFMessage, '/tf', qos_profile_sensor_data),
                 (TFMessage, '/tf_static', static_qos)]
    try:
        for kind, topic, qos in specs:
            subscriptions.append(node.create_subscription(
                kind, topic, lambda msg, topic=topic: receive(topic, msg), qos))
        print('Passive capture active: {}. No robot commands sent.'.format(args.output),
              flush=True)
        next_report = 5.0
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            if writer.reason:
                reason = writer.reason
                break
            elapsed = time.monotonic() - start
            if elapsed >= next_report:
                print('{:.0f}s: {} ({:.1f} MiB)'.format(
                    elapsed, dict(writer.counts), writer.bytes_written / 1024**2),
                    flush=True)
                next_report = elapsed + 5
    except KeyboardInterrupt:
        reason = 'user_stop'
    except Exception:
        reason = 'error'
        raise
    finally:
        writer.close()
        print('Capture ended: {}; messages: {}'.format(reason, dict(writer.counts)),
              flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
