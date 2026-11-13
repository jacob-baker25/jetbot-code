#!/usr/bin/env python3
"""Publish qualified planar Phidget gyro Z as sensor_msgs/Imu.

This node is intentionally passive: it publishes no TF, odometry, commands,
orientation, acceleration, or magnetometer data. It always zeros the gyro while
stationary before publishing. The only qualified mounting relationship is
base_link yaw rate = -raw sensor Z rate.
"""

import argparse
from collections import deque
import json
import math
import queue
import time

from imu_planar import DeviceTimeMapper, robot_yaw_rate_rad_s, split_stamp_ns


GYRO_Z_VARIANCE = 1.0e-5
UNUSED_AXIS_VARIANCE = 1.0e6
IMU_QOS_DEPTH = 100
SOURCE_GAP_REPORT_MS = 100.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/imu/data_raw")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--interval-ms", type=int, default=20)
    parser.add_argument("--serial", type=int, default=807962)
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="Stop after this duration; zero means run until Ctrl-C")
    args = parser.parse_args()
    if args.interval_ms <= 0:
        parser.error("--interval-ms must be positive")
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and nonnegative")

    # Lazy hardware and ROS imports keep pure conversion tests portable.
    from Phidget22.Devices.Spatial import Spatial
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from sensor_msgs.msg import Imu

    spatial = Spatial()
    events = queue.Queue(maxsize=1000)
    state = {"drops": 0, "invalid": 0, "error": None, "last_callback_mono": None}
    node = None
    published = 0
    last_device_ms = None
    max_device_gap_ms = 0.0
    source_gaps_over_limit = 0
    clock_mapper = DeviceTimeMapper()
    timing_max = {}
    timing_events = deque(maxlen=128)
    timing_event_count = 0

    def observe_timing(kind, seconds, device_ms=None):
        nonlocal timing_event_count
        timing_max[kind] = max(timing_max.get(kind, 0.0), seconds)
        if seconds > .1:
            timing_event_count += 1
            timing_events.append({"kind": kind, "seconds": seconds,
                                  "mono_s": time.monotonic(),
                                  "device_ms": device_ms})

    def on_data(device, acceleration, angular_rate, magnetic_field, timestamp):
        before_clock = time.monotonic()
        receipt_ros_ns = node.get_clock().now().nanoseconds
        after_clock = time.monotonic()
        clock_uncertainty_s = (after_clock - before_clock) / 2.0
        receipt_mono = (after_clock + before_clock) / 2.0
        previous = state["last_callback_mono"]
        callback_gap = 0.0 if previous is None else receipt_mono - previous
        state["last_callback_mono"] = receipt_mono
        if not math.isfinite(timestamp) or not math.isfinite(angular_rate[2]):
            state["invalid"] += 1
            return
        try:
            events.put_nowait((float(timestamp), float(angular_rate[2]),
                              receipt_mono, receipt_ros_ns, callback_gap, clock_uncertainty_s))
        except queue.Full:
            state["drops"] += 1

    def on_error(device, code, description):
        state["error"] = "{}: {}".format(code, description)

    try:
        spatial.setDeviceSerialNumber(args.serial)
        spatial.setOnErrorHandler(on_error)
        spatial.openWaitForAttachment(10000)
        spatial.setDataInterval(args.interval_ms)
        print(
            "Attached serial {} at {} ms. Keep robot completely stationary.".format(
                spatial.getDeviceSerialNumber(), spatial.getDataInterval()
            ),
            flush=True,
        )
        print("Zeroing gyro; waiting 3 seconds.", flush=True)
        spatial.zeroGyro()
        time.sleep(3.0)

        rclpy.init()
        node = Node("phidget_planar_imu_publisher")
        imu_qos = QoSProfile(depth=IMU_QOS_DEPTH)
        imu_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        imu_qos.durability = QoSDurabilityPolicy.VOLATILE
        publisher = node.create_publisher(Imu, args.topic, imu_qos)
        print(
            (
                "Publishing {} in {}: robot yaw rate = -raw sensor Z. "
                "QoS=best_effort depth={}. Fixed device clock anchor; quiet publication loop. No TF/odom."
            ).format(
                args.topic, args.frame, IMU_QOS_DEPTH
            ),
            flush=True,
        )
        # Do not activate delivery until startup output has completed.
        spatial.setOnSpatialDataHandler(on_data)

        start = time.monotonic()
        previous_loop = start
        last_sample = time.monotonic()
        while rclpy.ok() and (args.seconds == 0.0 or
                              time.monotonic() - start < args.seconds):
            now_mono = time.monotonic()
            observe_timing("loop_gap_s", now_mono - previous_loop)
            previous_loop = now_mono
            if state["error"]:
                raise RuntimeError("Phidget error: " + state["error"])
            try:
                device_ms, raw_z_deg_s, receipt_mono, receipt_ros_ns, callback_gap, clock_uncertainty_s = events.get(timeout=0.2)
            except queue.Empty:
                if time.monotonic() - last_sample > 3.0:
                    raise RuntimeError("No valid Spatial sample for three seconds")
                continue
            last_sample = time.monotonic()
            observe_timing("callback_gap_s", callback_gap, device_ms)
            observe_timing("clock_read_s", 2 * clock_uncertainty_s, device_ms)
            observe_timing("queue_wait_s", last_sample - receipt_mono, device_ms)
            if last_device_ms is not None:
                device_gap_ms = device_ms - last_device_ms
                max_device_gap_ms = max(max_device_gap_ms, device_gap_ms)
                if device_gap_ms > SOURCE_GAP_REPORT_MS:
                    source_gaps_over_limit += 1
                    observe_timing("device_gap_s", device_gap_ms / 1000.0, device_ms)
            last_device_ms = device_ms
            stamp_ns = clock_mapper.stamp(device_ms, receipt_ros_ns, receipt_mono, clock_uncertainty_s)
            observe_timing("sample_age_s", (node.get_clock().now().nanoseconds - stamp_ns) / 1e9, device_ms)

            msg = Imu()
            sec, nanosec = split_stamp_ns(stamp_ns)
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            msg.header.frame_id = args.frame
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.z = robot_yaw_rate_rad_s(raw_z_deg_s)
            msg.angular_velocity_covariance = [
                UNUSED_AXIS_VARIANCE, 0.0, 0.0,
                0.0, UNUSED_AXIS_VARIANCE, 0.0,
                0.0, 0.0, GYRO_Z_VARIANCE,
            ]
            msg.linear_acceleration_covariance[0] = -1.0
            before_publish = time.monotonic()
            publisher.publish(msg)
            observe_timing("publish_call_s", time.monotonic() - before_publish, device_ms)
            published += 1
            before_spin = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            observe_timing("spin_call_s", time.monotonic() - before_spin, device_ms)
    except KeyboardInterrupt:
        pass
    finally:
        spatial.close()
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (NameError, RuntimeError):
            pass
        print(
            (
                "Stopped after {} published samples; {} queue drops; {} invalid; "
                "source_gaps_gt_100ms={}; max_device_gap_ms={:.3f}. "
                "No TF/odom published."
            ).format(
                published, state["drops"], state["invalid"],
                source_gaps_over_limit, max_device_gap_ms
            ),
            flush=True,
        )
        print("IMU_TIMING_SUMMARY " + json.dumps({
            "max_seconds": timing_max, "event_count": timing_event_count,
            "retained_event_count": len(timing_events),
            "events": list(timing_events),
        }, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
