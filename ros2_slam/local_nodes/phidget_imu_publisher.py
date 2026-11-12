#!/usr/bin/env python3
"""Publish qualified planar Phidget gyro Z as sensor_msgs/Imu.

This node is intentionally passive: it publishes no TF, odometry, commands,
orientation, acceleration, or magnetometer data. It always zeros the gyro while
stationary before publishing. The only qualified mounting relationship is
base_link yaw rate = -raw sensor Z rate.
"""

import argparse
import math
import queue
import time

from imu_planar import anchored_stamp_ns, robot_yaw_rate_rad_s, split_stamp_ns


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
    state = {"drops": 0, "invalid": 0, "error": None}
    node = None
    published = 0
    last_device_ms = None
    max_device_gap_ms = 0.0
    source_gaps_over_limit = 0

    def on_data(device, acceleration, angular_rate, magnetic_field, timestamp):
        if not math.isfinite(timestamp) or not math.isfinite(angular_rate[2]):
            state["invalid"] += 1
            return
        try:
            events.put_nowait((float(timestamp), float(angular_rate[2])))
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
        spatial.setOnSpatialDataHandler(on_data)
        print(
            (
                "Publishing {} in {}: robot yaw rate = -raw sensor Z. "
                "QoS=best_effort depth={}. No TF/odom."
            ).format(
                args.topic, args.frame, IMU_QOS_DEPTH
            ),
            flush=True,
        )

        start = time.monotonic()
        next_report = start + 5.0
        anchor_device_ms = None
        anchor_ros_ns = None
        last_sample = time.monotonic()
        while rclpy.ok() and (args.seconds == 0.0 or
                              time.monotonic() - start < args.seconds):
            if state["error"]:
                raise RuntimeError("Phidget error: " + state["error"])
            try:
                device_ms, raw_z_deg_s = events.get(timeout=0.2)
            except queue.Empty:
                if time.monotonic() - last_sample > 3.0:
                    raise RuntimeError("No valid Spatial sample for three seconds")
                continue
            last_sample = time.monotonic()
            if last_device_ms is not None:
                device_gap_ms = device_ms - last_device_ms
                max_device_gap_ms = max(max_device_gap_ms, device_gap_ms)
                if device_gap_ms > SOURCE_GAP_REPORT_MS:
                    source_gaps_over_limit += 1
                    print(
                        "IMU_SOURCE_GAP device_gap_ms={:.3f}".format(
                            device_gap_ms
                        ),
                        flush=True,
                    )
            last_device_ms = device_ms
            now_ns = node.get_clock().now().nanoseconds
            if anchor_device_ms is None:
                anchor_device_ms = device_ms
                anchor_ros_ns = now_ns
            stamp_ns = anchored_stamp_ns(
                device_ms, anchor_device_ms, anchor_ros_ns
            )
            # A large divergence means the anchor no longer represents the
            # current ROS clock (clock adjustment or prolonged queue backlog).
            if abs(now_ns - stamp_ns) > 1000000000:
                anchor_device_ms = device_ms
                anchor_ros_ns = now_ns
                stamp_ns = now_ns

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
            publisher.publish(msg)
            published += 1
            rclpy.spin_once(node, timeout_sec=0.0)

            now = time.monotonic()
            if now >= next_report:
                print(
                    (
                        "Published {} samples; {} queue drops; {} invalid; "
                        "source_gaps_gt_100ms={}; max_device_gap_ms={:.3f}"
                    ).format(
                        published, state["drops"], state["invalid"],
                        source_gaps_over_limit, max_device_gap_ms
                    ),
                    flush=True,
                )
                next_report = now + 5.0
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


if __name__ == "__main__":
    main()
