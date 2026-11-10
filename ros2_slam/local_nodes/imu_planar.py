#!/usr/bin/env python3
"""Pure conversion helpers for the qualified planar Phidget mounting."""

import math


def robot_yaw_rate_rad_s(raw_sensor_z_deg_s):
    """Convert mounted Phidget raw Z rate to ROS base_link yaw rate.

    Bidirectional whole-robot tests on 2026-11-10 established that the taped,
    face-up, label-rear mounting has robot yaw = -sensor Z. ROS yaw is positive
    counterclockwise and sensor_msgs/Imu angular velocity uses radians/second.
    """

    return -math.radians(raw_sensor_z_deg_s)


def anchored_stamp_ns(device_timestamp_ms, anchor_device_ms, anchor_ros_ns):
    """Map the Phidget monotonic device clock onto a ROS clock anchor."""

    delta_ns = int(round((device_timestamp_ms - anchor_device_ms) * 1000000.0))
    return anchor_ros_ns + delta_ns


def split_stamp_ns(stamp_ns):
    """Split nonnegative nanoseconds into ROS Time sec/nanosec fields."""

    if stamp_ns < 0:
        raise ValueError("stamp must be nonnegative")
    return stamp_ns // 1000000000, stamp_ns % 1000000000
