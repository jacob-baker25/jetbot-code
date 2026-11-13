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


class DeviceTimeMapper:
    """One clock anchor per attachment; delayed delivery never changes time.

    First callback receipt approximates the first acquisition's ROS time.
    This does not estimate USB latency or compensate long-term clock skew.
    Host clock steps and device resets require restarting qualification.
    """

    def __init__(self):
        self.anchor = None
        self.last_device_ms = None

    def stamp(self, device_ms, receipt_ros_ns, receipt_mono_s, clock_uncertainty_s=0.0):
        if not all(math.isfinite(v) for v in (device_ms, receipt_ros_ns, receipt_mono_s, clock_uncertainty_s)):
            raise ValueError("nonfinite device/receipt time")
        if clock_uncertainty_s < 0:
            raise ValueError("negative clock sampling uncertainty")
        if self.last_device_ms is not None and device_ms <= self.last_device_ms:
            raise ValueError("device timestamp did not increase; restart IMU qualification")
        if self.anchor is None:
            self.anchor = (device_ms, receipt_ros_ns, receipt_mono_s, clock_uncertainty_s)
        device_anchor, ros_anchor, mono_anchor, anchor_uncertainty = self.anchor
        # Compare two HOST clocks, not sample age. Queue backlog is not a
        # clock discontinuity and must never trigger a re-anchor.
        clock_offset_change_s = ((receipt_ros_ns - ros_anchor) / 1e9 -
                                 (receipt_mono_s - mono_anchor))
        if abs(clock_offset_change_s) > .25 + anchor_uncertainty + clock_uncertainty_s:
            raise ValueError("host ROS clock changed relative to monotonic time; restart IMU qualification")
        self.last_device_ms = device_ms
        return anchored_stamp_ns(device_ms, device_anchor, ros_anchor)


def split_stamp_ns(stamp_ns):
    """Split nonnegative nanoseconds into ROS Time sec/nanosec fields."""

    if stamp_ns < 0:
        raise ValueError("stamp must be nonnegative")
    return stamp_ns // 1000000000, stamp_ns % 1000000000
