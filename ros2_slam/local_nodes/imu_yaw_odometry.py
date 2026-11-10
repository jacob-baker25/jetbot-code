#!/usr/bin/env python3
"""Experimental planar odometry: RF2O translation with integrated IMU yaw."""

import copy
import math
import time

from imu_yaw_odometry_core import (
    ImuTimingError,
    PlanarYawFusion,
    StartupAwareWatchdog,
    StreamTimingError,
    YawIntegrator,
)


IMU_TOPIC = "/imu/data_raw"
RAW_ODOM_TOPIC = "/odom_rf2o_raw"
OUTPUT_ODOM_TOPIC = "/odom_rf2o"
MAX_IMU_GAP_S = 0.10
MAX_IMU_STALE_S = 0.50
MAX_ODOM_STALE_S = 1.00
ODOM_STABLE_SAMPLES = 10
ODOM_STARTUP_TIMEOUT_S = 20.0


def message_stamp_s(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def main():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = Node("imu_yaw_odometry")
    publisher = node.create_publisher(Odometry, OUTPUT_ODOM_TOPIC, 10)
    broadcaster = TransformBroadcaster(node)
    integrator = YawIntegrator(max_gap_s=MAX_IMU_GAP_S)
    fusion = PlanarYawFusion()
    odom_watchdog = StartupAwareWatchdog(
        time.monotonic(),
        stable_samples=ODOM_STABLE_SAMPLES,
        startup_timeout_s=ODOM_STARTUP_TIMEOUT_S,
        stale_timeout_s=MAX_ODOM_STALE_S,
    )
    state = {
        "fatal": None,
        "last_imu_receipt": None,
        "last_odom_stamp": None,
        "published": 0,
        "startup_odom_drops": 0,
    }

    def imu_callback(message):
        if state["fatal"]:
            return
        if message.header.frame_id != "base_link":
            state["fatal"] = "unexpected IMU frame " + message.header.frame_id
            return
        try:
            integrator.add(message_stamp_s(message), message.angular_velocity.z)
            state["last_imu_receipt"] = time.monotonic()
        except (ImuTimingError, ValueError) as exc:
            state["fatal"] = str(exc)

    def odom_callback(message):
        if state["fatal"]:
            return
        if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
            state["fatal"] = "unexpected raw odometry frames {} -> {}".format(
                message.header.frame_id, message.child_frame_id
            )
            return
        timestamp = message_stamp_s(message)
        if state["last_odom_stamp"] is not None and timestamp <= state["last_odom_stamp"]:
            state["fatal"] = "raw odometry timestamp did not increase"
            return
        state["last_odom_stamp"] = timestamp
        interpolated = integrator.interpolate(timestamp)
        if interpolated is None:
            state["startup_odom_drops"] += 1
            return
        imu_yaw, imu_rate = interpolated
        raw_pose = message.pose.pose
        try:
            x, y, yaw = fusion.update(
                raw_pose.position.x,
                raw_pose.position.y,
                quaternion_yaw(raw_pose.orientation),
                imu_yaw,
            )
        except ValueError as exc:
            state["fatal"] = str(exc)
            return

        output = copy.deepcopy(message)
        output.header.frame_id = "odom"
        output.child_frame_id = "base_link"
        output.pose.pose.position.x = x
        output.pose.pose.position.y = y
        output.pose.pose.position.z = 0.0
        output.pose.pose.orientation.x = 0.0
        output.pose.pose.orientation.y = 0.0
        output.pose.pose.orientation.z = math.sin(0.5 * yaw)
        output.pose.pose.orientation.w = math.cos(0.5 * yaw)
        output.twist.twist.angular.z = imu_rate
        publisher.publish(output)

        transform = TransformStamped()
        transform.header = copy.deepcopy(output.header)
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = copy.deepcopy(output.pose.pose.orientation)
        broadcaster.sendTransform(transform)
        state["published"] += 1
        was_ready = odom_watchdog.ready
        odom_watchdog.observe(time.monotonic())
        if not was_ready and odom_watchdog.ready:
            node.get_logger().info(
                "IMU_YAW_ODOM_READY raw={} imu={} output={} tf=odom->base_link".format(
                    RAW_ODOM_TOPIC, IMU_TOPIC, OUTPUT_ODOM_TOPIC
                )
            )

    imu_subscription = node.create_subscription(
        Imu, IMU_TOPIC, imu_callback, qos_profile_sensor_data
    )
    odom_subscription = node.create_subscription(
        Odometry, RAW_ODOM_TOPIC, odom_callback, 10
    )
    node.get_logger().info(
        "Waiting for IMU and raw RF2O. Max IMU gap {:.3f}s; stale {:.3f}s.".format(
            MAX_IMU_GAP_S, MAX_IMU_STALE_S
        )
    )
    next_report = time.monotonic() + 5.0
    exit_code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["fatal"]:
                node.get_logger().error("IMU_YAW_ODOM_FATAL " + state["fatal"])
                exit_code = 1
                break
            if (state["last_imu_receipt"] is not None and
                    time.monotonic() - state["last_imu_receipt"] > MAX_IMU_STALE_S):
                node.get_logger().error("IMU_YAW_ODOM_FATAL IMU stream became stale")
                exit_code = 1
                break
            try:
                odom_watchdog.check(time.monotonic())
            except StreamTimingError as exc:
                node.get_logger().error("IMU_YAW_ODOM_FATAL " + str(exc))
                exit_code = 1
                break
            now = time.monotonic()
            if now >= next_report:
                node.get_logger().info(
                    "IMU_YAW_ODOM_STATUS published={} startup_drops={}".format(
                        state["published"], state["startup_odom_drops"]
                    )
                )
                next_report = now + 5.0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
