#!/usr/bin/env python3
"""Experimental planar odometry: RF2O translation with integrated IMU yaw."""

import copy
import math
import threading
import time

from imu_yaw_odometry_core import (
    ImuTimingError,
    PlanarYawFusion,
    StartupAwareWatchdog,
    StreamTimingError,
    TimestampSynchronizer,
    YawIntegrator,
)


IMU_TOPIC = "/imu/data_raw"
RAW_ODOM_TOPIC = "/odom_rf2o_raw"
OUTPUT_ODOM_TOPIC = "/odom_rf2o"
MAX_IMU_GAP_S = 0.10
MAX_IMU_STALE_S = 0.50
MAX_ODOM_STALE_S = 1.00
MAX_FUSION_WAIT_S = 1.00
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
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = Node("imu_yaw_odometry")
    imu_node = Node("imu_yaw_odometry_imu_input")
    state_lock = threading.RLock()
    publisher = node.create_publisher(Odometry, OUTPUT_ODOM_TOPIC, 10)
    broadcaster = TransformBroadcaster(node)
    integrator = YawIntegrator(max_gap_s=MAX_IMU_GAP_S)
    fusion = PlanarYawFusion()
    synchronizer = TimestampSynchronizer(max_pending=100)
    odom_watchdog = StartupAwareWatchdog(
        time.monotonic(),
        stable_samples=ODOM_STABLE_SAMPLES,
        startup_timeout_s=ODOM_STARTUP_TIMEOUT_S,
        stale_timeout_s=MAX_ODOM_STALE_S,
    )
    state = {
        "fatal": None,
        "last_imu_receipt": None,
        "last_raw_odom_receipt": None,
        "last_fused_receipt": None,
        "raw_received": 0,
        "published": 0,
        "too_old_drops": 0,
    }

    def publish_fused(message, imu_yaw, imu_rate):
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
        state["last_fused_receipt"] = time.monotonic()

    def process_pending():
        if state["fatal"] or len(integrator.times) < 2:
            return
        ready, too_old = synchronizer.drain(
            integrator.times[0], integrator.times[-1]
        )
        if too_old:
            state["too_old_drops"] += len(too_old)
            if odom_watchdog.ready:
                state["fatal"] = (
                    "raw odometry fell behind IMU history: dropped={} "
                    "oldest_imu={:.6f} newest_imu={:.6f}"
                ).format(
                    state["too_old_drops"], integrator.times[0],
                    integrator.times[-1]
                )
                return
        for timestamp, _receipt, message in ready:
            interpolated = integrator.interpolate(timestamp)
            if interpolated is None:
                state["fatal"] = "internal timestamp synchronization failure"
                return
            publish_fused(message, interpolated[0], interpolated[1])
            if state["fatal"]:
                return

    def imu_callback(message):
        with state_lock:
            if state["fatal"]:
                return
            if message.header.frame_id != "base_link":
                state["fatal"] = "unexpected IMU frame " + message.header.frame_id
                return
            try:
                integrator.add(message_stamp_s(message), message.angular_velocity.z)
                state["last_imu_receipt"] = time.monotonic()
                process_pending()
            except (ImuTimingError, ValueError) as exc:
                state["fatal"] = str(exc)

    def odom_callback(message):
        with state_lock:
            if state["fatal"]:
                return
            if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
                state["fatal"] = "unexpected raw odometry frames {} -> {}".format(
                    message.header.frame_id, message.child_frame_id
                )
                return
            now = time.monotonic()
            try:
                synchronizer.add(message_stamp_s(message), now, copy.deepcopy(message))
            except StreamTimingError as exc:
                state["fatal"] = str(exc)
                return
            state["raw_received"] += 1
            state["last_raw_odom_receipt"] = now
            was_ready = odom_watchdog.ready
            odom_watchdog.observe(now)
            process_pending()
            if not was_ready and odom_watchdog.ready:
                node.get_logger().info(
                    "IMU_YAW_ODOM_READY raw={} imu={} output={} tf=odom->base_link".format(
                        RAW_ODOM_TOPIC, IMU_TOPIC, OUTPUT_ODOM_TOPIC
                    )
                )

    # Use separate nodes so each subscription has an independent callback group.
    # Eloquent's single-threaded/global spin path was observed to service only
    # 10 raw-odom callbacks while RF2O published 42 messages under the 50 Hz IMU
    # load. A two-thread executor can dispatch both inputs; state_lock keeps the
    # timestamp queues and fusion state deterministic.
    odom_subscription = node.create_subscription(
        Odometry, RAW_ODOM_TOPIC, odom_callback, 10
    )
    imu_subscription = imu_node.create_subscription(
        Imu, IMU_TOPIC, imu_callback, qos_profile_sensor_data
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(imu_node)
    node.get_logger().info(
        (
            "Waiting for IMU and raw RF2O. Executor=two_node_two_thread; "
            "max IMU gap {:.3f}s; stale {:.3f}s."
        ).format(
            MAX_IMU_GAP_S, MAX_IMU_STALE_S
        )
    )
    next_report = time.monotonic() + 5.0
    exit_code = 0
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            with state_lock:
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
                pending_wait = synchronizer.oldest_wait_s(now)
                if pending_wait is not None and pending_wait > MAX_FUSION_WAIT_S:
                    newest_imu = (
                        integrator.times[-1] if integrator.times else float("nan")
                    )
                    oldest_odom = synchronizer.pending[0][0]
                    node.get_logger().error(
                        (
                            "IMU_YAW_ODOM_FATAL fusion synchronization blocked "
                            "for {:.3f}s: pending={} oldest_odom={:.6f} "
                            "newest_imu={:.6f}"
                        ).format(
                            pending_wait, len(synchronizer.pending), oldest_odom,
                            newest_imu
                        )
                    )
                    exit_code = 1
                    break
                if now >= next_report:
                    raw_age = (
                        now - state["last_raw_odom_receipt"]
                        if state["last_raw_odom_receipt"] is not None else None
                    )
                    fused_age = (
                        now - state["last_fused_receipt"]
                        if state["last_fused_receipt"] is not None else None
                    )
                    node.get_logger().info(
                        (
                            "IMU_YAW_ODOM_STATUS executor=two_node_two_thread "
                            "raw_received={} published={} pending={} "
                            "too_old_drops={} raw_age={} fused_age={}"
                        ).format(
                            state["raw_received"], state["published"],
                            len(synchronizer.pending), state["too_old_drops"],
                            "{:.3f}".format(raw_age) if raw_age is not None else "none",
                            "{:.3f}".format(fused_age) if fused_age is not None else "none",
                        )
                    )
                    next_report = now + 5.0
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(imu_node)
        executor.remove_node(node)
        executor.shutdown()
        imu_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
