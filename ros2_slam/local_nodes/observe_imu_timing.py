#!/usr/bin/env python3
"""Passive IMU timing observer for Python 3.6 / ROS2 Eloquent.

No publishers, robot commands, TF, or fusion. Best-effort reception is not a
lossless record. A gap here cannot by itself identify its upstream cause.
"""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="New JSONL file; will not overwrite")
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--quiet-memory", action="store_true",
                        help="Keep bounded records in RAM; write only after capture; no live reports")
    parser.add_argument("--include-odom", action="store_true",
                        help="Also passively timestamp raw and fused odometry arrivals")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 600:
        parser.error("--seconds must be in (0, 600]")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry

    # Exclusive creation protects previous diagnostics.
    stream = open(args.output, "x")
    rclpy.init()
    node = Node("independent_imu_timing_observer")
    qos = QoSProfile(depth=100)
    qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
    qos.durability = QoSDurabilityPolicy.VOLATILE
    buffered_records = []
    state = {"count": 0, "last_receipt": None, "last_stamp_ns": None,
             "max_receipt_gap_s": 0.0, "max_stamp_gap_s": 0.0,
             "max_spin_s": 0.0, "max_loop_gap_s": 0.0,
             "silence_reported": False}

    def record(kind, **fields):
        fields.update(kind=kind, mono_s=time.monotonic(), unix_s=time.time())
        payload = json.dumps(fields, allow_nan=False) + "\n"
        if args.quiet_memory:
            if len(buffered_records) >= 100000:
                raise RuntimeError("Observer memory record limit reached")
            buffered_records.append(payload)
        else:
            stream.write(payload)

    def report(message):
        if not args.quiet_memory:
            print(message, flush=True)

    def receive(msg):
        now = time.monotonic()
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        receipt_gap = (None if state["last_receipt"] is None else
                       now - state["last_receipt"])
        stamp_gap = (None if state["last_stamp_ns"] is None else
                     (stamp_ns - state["last_stamp_ns"]) / 1e9)
        age = (node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        record("imu", stamp_ns=stamp_ns, receipt_gap_s=receipt_gap,
               stamp_gap_s=stamp_gap, ros_age_s=age,
               yaw_rate_rad_s=float(msg.angular_velocity.z))
        if receipt_gap is not None:
            state["max_receipt_gap_s"] = max(state["max_receipt_gap_s"], receipt_gap)
        if stamp_gap is not None:
            state["max_stamp_gap_s"] = max(state["max_stamp_gap_s"], stamp_gap)
        if ((receipt_gap is not None and receipt_gap > 0.1) or
                (stamp_gap is not None and (stamp_gap > 0.1 or stamp_gap <= 0))):
            report("GAP mono={:.3f} receipt={} stamp={} age={:.3f}".format(
                now, receipt_gap, stamp_gap, age))
        state.update(last_receipt=now, last_stamp_ns=stamp_ns,
                     count=state["count"] + 1, silence_reported=False)

    subscription = node.create_subscription(Imu, "/imu/data_raw", receive, qos)
    odom_subscriptions = []
    odom_counts = {}
    odom_previous = {}

    def receive_odom(topic, msg):
        now = time.monotonic()
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        previous = odom_previous.get(topic)
        record("odom", topic=topic, stamp_ns=stamp_ns,
               receipt_gap_s=None if previous is None else now - previous[0],
               stamp_gap_s=None if previous is None else (stamp_ns - previous[1]) / 1e9,
               ros_age_s=(node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
               x_m=float(msg.pose.pose.position.x), y_m=float(msg.pose.pose.position.y),
               yaw_deg=math.degrees(math.atan2(
                   2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                          msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                   1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2 +
                                msg.pose.pose.orientation.z ** 2))))
        odom_previous[topic] = (now, stamp_ns)
        odom_counts[topic] = odom_counts.get(topic, 0) + 1

    if args.include_odom:
        for topic in ("/odom_rf2o_raw", "/odom_rf2o"):
            odom_subscriptions.append(node.create_subscription(
                Odometry, topic, lambda msg, topic=topic: receive_odom(topic, msg), qos))
    start = time.monotonic()
    next_report = start
    record("start", topic="/imu/data_raw", qos="best_effort_depth_100",
           seconds=args.seconds, observer_version=3, quiet_memory=args.quiet_memory,
           include_odom=args.include_odom)
    print("Passive observer ready. No TF or robot commands. Start mapping now.", flush=True)
    if args.quiet_memory:
        print("Quiet capture: no further output until capture ends; data saved on normal exit/Ctrl+C.", flush=True)
    previous_loop_start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            before = time.monotonic()
            loop_gap = before - previous_loop_start
            previous_loop_start = before
            state["max_loop_gap_s"] = max(state["max_loop_gap_s"], loop_gap)
            if loop_gap > 0.1:
                record("observer_loop_delay", duration_s=loop_gap)
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            spin_s = now - before
            state["max_spin_s"] = max(state["max_spin_s"], spin_s)
            if spin_s > 0.1:
                record("observer_spin_delay", duration_s=spin_s)
                report("OBSERVER_DELAY {:.3f}s".format(spin_s))
            silent_s = (None if state["last_receipt"] is None else
                        now - state["last_receipt"])
            if silent_s is not None and silent_s > 0.5 and not state["silence_reported"]:
                record("silence", duration_s=silent_s)
                report("SILENCE mono={:.3f} duration={:.3f}s".format(now, silent_s))
                state["silence_reported"] = True
            if now >= next_report:
                record("status", **state)
                report("STATUS {:.0f}s count={} max_receipt={:.3f}s "
                      "max_stamp={:.3f}s max_spin={:.3f}s".format(
                          now - start, state["count"], state["max_receipt_gap_s"],
                          state["max_stamp_gap_s"], state["max_spin_s"]))
                if not args.quiet_memory:
                    stream.flush()
                next_report = now + 5
    except KeyboardInterrupt:
        pass
    finally:
        record("stop", odom_counts=odom_counts, **state)
        if args.quiet_memory:
            stream.writelines(buffered_records)
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Saved {} IMU receipts to {}".format(state["count"], args.output), flush=True)


if __name__ == "__main__":
    main()
