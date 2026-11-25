#!/usr/bin/env python3
"""Replay captured SLAM inputs without reopening JetBot hardware.

This player is intentionally narrower than a rosbag replacement.  Its
``slam_inputs`` mode publishes the already-recorded ``/scan_slam`` and
``/odom_rf2o`` streams plus only the ``odom -> base_link`` transform.  It never
replays the original SLAM-owned ``map -> odom`` transform into a candidate SLAM
process.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import sqlite3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

from lidar_deskew import PoseTimeline, ScanDeskewer, pose_samples_from_connection


VERSION = 1
SLAM_INPUT_TOPICS = ("/scan_slam", "/odom_rf2o", "/tf")
NANOSECONDS = 1000000000


class ReplayFormatError(RuntimeError):
    pass


class StampMapper(object):
    """Shift and optionally scale captured ROS stamps to the replay clock."""

    def __init__(self, capture_anchor_unix, replay_anchor_unix, rate):
        if rate <= 0:
            raise ValueError("Replay rate must be positive")
        self.capture_anchor_ns = int(round(capture_anchor_unix * NANOSECONDS))
        self.replay_anchor_ns = int(round(replay_anchor_unix * NANOSECONDS))
        self.rate = float(rate)

    def map(self, stamp_payload):
        original_ns = (
            int(stamp_payload["sec"]) * NANOSECONDS
            + int(stamp_payload["nanosec"])
        )
        relative_ns = original_ns - self.capture_anchor_ns
        mapped_ns = self.replay_anchor_ns + int(round(relative_ns / self.rate))
        if mapped_ns < 0:
            raise ReplayFormatError("Mapped ROS stamp is negative")
        return divmod(mapped_ns, NANOSECONDS)


def set_header(target, payload, mapper):
    sec, nanosec = mapper.map(payload)
    target.stamp.sec = sec
    target.stamp.nanosec = nanosec
    target.frame_id = payload["frame_id"]


def set_vector(target, payload):
    target.x = float(payload["x"])
    target.y = float(payload["y"])
    target.z = float(payload["z"])


def set_quaternion(target, payload):
    target.x = float(payload["x"])
    target.y = float(payload["y"])
    target.z = float(payload["z"])
    target.w = float(payload["w"])


def restore_finite_array(values):
    return [float("inf") if value is None else float(value) for value in values]


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def stamp_seconds(header):
    return int(header["sec"]) + int(header["nanosec"]) / float(NANOSECONDS)


def quaternion_yaw(value):
    return math.atan2(
        2.0 * (float(value["w"]) * float(value["z"])
               + float(value["x"]) * float(value["y"])),
        1.0 - 2.0 * (float(value["y"]) ** 2
                     + float(value["z"]) ** 2))


class ScanRotationFilter(object):
    """Reject scans whose acquisition spans excessive recorded yaw motion."""

    def __init__(self, samples, maximum_rotation):
        if maximum_rotation <= 0:
            raise ValueError("Maximum scan rotation must be positive")
        if len(samples) < 2:
            raise ReplayFormatError(
                "Scan rotation filter requires at least two odometry samples")
        ordered = sorted((float(stamp), float(yaw)) for stamp, yaw in samples)
        self.times = [value[0] for value in ordered]
        if any(current <= previous
               for previous, current in zip(self.times, self.times[1:])):
            raise ReplayFormatError(
                "Odometry stamps must increase for scan rotation filtering")
        self.yaws = [ordered[0][1]]
        previous_raw = ordered[0][1]
        for unused_stamp, current_raw in ordered[1:]:
            delta = math.atan2(
                math.sin(current_raw - previous_raw),
                math.cos(current_raw - previous_raw))
            self.yaws.append(self.yaws[-1] + delta)
            previous_raw = current_raw
        self.maximum_rotation = float(maximum_rotation)
        self.observed = 0
        self.filtered = 0
        self.maximum_observed_rotation = 0.0

    @classmethod
    def from_connection(cls, connection, maximum_rotation):
        samples = []
        for encoded, in connection.execute(
                "SELECT payload FROM messages WHERE topic='/odom_rf2o' "
                "ORDER BY sequence"):
            payload = json.loads(encoded)
            samples.append((stamp_seconds(payload["header"]),
                            quaternion_yaw(payload["orientation"])))
        return cls(samples, maximum_rotation)

    def interpolate_yaw(self, stamp):
        upper = bisect.bisect_left(self.times, stamp)
        if upper == 0:
            return self.yaws[0]
        if upper == len(self.times):
            return self.yaws[-1]
        lower = upper - 1
        ratio = ((stamp - self.times[lower])
                 / (self.times[upper] - self.times[lower]))
        return self.yaws[lower] + ratio * (self.yaws[upper] - self.yaws[lower])

    def rotation(self, scan_payload):
        # The current bridge stamps at publication after iter_scans() has
        # returned a complete revolution, so the captured interval ends at
        # the header stamp rather than beginning there.
        end = stamp_seconds(scan_payload["header"])
        start = end - float(scan_payload["scan_time"])
        return abs(self.interpolate_yaw(end) - self.interpolate_yaw(start))

    def keep(self, scan_payload):
        rotation = self.rotation(scan_payload)
        self.observed += 1
        self.maximum_observed_rotation = max(
            self.maximum_observed_rotation, rotation)
        if rotation > self.maximum_rotation:
            self.filtered += 1
            return False
        return True

    def summary(self):
        return {
            "maximum_rotation_rad": self.maximum_rotation,
            "maximum_rotation_deg": math.degrees(self.maximum_rotation),
            "maximum_observed_rotation_rad": self.maximum_observed_rotation,
            "maximum_observed_rotation_deg": math.degrees(
                self.maximum_observed_rotation),
            "observed_scans": self.observed,
            "filtered_scans": self.filtered,
        }


class LatenessTracker(object):
    """Describe scheduler stalls without changing the replay cadence."""

    def __init__(self, limit):
        self.limit = float(limit)
        self.values = []
        self.topic_maximum = {}
        self.topic_late_counts = {}
        self.bursts = []
        self.active = None

    def observe(self, sequence, topic, relative_s, lateness_s):
        lateness_s = float(lateness_s)
        self.values.append(lateness_s)
        self.topic_maximum[topic] = max(
            lateness_s, self.topic_maximum.get(topic, 0.0))
        if lateness_s <= self.limit:
            self.close_burst()
            return
        self.topic_late_counts[topic] = self.topic_late_counts.get(topic, 0) + 1
        if self.active is None:
            self.active = {
                "start_sequence": int(sequence),
                "end_sequence": int(sequence),
                "start_relative_s": float(relative_s),
                "end_relative_s": float(relative_s),
                "message_count": 0,
                "start_topic": topic,
                "maximum_lateness_s": lateness_s,
                "maximum_sequence": int(sequence),
                "maximum_topic": topic,
            }
        self.active["end_sequence"] = int(sequence)
        self.active["end_relative_s"] = float(relative_s)
        self.active["message_count"] += 1
        if lateness_s > self.active["maximum_lateness_s"]:
            self.active["maximum_lateness_s"] = lateness_s
            self.active["maximum_sequence"] = int(sequence)
            self.active["maximum_topic"] = topic

    def close_burst(self):
        if self.active is not None:
            self.bursts.append(self.active)
            self.active = None

    def summary(self):
        self.close_burst()
        return {
            "scheduled_messages": len(self.values),
            "lateness_limit_s": self.limit,
            "median_lateness_s": percentile(self.values, 0.50),
            "p95_lateness_s": percentile(self.values, 0.95),
            "p99_lateness_s": percentile(self.values, 0.99),
            "maximum_lateness_s": max(self.values or [0.0]),
            "late_messages": sum(self.topic_late_counts.values()),
            "late_burst_count": len(self.bursts),
            "late_counts_by_topic": self.topic_late_counts,
            "maximum_lateness_by_topic_s": self.topic_maximum,
            "late_bursts": self.bursts,
        }


def decode_laser_scan(payload, mapper):
    message = LaserScan()
    set_header(message.header, payload["header"], mapper)
    for name in (
            "angle_min", "angle_max", "angle_increment", "time_increment",
            "scan_time", "range_min", "range_max"):
        setattr(message, name, float(payload[name]))
    message.ranges = restore_finite_array(payload["ranges"])
    message.intensities = restore_finite_array(payload["intensities"])
    return message


def decode_odometry(payload, mapper):
    message = Odometry()
    set_header(message.header, payload["header"], mapper)
    message.child_frame_id = payload["child_frame_id"]
    set_vector(message.pose.pose.position, payload["position"])
    set_quaternion(message.pose.pose.orientation, payload["orientation"])
    message.pose.covariance = [float(value) for value in payload["pose_covariance"]]
    set_vector(message.twist.twist.linear, payload["linear"])
    set_vector(message.twist.twist.angular, payload["angular"])
    message.twist.covariance = [
        float(value) for value in payload["twist_covariance"]]
    return message


def decode_imu(payload, mapper):
    message = Imu()
    set_header(message.header, payload["header"], mapper)
    set_quaternion(message.orientation, payload["orientation"])
    message.orientation_covariance = [
        float(value) for value in payload["orientation_covariance"]]
    set_vector(message.angular_velocity, payload["angular_velocity"])
    message.angular_velocity_covariance = [
        float(value) for value in payload["angular_velocity_covariance"]]
    set_vector(message.linear_acceleration, payload["linear_acceleration"])
    message.linear_acceleration_covariance = [
        float(value) for value in payload["linear_acceleration_covariance"]]
    return message


def decode_transforms(payload, mapper, allowed_edges=None):
    message = TFMessage()
    message.transforms = []
    for value in payload["transforms"]:
        edge = (value["header"]["frame_id"], value["child_frame_id"])
        if allowed_edges is not None and edge not in allowed_edges:
            continue
        transform = TransformStamped()
        set_header(transform.header, value["header"], mapper)
        transform.child_frame_id = value["child_frame_id"]
        set_vector(transform.transform.translation, value["translation"])
        set_quaternion(transform.transform.rotation, value["rotation"])
        message.transforms.append(transform)
    return message


def open_read_only(database):
    database = Path(database).resolve()
    if not database.is_file():
        raise ReplayFormatError("Replay database does not exist: {}".format(database))
    return sqlite3.connect(database.as_uri() + "?mode=ro", uri=True)


def validate_database(database, required_topics=SLAM_INPUT_TOPICS):
    connection = open_read_only(database)
    try:
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise ReplayFormatError("SQLite integrity check failed: {}".format(integrity))
        count, minimum, maximum = connection.execute(
            "SELECT COUNT(*), MIN(sequence), MAX(sequence) FROM messages").fetchone()
        if not count:
            raise ReplayFormatError("Replay database contains no messages")
        if minimum != 1 or maximum != count:
            raise ReplayFormatError(
                "Replay sequences are not contiguous: count={} min={} max={}".format(
                    count, minimum, maximum))
        rows = connection.execute(
            "SELECT topic, COUNT(*) FROM messages GROUP BY topic").fetchall()
        counts = {topic: topic_count for topic, topic_count in rows}
        missing = [topic for topic in required_topics if counts.get(topic, 0) == 0]
        if missing:
            raise ReplayFormatError(
                "Replay database is missing required topics: {}".format(
                    ",".join(missing)))
        odom_tf_count = 0
        for payload_text, in connection.execute(
                "SELECT payload FROM messages WHERE topic='/tf'"):
            payload = json.loads(payload_text)
            odom_tf_count += sum(
                1 for value in payload.get("transforms", [])
                if value.get("header", {}).get("frame_id") == "odom"
                and value.get("child_frame_id") == "base_link")
        if not odom_tf_count:
            raise ReplayFormatError(
                "Replay database has no odom -> base_link transforms")
        first = connection.execute(
            "SELECT receipt_unix, receipt_mono FROM messages ORDER BY sequence LIMIT 1"
        ).fetchone()
        last_mono = connection.execute(
            "SELECT receipt_mono FROM messages ORDER BY sequence DESC LIMIT 1"
        ).fetchone()[0]
        return {
            "rows": count,
            "topic_counts": counts,
            "odom_to_base_transforms": odom_tf_count,
            "first_receipt_unix": float(first[0]),
            "first_receipt_mono": float(first[1]),
            "duration_s": float(last_mono - first[1]),
        }
    except sqlite3.Error as exc:
        raise ReplayFormatError("Invalid replay database: {}".format(exc))
    finally:
        connection.close()


class ReplayPlayer(Node):
    def __init__(self):
        Node.__init__(self, "jetbot_replay_player")
        self._replay_publishers = {
            "/scan_slam": self.create_publisher(
                LaserScan, "/scan_slam", qos_profile_sensor_data),
            "/odom_rf2o": self.create_publisher(Odometry, "/odom_rf2o", 100),
            # TF listeners request reliable dynamic transforms.  A depth value
            # gives the Eloquent default reliable/volatile publisher behavior.
            "/tf": self.create_publisher(TFMessage, "/tf", 100),
        }

    def publish_payload(self, topic, payload, mapper):
        if topic == "/scan_slam":
            message = decode_laser_scan(payload, mapper)
        elif topic == "/odom_rf2o":
            message = decode_odometry(payload, mapper)
        elif topic == "/tf":
            message = decode_transforms(
                payload, mapper, allowed_edges={("odom", "base_link")})
            if not message.transforms:
                return False
        else:
            return False
        self._replay_publishers[topic].publish(message)
        return True


def replay(database, node, rate=1.0, start_delay=2.0, linger=1.0,
           maximum_lateness=0.25, maximum_scan_rotation=None,
           deskew_scans=False, laser_x=0.0, laser_y=0.0,
           laser_yaw=math.pi):
    metadata = validate_database(database)
    connection = open_read_only(database)
    counts = {topic: 0 for topic in SLAM_INPUT_TOPICS}
    filtered_tf = 0
    timing = LatenessTracker(maximum_lateness)
    rotation_filter = None
    deskewer = None
    try:
        if maximum_scan_rotation is not None:
            rotation_filter = ScanRotationFilter.from_connection(
                connection, maximum_scan_rotation)
        if deskew_scans:
            deskewer = ScanDeskewer(
                PoseTimeline(pose_samples_from_connection(connection)),
                laser_x=laser_x, laser_y=laser_y, laser_yaw=laser_yaw)
        print(
            "REPLAY_PLAYER_READY version={} mode=slam_inputs rate={} rows={} "
            "duration_s={:.3f}".format(
                VERSION, rate, metadata["rows"], metadata["duration_s"]),
            flush=True)
        time.sleep(start_delay)
        start_mono = time.monotonic()
        start_unix = time.time()
        mapper = StampMapper(metadata["first_receipt_unix"], start_unix, rate)
        print("REPLAY_PLAYER_START", flush=True)
        cursor = connection.execute(
            "SELECT sequence, receipt_mono, topic, payload FROM messages "
            "WHERE topic IN ('/scan_slam','/odom_rf2o','/tf') ORDER BY sequence")
        for sequence, receipt_mono, topic, encoded in cursor:
            relative_s = float(receipt_mono) - metadata["first_receipt_mono"]
            target = start_mono + relative_s / rate
            remaining = target - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            lateness = max(0.0, time.monotonic() - target)
            timing.observe(sequence, topic, relative_s, lateness)
            payload = json.loads(encoded)
            if (topic == "/scan_slam" and rotation_filter is not None
                    and not rotation_filter.keep(payload)):
                continue
            if topic == "/scan_slam" and deskewer is not None:
                payload = deskewer.deskew(payload)
            published = node.publish_payload(topic, payload, mapper)
            if published:
                counts[topic] += 1
            elif topic == "/tf":
                filtered_tf += 1
        time.sleep(linger)
    finally:
        connection.close()
    timing_summary = timing.summary()
    for burst in timing_summary["late_bursts"]:
        print("REPLAY_LATE_BURST " + json.dumps(burst, sort_keys=True), flush=True)
    summary = {
        "version": VERSION,
        "mode": "slam_inputs",
        "rate": float(rate),
        "published_counts": counts,
        "filtered_tf_messages": filtered_tf,
        "maximum_lateness_s": timing_summary["maximum_lateness_s"],
        "lateness_limit_s": maximum_lateness,
        "late_messages": timing_summary["late_messages"],
        "timing": timing_summary,
        "scan_rotation_filter": (
            rotation_filter.summary() if rotation_filter is not None else None),
        "scan_deskew": (
            deskewer.summary() if deskewer is not None else None),
    }
    print("REPLAY_PLAYER_SUMMARY " + json.dumps(summary, sort_keys=True), flush=True)
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-delay", type=float, default=2.0)
    parser.add_argument("--linger", type=float, default=1.0)
    parser.add_argument("--maximum-lateness", type=float, default=0.25)
    parser.add_argument("--maximum-scan-rotation-degrees", type=float)
    parser.add_argument("--deskew-scans", action="store_true")
    parser.add_argument("--laser-x", type=float, default=0.0)
    parser.add_argument("--laser-y", type=float, default=0.0)
    parser.add_argument("--laser-yaw", type=float, default=math.pi)
    parsed = parser.parse_args(args)
    if parsed.rate <= 0 or parsed.start_delay < 0 or parsed.linger < 0:
        parser.error("rate must be positive; delays must be nonnegative")
    if parsed.maximum_lateness <= 0:
        parser.error("maximum lateness must be positive")
    if (parsed.maximum_scan_rotation_degrees is not None
            and not 0 < parsed.maximum_scan_rotation_degrees <= 180):
        parser.error("maximum scan rotation must be in (0, 180] degrees")
    if (parsed.deskew_scans
            and parsed.maximum_scan_rotation_degrees is not None):
        parser.error("deskew and scan-rotation filtering are mutually exclusive")

    metadata = validate_database(parsed.database)
    if metadata["duration_s"] / parsed.rate > 3600:
        parser.error("replay would exceed one hour")
    rclpy.init()
    node = ReplayPlayer()
    try:
        summary = replay(
            parsed.database, node, parsed.rate, parsed.start_delay,
            parsed.linger, parsed.maximum_lateness,
            (math.radians(parsed.maximum_scan_rotation_degrees)
             if parsed.maximum_scan_rotation_degrees is not None else None),
            parsed.deskew_scans, parsed.laser_x, parsed.laser_y,
            parsed.laser_yaw)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if not summary["late_messages"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
