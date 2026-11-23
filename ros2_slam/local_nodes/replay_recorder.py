#!/usr/bin/env python3
"""Bounded out-of-process ROS replay recorder for the Eloquent Jetson.

Callbacks only convert messages to plain values and enqueue them without
waiting. A dedicated writer owns SQLite. Queue overflow is counted and makes
the capture fail closed at shutdown.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import sqlite3
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


VERSION = 1
QUEUE_MESSAGES = 4096


def stamp(message):
    return {
        "sec": int(message.header.stamp.sec),
        "nanosec": int(message.header.stamp.nanosec),
        "frame_id": message.header.frame_id,
    }


def vector(value):
    return {"x": float(value.x), "y": float(value.y), "z": float(value.z)}


def quaternion(value):
    return {
        "x": float(value.x), "y": float(value.y),
        "z": float(value.z), "w": float(value.w),
    }


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def laser_scan(message):
    return {
        "header": stamp(message),
        "angle_min": float(message.angle_min),
        "angle_max": float(message.angle_max),
        "angle_increment": float(message.angle_increment),
        "time_increment": float(message.time_increment),
        "scan_time": float(message.scan_time),
        "range_min": float(message.range_min),
        "range_max": float(message.range_max),
        # The bridge uses +inf for bins without a return. JSON null preserves
        # that state compactly; the replay reader restores null to +inf.
        "ranges": [finite_or_none(value) for value in message.ranges],
        "intensities": [finite_or_none(value) for value in message.intensities],
    }


def odometry(message):
    return {
        "header": stamp(message),
        "child_frame_id": message.child_frame_id,
        "position": vector(message.pose.pose.position),
        "orientation": quaternion(message.pose.pose.orientation),
        "pose_covariance": [float(value) for value in message.pose.covariance],
        "linear": vector(message.twist.twist.linear),
        "angular": vector(message.twist.twist.angular),
        "twist_covariance": [float(value) for value in message.twist.covariance],
    }


def imu(message):
    return {
        "header": stamp(message),
        "orientation": quaternion(message.orientation),
        "orientation_covariance": [float(value) for value in message.orientation_covariance],
        "angular_velocity": vector(message.angular_velocity),
        "angular_velocity_covariance": [
            float(value) for value in message.angular_velocity_covariance],
        "linear_acceleration": vector(message.linear_acceleration),
        "linear_acceleration_covariance": [
            float(value) for value in message.linear_acceleration_covariance],
    }


def transform(value):
    if not isinstance(value, TransformStamped):
        raise TypeError("Expected TransformStamped")
    return {
        "header": stamp(value),
        "child_frame_id": value.child_frame_id,
        "translation": vector(value.transform.translation),
        "rotation": quaternion(value.transform.rotation),
    }


def transforms(message):
    return {"transforms": [transform(value) for value in message.transforms]}


class DatabaseWriter(threading.Thread):
    def __init__(self, database, messages):
        threading.Thread.__init__(self, name="replay-database-writer")
        self.database = database
        self.messages = messages
        self.counts = {}
        self.errors = []

    def run(self):
        connection = None
        try:
            connection = sqlite3.connect(str(self.database))
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=NORMAL")
            connection.execute(
                "CREATE TABLE messages ("
                "sequence INTEGER PRIMARY KEY AUTOINCREMENT, "
                "receipt_unix REAL NOT NULL, receipt_mono REAL NOT NULL, "
                "topic TEXT NOT NULL, payload TEXT NOT NULL)")
            connection.execute("CREATE INDEX messages_topic ON messages(topic)")
            pending = 0
            while True:
                item = self.messages.get()
                if item is None:
                    break
                receipt_unix, receipt_mono, topic, payload = item
                connection.execute(
                    "INSERT INTO messages(receipt_unix, receipt_mono, topic, payload) "
                    "VALUES (?, ?, ?, ?)",
                    (receipt_unix, receipt_mono, topic,
                     json.dumps(payload, separators=(",", ":"), allow_nan=False)))
                self.counts[topic] = self.counts.get(topic, 0) + 1
                pending += 1
                if pending >= 100:
                    connection.commit()
                    pending = 0
            connection.commit()
        except Exception as exc:
            self.errors.append(repr(exc))
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception as exc:
                    self.errors.append(repr(exc))


class ReplayRecorder(Node):
    def __init__(self, output, odom_mode):
        Node.__init__(self, "jetbot_replay_recorder")
        self.output = output
        self.messages = queue.Queue(maxsize=QUEUE_MESSAGES)
        self.writer = DatabaseWriter(output / "capture.sqlite3", self.messages)
        self.received = {}
        self.first_topics = set()
        self.drops = 0
        self.maximum_queue = 0
        self.start_unix = time.time()
        self.start_mono = time.monotonic()
        # Eloquent Node already exposes a read-only `subscriptions` property.
        # Keep our handles under a private name, and do not start the non-daemon
        # writer until every subscription has been constructed successfully.
        self._replay_subscriptions = []
        self.subscribe("/scan", LaserScan, laser_scan, qos_profile_sensor_data)
        self.subscribe("/scan_slam", LaserScan, laser_scan, qos_profile_sensor_data)
        self.subscribe("/odom_rf2o", Odometry, odometry, 100)
        self.subscribe("/tf", TFMessage, transforms, qos_profile_sensor_data)
        if odom_mode == "imu_yaw":
            self.subscribe("/imu/data_raw", Imu, imu, qos_profile_sensor_data)
            self.subscribe("/odom_rf2o_raw", Odometry, odometry, 100)
        self.writer.start()
        print("REPLAY_RECORDER_READY version={} queue_messages={} topics={}".format(
            VERSION, QUEUE_MESSAGES, ",".join(sorted(self.received))), flush=True)

    def subscribe(self, topic, message_type, encoder, qos):
        self.received[topic] = 0

        def callback(message):
            receipt_unix = time.time()
            receipt_mono = time.monotonic()
            try:
                payload = encoder(message)
                self.messages.put_nowait((receipt_unix, receipt_mono, topic, payload))
                self.received[topic] += 1
                self.maximum_queue = max(self.maximum_queue, self.messages.qsize())
                if topic not in self.first_topics:
                    self.first_topics.add(topic)
                    print("REPLAY_TOPIC_FIRST topic={}".format(topic), flush=True)
            except queue.Full:
                self.drops += 1
            except Exception as exc:
                self.writer.errors.append("callback {}: {}".format(topic, repr(exc)))

        self._replay_subscriptions.append(
            self.create_subscription(message_type, topic, callback, qos))

    def finish(self):
        try:
            self.messages.put(None, timeout=5)
        except queue.Full:
            self.writer.errors.append("queue remained full during shutdown")
        self.writer.join(timeout=30)
        if self.writer.is_alive():
            self.writer.errors.append("writer did not stop within 30 seconds")
        summary = {
            "version": VERSION,
            "start_unix": self.start_unix,
            "end_unix": time.time(),
            "duration_s": time.monotonic() - self.start_mono,
            "queue_capacity_messages": QUEUE_MESSAGES,
            "maximum_queue_messages": self.maximum_queue,
            "received_counts": self.received,
            "written_counts": self.writer.counts,
            "drops": self.drops,
            "writer_errors": self.writer.errors,
        }
        (self.output / "summary.json").write_text(
            json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print("REPLAY_RECORDER_SUMMARY " + json.dumps(summary, allow_nan=False), flush=True)
        return not self.drops and not self.writer.errors


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--odom-mode", choices=("imu_yaw", "rf2o"), required=True)
    parsed = parser.parse_args(args)
    parsed.output.mkdir()
    rclpy.init()
    node = ReplayRecorder(parsed.output, parsed.odom_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        passed = node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
