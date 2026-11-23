import importlib.util
import json
import math
from pathlib import Path
import sqlite3
import sys
import tempfile
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def vector():
    return types.SimpleNamespace(x=0., y=0., z=0.)


def quaternion():
    return types.SimpleNamespace(x=0., y=0., z=0., w=0.)


def header():
    return types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id="")


class LaserScan(object):
    def __init__(self):
        self.header = header()


class Odometry(object):
    def __init__(self):
        self.header = header()
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=vector(), orientation=quaternion()),
            covariance=[])
        self.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=vector(), angular=vector()),
            covariance=[])


class Imu(object):
    def __init__(self):
        self.header = header()
        self.orientation = quaternion()
        self.angular_velocity = vector()
        self.linear_acceleration = vector()


class TransformStamped(object):
    def __init__(self):
        self.header = header()
        self.transform = types.SimpleNamespace(
            translation=vector(), rotation=quaternion())


class TFMessage(object):
    def __init__(self):
        self.transforms = []


class FakeNode(object):
    def __init__(self, name):
        self.name = name


def load_player():
    rclpy = types.SimpleNamespace()
    node = types.ModuleType("rclpy.node"); node.Node = FakeNode
    qos = types.ModuleType("rclpy.qos"); qos.qos_profile_sensor_data = object()
    fake_modules = {"rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos}
    for module_name, values in (
            ("geometry_msgs.msg", {"TransformStamped": TransformStamped}),
            ("nav_msgs.msg", {"Odometry": Odometry}),
            ("sensor_msgs.msg", {"Imu": Imu, "LaserScan": LaserScan}),
            ("tf2_msgs.msg", {"TFMessage": TFMessage})):
        module = types.ModuleType(module_name)
        for name, value in values.items():
            setattr(module, name, value)
        parent_name = module_name.split(".", 1)[0]
        parent = types.ModuleType(parent_name); parent.msg = module
        fake_modules[parent_name] = parent; fake_modules[module_name] = module
    with mock.patch.dict(sys.modules, fake_modules):
        spec = importlib.util.spec_from_file_location(
            "replay_player", str(ROOT / "local_nodes" / "replay_player.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


def create_database(path):
    connection = sqlite3.connect(str(path))
    connection.execute(
        "CREATE TABLE messages (sequence INTEGER PRIMARY KEY AUTOINCREMENT, "
        "receipt_unix REAL NOT NULL, receipt_mono REAL NOT NULL, "
        "topic TEXT NOT NULL, payload TEXT NOT NULL)")
    payloads = {
        "/scan_slam": {},
        "/odom_rf2o": {},
        "/tf": {"transforms": [{
            "header": {"frame_id": "odom"},
            "child_frame_id": "base_link",
        }]},
    }
    for index, topic in enumerate(("/scan_slam", "/odom_rf2o", "/tf")):
        connection.execute(
            "INSERT INTO messages(receipt_unix, receipt_mono, topic, payload) "
            "VALUES (?, ?, ?, ?)",
            (100. + index, 10. + index, topic, json.dumps(payloads[topic])))
    connection.commit(); connection.close()


class ReplayPlayerTest(unittest.TestCase):
    def setUp(self):
        self.player = load_player()

    def test_database_validation_requires_integrity_sequence_and_topics(self):
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "capture.sqlite3"
            create_database(database)
            metadata = self.player.validate_database(database)
            self.assertEqual(metadata["rows"], 3)
            self.assertEqual(metadata["duration_s"], 2.)
            self.assertEqual(metadata["odom_to_base_transforms"], 1)
            connection = sqlite3.connect(str(database))
            connection.execute("DELETE FROM messages WHERE topic='/tf'")
            connection.commit(); connection.close()
            with self.assertRaises(self.player.ReplayFormatError):
                self.player.validate_database(database)

    def test_stamp_mapper_shifts_and_scales_relative_time(self):
        mapper = self.player.StampMapper(100., 200., 2.)
        sec, nanosec = mapper.map({"sec": 102, "nanosec": 500000000})
        self.assertEqual((sec, nanosec), (201, 250000000))

    def test_laser_restore_preserves_missing_return_as_positive_infinity(self):
        payload = {
            "header": {"sec": 100, "nanosec": 0, "frame_id": "laser_frame"},
            "angle_min": 0., "angle_max": 1., "angle_increment": .5,
            "time_increment": .01, "scan_time": .1,
            "range_min": .15, "range_max": 12.,
            "ranges": [1., None], "intensities": [2., 3.],
        }
        message = self.player.decode_laser_scan(
            payload, self.player.StampMapper(100., 200., 1.))
        self.assertEqual(message.header.stamp.sec, 200)
        self.assertEqual(message.header.frame_id, "laser_frame")
        self.assertTrue(math.isinf(message.ranges[1]))
        self.assertGreater(message.ranges[1], 0)

    def test_candidate_slam_never_receives_original_map_to_odom(self):
        def transform(parent, child):
            return {
                "header": {"sec": 100, "nanosec": 0, "frame_id": parent},
                "child_frame_id": child,
                "translation": {"x": 1., "y": 2., "z": 3.},
                "rotation": {"x": 0., "y": 0., "z": 0., "w": 1.},
            }
        message = self.player.decode_transforms(
            {"transforms": [
                transform("map", "odom"), transform("odom", "base_link")]},
            self.player.StampMapper(100., 200., 1.),
            allowed_edges={("odom", "base_link")})
        self.assertEqual(len(message.transforms), 1)
        self.assertEqual(message.transforms[0].header.frame_id, "odom")
        self.assertEqual(message.transforms[0].child_frame_id, "base_link")


if __name__ == "__main__":
    unittest.main()
