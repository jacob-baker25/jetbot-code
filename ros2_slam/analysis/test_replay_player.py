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
    deskew = types.ModuleType("lidar_deskew")
    deskew.PoseTimeline = object
    deskew.ScanDeskewer = object
    deskew.pose_samples_from_connection = lambda connection: []
    fake_modules["lidar_deskew"] = deskew
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

    def test_lateness_tracker_groups_stalls_and_reports_topics(self):
        tracker = self.player.LatenessTracker(.25)
        tracker.observe(1, "/tf", 0., .01)
        tracker.observe(2, "/scan_slam", 1., .30)
        tracker.observe(3, "/tf", 1.01, .40)
        tracker.observe(4, "/odom_rf2o", 1.02, .10)
        tracker.observe(5, "/tf", 2., .26)
        result = tracker.summary()
        self.assertEqual(result["scheduled_messages"], 5)
        self.assertEqual(result["late_messages"], 3)
        self.assertEqual(result["late_burst_count"], 2)
        self.assertEqual(result["late_counts_by_topic"], {
            "/scan_slam": 1, "/tf": 2})
        self.assertEqual(result["late_bursts"][0]["message_count"], 2)
        self.assertEqual(result["late_bursts"][0]["maximum_sequence"], 3)
        self.assertAlmostEqual(result["maximum_lateness_s"], .40)

    def test_scan_rotation_filter_interpolates_yaw_and_handles_wrap(self):
        samples = [
            (0., math.radians(170.)),
            (.5, math.radians(170.)),
            (.6, math.radians(172.)),
            (1., math.radians(-170.)),
        ]
        rotation_filter = self.player.ScanRotationFilter(
            samples, math.radians(1.5))
        scan = {
            "header": {"sec": 0, "nanosec": 600000000},
            "scan_time": .1,
        }
        self.assertAlmostEqual(
            math.degrees(rotation_filter.rotation(scan)), 2.0, places=6)
        self.assertFalse(rotation_filter.keep(scan))
        summary = rotation_filter.summary()
        self.assertEqual(summary["observed_scans"], 1)
        self.assertEqual(summary["filtered_scans"], 1)
        self.assertAlmostEqual(summary["maximum_observed_rotation_deg"], 2.0)

    def test_scan_translation_filter_uses_complete_scan_interval(self):
        class Timeline(object):
            def covers(self, start, end):
                return start >= 0.0 and end <= 1.0

            def interpolate(self, stamp):
                return stamp, 2.0 * stamp, 0.0

        translation_filter = self.player.ScanTranslationFilter(
            Timeline(), .20)
        scan = {
            "header": {"sec": 0, "nanosec": 600000000},
            "scan_time": .1,
        }
        expected = math.hypot(.1, .2)
        self.assertAlmostEqual(
            translation_filter.translation(scan), expected, places=12)
        self.assertFalse(translation_filter.keep(scan))
        summary = translation_filter.summary()
        self.assertEqual(summary["observed_scans"], 1)
        self.assertEqual(summary["filtered_scans"], 1)
        self.assertAlmostEqual(
            summary["maximum_observed_translation_m"], expected)

    def test_scan_translation_filter_keeps_uncovered_boundary_scan(self):
        class Timeline(object):
            def covers(self, unused_start, unused_end):
                return False

        translation_filter = self.player.ScanTranslationFilter(
            Timeline(), .003)
        scan = {
            "header": {"sec": 0, "nanosec": 50000000},
            "scan_time": .1,
        }
        self.assertTrue(translation_filter.keep(scan))
        summary = translation_filter.summary()
        self.assertEqual(summary["scans_without_coverage"], 1)
        self.assertEqual(summary["filtered_scans"], 0)

    def test_deskew_and_motion_filters_are_separate_opt_in_modes(self):
        source = (ROOT / "local_nodes" / "replay_player.py").read_text(
            encoding="utf-8")
        self.assertIn('parser.add_argument("--deskew-scans"', source)
        self.assertIn(
            'parser.add_argument("--maximum-scan-translation-meters"', source)
        self.assertIn('"scan_translation_filter":', source)
        self.assertIn('"scan_deskew":', source)
        self.assertIn("deskewer.deskew(payload)", source)


if __name__ == "__main__":
    unittest.main()
