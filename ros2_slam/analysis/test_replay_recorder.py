import importlib.util
import json
from pathlib import Path
import queue
import sqlite3
import sys
import tempfile
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


class FakeNode(object):
    def __init__(self, name):
        self.name = name


def load_recorder():
    rclpy = types.SimpleNamespace()
    node = types.ModuleType("rclpy.node"); node.Node = FakeNode
    qos = types.ModuleType("rclpy.qos"); qos.qos_profile_sensor_data = object()
    messages = {}
    fake_modules = {"rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos}
    for module_name, class_names in (
            ("geometry_msgs.msg", ("TransformStamped",)),
            ("nav_msgs.msg", ("Odometry",)),
            ("sensor_msgs.msg", ("Imu", "LaserScan")),
            ("tf2_msgs.msg", ("TFMessage",))):
        module = types.ModuleType(module_name)
        for class_name in class_names:
            value = type(class_name, (object,), {})
            setattr(module, class_name, value)
            messages[class_name] = value
        parent_name = module_name.split(".", 1)[0]
        parent = types.ModuleType(parent_name)
        parent.msg = module
        fake_modules[parent_name] = parent
        fake_modules[module_name] = module
    with mock.patch.dict(sys.modules, fake_modules):
        spec = importlib.util.spec_from_file_location(
            "replay_recorder", str(ROOT / "local_nodes" / "replay_recorder.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module, messages


class ReplayRecorderTest(unittest.TestCase):
    def test_writer_persists_ordered_messages_and_counts(self):
        recorder, unused = load_recorder()
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "capture.sqlite3"
            messages = queue.Queue()
            writer = recorder.DatabaseWriter(database, messages)
            writer.start()
            messages.put((1.0, 2.0, "/scan", {"value": 1}))
            messages.put((1.1, 2.1, "/odom_rf2o", {"value": 2}))
            messages.put(None)
            writer.join(5)
            self.assertFalse(writer.is_alive())
            self.assertEqual(writer.errors, [])
            self.assertEqual(writer.counts, {"/scan": 1, "/odom_rf2o": 1})
            connection = sqlite3.connect(str(database))
            rows = connection.execute(
                "SELECT topic, payload FROM messages ORDER BY sequence").fetchall()
            connection.close()
            self.assertEqual([row[0] for row in rows], ["/scan", "/odom_rf2o"])
            self.assertEqual(json.loads(rows[0][1]), {"value": 1})

    def test_laser_encoder_preserves_scan_geometry_and_arrays(self):
        recorder, messages = load_recorder()
        scan = messages["LaserScan"]()
        scan.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1, nanosec=2), frame_id="laser")
        scan.angle_min = -1.; scan.angle_max = 1.; scan.angle_increment = .5
        scan.time_increment = .01; scan.scan_time = .1
        scan.range_min = .15; scan.range_max = 12.
        scan.ranges = [1., float('inf')]; scan.intensities = [3., 4.]
        encoded = recorder.laser_scan(scan)
        self.assertEqual(encoded["header"], {"sec": 1, "nanosec": 2, "frame_id": "laser"})
        self.assertEqual(encoded["ranges"], [1., None])
        self.assertEqual(encoded["intensities"], [3., 4.])


if __name__ == "__main__":
    unittest.main()
