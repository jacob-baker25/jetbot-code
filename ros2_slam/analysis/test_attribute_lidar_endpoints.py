import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest


MODULE = Path(__file__).resolve().parent / "attribute_lidar_endpoints.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "attribute_lidar_endpoints", str(MODULE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def checkpoint(index, x, y, yaw, stamp):
    return {
        "index": index,
        "label": "checkpoint_{}".format(index),
        "time_unix": stamp,
        "rf2o": {"x": x, "y": y, "yaw_deg": yaw},
    }


class AttributeLidarEndpointsTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()

    def test_route_inference_finds_distant_turnaround_and_turn_end(self):
        value = {
            "experiment_name": "closed_route",
            "checkpoints": [
                checkpoint(0, 0.0, 0.0, 0.0, 10.0),
                checkpoint(1, 1.0, 0.0, 0.0, 20.0),
                checkpoint(2, 2.0, 1.0, 90.0, 30.0),
                checkpoint(3, 1.98, 0.99, -90.0, 40.0),
                checkpoint(4, 0.0, 0.0, 0.0, 50.0),
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.json"
            path.write_text(json.dumps(value), encoding="utf-8")
            route = self.module.load_route(path)
        self.assertEqual(route["turnaround_start_index"], 2)
        self.assertEqual(route["turnaround_end_index"], 3)
        self.assertEqual(self.module.pass_for_stamp(25.0, route), "outbound")
        self.assertEqual(self.module.pass_for_stamp(35.0, route), "turnaround")
        self.assertEqual(self.module.pass_for_stamp(45.0, route), "return")

    def test_motion_classification_changes_one_factor_at_a_time(self):
        classify = self.module.motion_for_delta
        rotation = math.radians(1.0)
        self.assertEqual(classify(0.0, 0.0, .003, rotation), "stationary")
        self.assertEqual(classify(.004, 0.0, .003, rotation), "translating")
        self.assertEqual(classify(0.0, rotation, .003, rotation), "turning")
        self.assertEqual(
            classify(.004, rotation, .003, rotation),
            "turning_translating")

    def test_neighborhood_overlap_is_directional_and_bounded(self):
        first = {(0, 0), (10, 10)}
        second = {(1, 0), (20, 20), (21, 20)}
        count, fraction = self.module.neighborhood_overlap(first, second, 1)
        self.assertEqual(count, 1)
        self.assertEqual(fraction, 0.5)
        reverse_count, reverse_fraction = self.module.neighborhood_overlap(
            second, first, 1)
        self.assertEqual(reverse_count, 1)
        self.assertAlmostEqual(reverse_fraction, 1.0 / 3.0)

    def test_scan_gate_reproduces_distance_and_heading_reasons(self):
        gate = self.module.scan_gate_decision
        accepted, reason = gate(None, (0.0, 0.0, 0.0), 0.5, 0.5)
        self.assertTrue(accepted)
        self.assertEqual(reason, "initial")
        accepted, reason = gate(
            (0.0, 0.0, 0.0), (0.51, 0.0, 0.0), 0.5, 0.5)
        self.assertTrue(accepted)
        self.assertEqual(reason, "distance")
        accepted, reason = gate(
            (0.0, 0.0, 0.0), (0.1, 0.0, 0.51), 0.5, 0.5)
        self.assertTrue(accepted)
        self.assertEqual(reason, "heading")
        accepted, reason = gate(
            (0.0, 0.0, 0.0), (0.1, 0.0, 0.1), 0.5, 0.5)
        self.assertFalse(accepted)
        self.assertEqual(reason, "below_threshold")

    def test_world_projection_honors_laser_yaw_and_range_limit(self):
        scan = {
            "angle_min": 0.0,
            "angle_increment": math.pi / 2.0,
            "range_min": 0.1,
            "ranges": [1.0, 2.0, 20.0, None],
        }
        points = self.module.world_endpoints(
            scan, (1.0, 2.0, 0.0), 0.0, 0.0, math.pi, 12.0)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], 0.0)
        self.assertAlmostEqual(points[0][1], 2.0)
        self.assertAlmostEqual(points[1][0], 1.0)
        self.assertAlmostEqual(points[1][1], 0.0)

    def test_category_color_marks_multi_source_cells_black(self):
        colors = {"a": (1, 2, 3), "b": (4, 5, 6)}
        self.assertEqual(
            self.module.category_color({"a": 2}, ("a", "b"), colors, 2),
            (1, 2, 3))
        self.assertEqual(
            self.module.category_color(
                {"a": 2, "b": 3}, ("a", "b"), colors, 2),
            (0, 0, 0))
        self.assertIsNone(
            self.module.category_color({"a": 1}, ("a", "b"), colors, 2))


if __name__ == "__main__":
    unittest.main()
