import importlib.util
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "stationary_scan_aggregation",
        str(ROOT / "local_nodes" / "stationary_scan_aggregation.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scan(stamp, ranges, scan_time=0.1):
    return {
        "header": {
            "sec": int(stamp),
            "nanosec": int(round((stamp - int(stamp)) * 1e9)),
            "frame_id": "laser_frame",
        },
        "angle_min": -math.pi,
        "angle_max": math.pi,
        "angle_increment": math.pi / 2,
        "time_increment": scan_time / len(ranges),
        "scan_time": scan_time,
        "range_min": 0.15,
        "range_max": 12.0,
        "ranges": ranges,
        "intensities": [10.0 + index for index in range(len(ranges))],
    }


class Timeline(object):
    def __init__(self, poses=None, covered=True):
        self.poses = poses or {}
        self.covered = covered

    def covers(self, unused_start, unused_end):
        return self.covered

    def interpolate(self, stamp):
        if self.poses:
            return self.poses[round(stamp, 3)]
        return 0.0, 0.0, 0.0


class StationaryScanAggregationTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()

    def test_stationary_episode_emits_one_median_at_last_sequence(self):
        selector = self.module.StationaryMedianScanSelector(
            Timeline(), minimum_episode_scans=3,
            minimum_observation_fraction=2.0 / 3.0)
        selector.prepare([
            (10, scan(1.0, [1.0, None, 3.0, 9.0])),
            (11, scan(1.1, [2.0, 5.0, None, 8.0])),
            (12, scan(1.2, [3.0, 7.0, 4.0, None])),
        ])
        self.assertIsNone(selector.payload_for(10))
        result = selector.payload_for(12)
        self.assertEqual(result["header"]["nanosec"], 200000000)
        self.assertEqual(result["ranges"], [2.0, 6.0, 3.5, 8.5])
        self.assertEqual(selector.summary()["published_episodes"], 1)
        self.assertEqual(
            selector.summary()["published_episode_scan_counts"], [3])
        self.assertEqual(selector.summary()["published_sequences"], [12])
        self.assertEqual(selector.summary()["published_end_stamps"], [1.2])

    def test_missing_bin_requires_observation_fraction(self):
        selector = self.module.StationaryMedianScanSelector(
            Timeline(), minimum_episode_scans=3,
            minimum_observation_fraction=1.0)
        selector.prepare([
            (1, scan(1.0, [1.0, None])),
            (2, scan(1.1, [2.0, 5.0])),
            (3, scan(1.2, [3.0, 6.0])),
        ])
        self.assertEqual(selector.payload_for(3)["ranges"], [2.0, None])
        self.assertEqual(selector.payload_for(3)["intensities"][1], 0.0)

    def test_motion_splits_episodes_and_short_candidates_are_rejected(self):
        poses = {}
        for end in (1.0, 1.1, 2.0, 2.1, 2.2):
            poses[round(end - 0.1, 3)] = (0.0, 0.0, 0.0)
            poses[round(end, 3)] = (
                0.02 if end == 1.1 else 0.0, 0.0, 0.0)
        selector = self.module.StationaryMedianScanSelector(
            Timeline(poses), minimum_episode_scans=3)
        selector.prepare([
            (1, scan(1.0, [1.0, 2.0])),
            (2, scan(1.1, [1.0, 2.0])),
            (3, scan(2.0, [2.0, 3.0])),
            (4, scan(2.1, [2.0, 3.0])),
            (5, scan(2.2, [2.0, 3.0])),
        ])
        summary = selector.summary()
        self.assertEqual(summary["episode_candidates"], 2)
        self.assertEqual(summary["short_episodes_rejected"], 1)
        self.assertEqual(summary["published_episodes"], 1)
        self.assertIsNotNone(selector.payload_for(5))

    def test_uncovered_scans_are_not_silently_selected(self):
        selector = self.module.StationaryMedianScanSelector(
            Timeline(covered=False), minimum_episode_scans=2)
        with self.assertRaises(self.module.StationaryAggregationError):
            selector.prepare([
                (1, scan(1.0, [1.0, 2.0])),
                (2, scan(1.1, [1.0, 2.0])),
            ])
        self.assertEqual(selector.summary()["scans_without_coverage"], 2)

    def test_geometry_changes_are_rejected(self):
        first = scan(1.0, [1.0, 2.0])
        second = scan(1.1, [1.0, 2.0])
        second["angle_increment"] = 0.5
        selector = self.module.StationaryMedianScanSelector(
            Timeline(), minimum_episode_scans=2)
        with self.assertRaises(self.module.StationaryAggregationError):
            selector.prepare([(1, first), (2, second)])


if __name__ == "__main__":
    unittest.main()
