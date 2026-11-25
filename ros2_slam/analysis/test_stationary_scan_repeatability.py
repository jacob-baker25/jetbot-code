import importlib.util
import math
from pathlib import Path
import unittest


MODULE = (Path(__file__).resolve().parent
          / "analyze_stationary_scan_repeatability.py")


def load_module():
    spec = importlib.util.spec_from_file_location(
        "analyze_stationary_scan_repeatability", str(MODULE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample(stamp, x=0.0, ranges=None, intensities=None):
    ranges = ranges or [1.0, 2.0, None, 4.0]
    intensities = intensities or [10.0, 20.0, None, 40.0]
    return {
        "stamp": stamp,
        "pose": (x, 0.0, 0.0),
        "scan": {
            "angle_min": 0.0,
            "angle_increment": math.pi / 2.0,
            "range_min": 0.1,
            "range_max": 12.0,
            "ranges": ranges,
            "intensities": intensities,
        },
    }


class StationaryScanRepeatabilityTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()

    def test_grouping_splits_motion_and_drops_short_episode(self):
        values = [sample(index * .1) for index in range(8)]
        values += [sample(1.0 + index * .1, x=.10) for index in range(7)]
        episodes = self.module.group_stationary_samples(
            values, minimum_scans=8)
        self.assertEqual(len(episodes), 1)
        self.assertEqual(len(episodes[0]), 8)

    def test_summary_reports_stable_medians_and_quality(self):
        values = []
        for index in range(10):
            delta = .01 if index % 2 else -.01
            values.append(sample(
                index * .1,
                ranges=[1.0 + delta, 2.0, None, 4.0],
                intensities=[10.0, 20.0, None, 40.0]))
        public, private = self.module.summarize_episode(values)
        self.assertEqual(public["scan_count"], 10)
        self.assertEqual(public["well_observed_bins"], 3)
        self.assertEqual(public["stable_bins"], 3)
        self.assertAlmostEqual(public["stable_bin_fraction"], 1.0)
        self.assertAlmostEqual(private["median_ranges"][0], 1.0)
        self.assertEqual(public["quality"]["median"], 20.0)

    def test_endpoint_overlap_is_directional(self):
        first = {(0, 0), (10, 10)}
        second = {(1, 0), (20, 20), (21, 20)}
        count, fraction = self.module.neighborhood_overlap(first, second, 1)
        self.assertEqual(count, 1)
        self.assertEqual(fraction, 0.5)
        count, fraction = self.module.neighborhood_overlap(second, first, 1)
        self.assertEqual(count, 1)
        self.assertAlmostEqual(fraction, 1.0 / 3.0)

    def test_closest_revisit_uses_pose_not_best_overlap(self):
        public = [
            {"start_stamp": 0.0, "end_stamp": 1.0},
            {"start_stamp": 20.0, "end_stamp": 21.0},
            {"start_stamp": 40.0, "end_stamp": 41.0},
        ]
        private = []
        for pose in ((0.0, 0.0, 0.0), (.15, 0.0, 0.0), (.02, 0.0, 0.0)):
            private.append({
                "pose_tuple": pose,
                "median_ranges": [1.0, 1.0, 1.0, 1.0],
                "angle_min": 0.0,
                "angle_increment": math.pi / 2.0,
                "range_min": 0.1,
                "range_max": 12.0,
            })
        result = self.module.closest_revisit(
            public, private, 10.0, .20, math.radians(15.0),
            .05, 0.0, 0.0, 0.0)
        self.assertEqual(result["first_episode"], 0)
        self.assertEqual(result["second_episode"], 2)
        self.assertGreaterEqual(
            result["within_one_cell"]["first_fraction"], .75)

    def test_alignment_recovers_bounded_pose_offset(self):
        episode = {
            "pose_tuple": (0.0, 0.0, 0.0),
            "median_ranges": [1.0, 2.0, 1.5, .75],
            "angle_min": 0.0,
            "angle_increment": math.pi / 2.0,
            "range_min": 0.1,
            "range_max": 12.0,
        }
        shifted = dict(episode)
        shifted["pose_tuple"] = (.10, -.05, math.radians(3.0))
        result = self.module.optimize_revisit_alignment(
            episode, shifted, resolution=.025, laser_yaw=0.0,
            translation_window_m=.15, translation_step_m=.025,
            rotation_window_rad=math.radians(5.0),
            rotation_step_rad=math.radians(1.0))
        correction = result["best_correction"]
        # One-cell endpoint tolerance makes neighboring grid corrections
        # equivalent; the tie breaker intentionally prefers the smaller move.
        self.assertLessEqual(abs(correction["x_m"] + .10), .025001)
        self.assertLessEqual(abs(correction["y_m"] - .05), .025001)
        self.assertLessEqual(abs(correction["yaw_deg"] + 3.0), 1.000001)
        self.assertEqual(
            result["within_one_cell"]["symmetric_minimum_fraction"], 1.0)


if __name__ == "__main__":
    unittest.main()
