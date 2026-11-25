import importlib.util
import math
from pathlib import Path
import unittest


MODULE = Path(__file__).resolve().parents[1] / "local_nodes" / "lidar_deskew.py"


def load_module():
    spec = importlib.util.spec_from_file_location("lidar_deskew", str(MODULE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scan(count=4, end=1.0, duration=1.0):
    seconds = int(end)
    nanoseconds = int(round((end - seconds) * 1000000000))
    return {
        "header": {
            "sec": seconds,
            "nanosec": nanoseconds,
            "frame_id": "laser_frame",
        },
        "angle_min": 0.0,
        "angle_max": 2.0 * math.pi,
        "angle_increment": 2.0 * math.pi / count,
        "time_increment": duration / count,
        "scan_time": duration,
        "range_min": 0.15,
        "range_max": 12.0,
        "ranges": [None] * count,
        "intensities": [None] * count,
    }


class LidarDeskewTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()

    def timeline(self, start, end):
        return self.module.PoseTimeline([
            (0.0,) + tuple(start),
            (1.0,) + tuple(end),
        ])

    def test_stationary_scan_preserves_geometry_and_sets_common_time(self):
        value = scan()
        value["ranges"] = [1.0, 2.0, 3.0, 4.0]
        value["intensities"] = [10.0, 20.0, 30.0, 40.0]
        deskewer = self.module.ScanDeskewer(
            self.timeline((2.0, -1.0, 0.4), (2.0, -1.0, 0.4)),
            laser_x=0.1, laser_y=-0.2, laser_yaw=math.pi)
        result = deskewer.deskew(value)
        for actual, expected in zip(result["ranges"], value["ranges"]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(result["intensities"], value["intensities"])
        self.assertEqual(result["header"], value["header"])
        self.assertEqual(result["time_increment"], 0.0)
        self.assertEqual(deskewer.summary()["scans_corrected"], 1)
        self.assertAlmostEqual(
            deskewer.summary()["maximum_point_correction"], 0.0, places=12)

    def test_translation_moves_early_endpoint_to_scan_end_frame(self):
        value = scan()
        value["ranges"][0] = 2.0
        value["intensities"][0] = 7.0
        deskewer = self.module.ScanDeskewer(
            self.timeline((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            laser_yaw=0.0)
        result = deskewer.deskew(value)
        self.assertAlmostEqual(result["ranges"][0], 1.0)
        self.assertEqual(result["intensities"][0], 7.0)
        self.assertEqual(sum(x is not None for x in result["ranges"]), 1)

    def test_rotation_moves_early_endpoint_to_correct_reference_bin(self):
        value = scan(count=8)
        value["ranges"][0] = 2.0
        deskewer = self.module.ScanDeskewer(
            self.timeline((0.0, 0.0, 0.0),
                          (0.0, 0.0, math.pi / 2.0)),
            laser_yaw=0.0)
        result = deskewer.deskew(value)
        self.assertIsNone(result["ranges"][0])
        self.assertAlmostEqual(result["ranges"][6], 2.0)
        self.assertEqual(sum(x is not None for x in result["ranges"]), 1)

    def test_clockwise_acquisition_order_is_inverse_of_ros_bins(self):
        self.assertEqual(
            [self.module.acquisition_rank(index, 360)
             for index in (0, 359, 358, 1)],
            [0, 1, 2, 359])

    def test_yaw_interpolation_unwraps_across_pi(self):
        timeline = self.timeline(
            (0.0, 0.0, math.radians(170.0)),
            (0.0, 0.0, math.radians(-170.0)))
        unused_x, unused_y, yaw = timeline.interpolate(0.5)
        self.assertAlmostEqual(math.degrees(yaw), 180.0)

    def test_nearer_return_wins_rebin_collision(self):
        value = scan()
        # A rapid reference-frame rotation puts multiple coarse beams into the
        # same rounded output bin.  The output must retain the nearer obstacle.
        value["ranges"][0] = 3.0
        value["ranges"][3] = 1.0
        deskewer = self.module.ScanDeskewer(
            self.timeline((0.0, 0.0, 0.0),
                          (0.0, 0.0, math.pi)),
            laser_yaw=0.0)
        result = deskewer.deskew(value)
        finite = [item for item in result["ranges"] if item is not None]
        self.assertTrue(finite)
        self.assertEqual(deskewer.summary()["rebin_collisions"], 1)
        self.assertAlmostEqual(min(finite), 1.0)

    def test_scan_without_full_pose_coverage_is_unchanged_and_counted(self):
        value = scan(end=2.0)
        value["ranges"][0] = 1.0
        deskewer = self.module.ScanDeskewer(
            self.timeline((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        result = deskewer.deskew(value)
        self.assertIs(result, value)
        self.assertEqual(deskewer.summary()["scans_without_coverage"], 1)
        self.assertEqual(deskewer.summary()["scans_corrected"], 0)


if __name__ == "__main__":
    unittest.main()
