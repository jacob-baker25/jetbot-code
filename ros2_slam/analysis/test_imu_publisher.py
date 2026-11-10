import ast
import importlib.util
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "local_nodes" / "imu_planar.py"
PUBLISHER = ROOT / "local_nodes" / "phidget_imu_publisher.py"
spec = importlib.util.spec_from_file_location("imu_planar", str(HELPER))
imu_planar = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imu_planar)


class ImuPublisherTest(unittest.TestCase):

    def test_validated_planar_sign_and_units(self):
        self.assertAlmostEqual(
            imu_planar.robot_yaw_rate_rad_s(-90.0), math.pi / 2.0
        )
        self.assertAlmostEqual(
            imu_planar.robot_yaw_rate_rad_s(90.0), -math.pi / 2.0
        )

    def test_device_timestamp_anchor(self):
        anchor_ros_ns = 1700000000123456789
        self.assertEqual(
            imu_planar.anchored_stamp_ns(1020.0, 1000.0, anchor_ros_ns),
            anchor_ros_ns + 20000000,
        )

    def test_stamp_split(self):
        self.assertEqual(
            imu_planar.split_stamp_ns(1700000000123456789),
            (1700000000, 123456789),
        )
        with self.assertRaises(ValueError):
            imu_planar.split_stamp_ns(-1)

    def test_python36_syntax(self):
        for path in (HELPER, PUBLISHER):
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 6))


if __name__ == "__main__":
    unittest.main()
