import ast
import importlib.util
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "local_nodes" / "lidar_scan_geometry.py"
spec = importlib.util.spec_from_file_location("lidar_scan_geometry", str(HELPER))
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class LidarGeometryTest(unittest.TestCase):

    def test_clockwise_hardware_angles_become_counterclockwise_ros_bins(self):
        expected = {0: 0, 90: 270, 180: 180, 270: 90, 360: 0}
        for hardware_angle, ros_bin in expected.items():
            self.assertEqual(
                geometry.ros_bin_for_hardware_angle(hardware_angle), ros_bin
            )

    def test_integer_binning_is_preserved_before_direction_reversal(self):
        self.assertEqual(geometry.ros_bin_for_hardware_angle(0.9), 0)
        self.assertEqual(geometry.ros_bin_for_hardware_angle(1.9), 359)

    def test_laserscan_metadata_matches_last_sample(self):
        angle_min, angle_max, increment = geometry.laser_scan_angles()
        self.assertEqual(angle_min, 0.0)
        self.assertAlmostEqual(increment, 2.0 * math.pi / 360.0)
        self.assertAlmostEqual(angle_max, angle_min + 359 * increment)

    def test_python36_syntax(self):
        for path in (HELPER, ROOT / "local_nodes" / "rplidar_python_bridge.py"):
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 6))

    def test_deployment_copies_files_and_preserves_bridge(self):
        bash = ("C:/Program Files/Git/bin/bash.exe" if os.name == "nt"
                else shutil.which("bash"))
        if not bash or not Path(bash).is_file():
            self.skipTest("Bash required for deployment test")
        with tempfile.TemporaryDirectory() as folder:
            workspace = Path(folder)
            target = workspace / "local_nodes"
            target.mkdir()
            bridge = target / "rplidar_python_bridge.py"
            bridge.write_text("previous bridge")
            env = dict(os.environ, AUTOCHAIR_WS=workspace.as_posix())
            subprocess.run(
                [bash, (ROOT / "deploy_lidar_geometry_fix.sh").as_posix()],
                env=env,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            for name in (
                "lidar_scan_geometry.py",
                "rplidar_python_bridge.py",
                "mapping_session.py",
            ):
                self.assertEqual(
                    (target / name).read_bytes(),
                    (ROOT / "local_nodes" / name).read_bytes(),
                )
            backups = list((workspace / "backups").glob(
                "lidar_geometry_*/rplidar_python_bridge.py"
            ))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_text(), "previous bridge")


if __name__ == "__main__":
    unittest.main()
