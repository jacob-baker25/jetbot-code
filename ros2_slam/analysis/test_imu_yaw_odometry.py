import ast
import importlib.util
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "local_nodes" / "imu_yaw_odometry_core.py"
NODE = ROOT / "local_nodes" / "imu_yaw_odometry.py"
SESSION = ROOT / "local_nodes" / "mapping_session.py"
spec = importlib.util.spec_from_file_location("imu_yaw_odometry_core", str(CORE))
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)


class ImuYawOdometryTest(unittest.TestCase):

    def test_integrator_interpolates_device_stamped_yaw(self):
        integrator = core.YawIntegrator(max_gap_s=0.1)
        integrator.add(10.00, 0.0)
        integrator.add(10.02, 1.0)
        integrator.add(10.04, 1.0)
        angle, rate = integrator.interpolate(10.03)
        self.assertAlmostEqual(angle, 0.02)
        self.assertAlmostEqual(rate, 1.0)

    def test_integrator_fails_closed_on_gap_and_reverse_time(self):
        integrator = core.YawIntegrator(max_gap_s=0.1)
        integrator.add(1.0, 0.0)
        with self.assertRaises(core.ImuTimingError):
            integrator.add(1.2, 0.0)
        integrator = core.YawIntegrator(max_gap_s=0.1)
        integrator.add(1.0, 0.0)
        with self.assertRaises(core.ImuTimingError):
            integrator.add(1.0, 0.0)

    def test_fusion_uses_imu_yaw_and_rf2o_body_translation(self):
        fusion = core.PlanarYawFusion()
        x, y, yaw = fusion.update(0.0, 0.0, 0.0, 0.0)
        self.assertEqual((x, y, yaw), (0.0, 0.0, 0.0))
        # RF2O under-reports an in-place 90-degree turn as 0.8 rad. IMU wins.
        x, y, yaw = fusion.update(0.0, 0.0, 0.8, math.pi / 2.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)
        # RF2O then reports one metre forward in its own 0.8-rad frame.
        raw_x = math.cos(0.8)
        raw_y = math.sin(0.8)
        x, y, yaw = fusion.update(raw_x, raw_y, 0.8, math.pi / 2.0)
        self.assertAlmostEqual(x, 0.0, places=7)
        self.assertAlmostEqual(y, 1.0, places=7)

    def test_initial_heading_offset_is_preserved(self):
        fusion = core.PlanarYawFusion()
        fusion.update(0.0, 0.0, 0.2, 3.0)
        _, _, yaw = fusion.update(0.0, 0.0, 0.2, 3.5)
        self.assertAlmostEqual(yaw, 0.7)

    def test_watchdog_allows_bounded_startup_then_arms_strict_stale_gate(self):
        watchdog = core.StartupAwareWatchdog(
            0.0, stable_samples=3, startup_timeout_s=20.0,
            stale_timeout_s=1.0
        )
        watchdog.observe(1.0)
        watchdog.check(5.0)  # A first-sample pause is allowed during startup.
        watchdog.observe(5.1)
        watchdog.observe(5.2)
        self.assertTrue(watchdog.ready)
        watchdog.check(6.1)
        with self.assertRaises(core.StreamTimingError):
            watchdog.check(6.3)

    def test_watchdog_fails_if_stream_never_stabilizes(self):
        watchdog = core.StartupAwareWatchdog(
            0.0, stable_samples=3, startup_timeout_s=20.0,
            stale_timeout_s=1.0
        )
        watchdog.observe(2.0)
        with self.assertRaises(core.StreamTimingError):
            watchdog.check(20.1)

    def test_session_has_explicit_baseline_and_imu_modes(self):
        text = SESSION.read_text(encoding="utf-8")
        self.assertIn('["rf2o", "imu_yaw"]', text)
        self.assertIn("odom_topic:=/odom_rf2o_raw", text)
        self.assertIn("publish_tf:=false", text)
        self.assertIn("imu_yaw_odometry.py", text)

    def test_python36_syntax(self):
        for path in (CORE, NODE, SESSION):
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 6))


if __name__ == "__main__":
    unittest.main()
