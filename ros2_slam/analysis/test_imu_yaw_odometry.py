import ast
import importlib.util
import math
import contextlib
import io
import types
from pathlib import Path
import tempfile
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

    def test_synchronizer_waits_for_reference_then_releases_in_order(self):
        queue = core.TimestampSynchronizer(max_pending=3)
        queue.add(10.04, 100.0, "first")
        queue.add(10.06, 100.1, "second")
        ready, too_old = queue.drain(10.00, 10.03)
        self.assertEqual(ready, [])
        self.assertEqual(too_old, [])
        self.assertAlmostEqual(queue.oldest_wait_s(100.5), 0.5)
        ready, too_old = queue.drain(10.00, 10.05)
        self.assertEqual([item[2] for item in ready], ["first"])
        self.assertEqual(too_old, [])
        ready, too_old = queue.drain(10.00, 10.07)
        self.assertEqual([item[2] for item in ready], ["second"])
        self.assertEqual(too_old, [])

    def test_synchronizer_reports_too_old_and_rejects_bad_order(self):
        queue = core.TimestampSynchronizer(max_pending=2)
        queue.add(9.0, 100.0, "old")
        queue.add(10.0, 100.1, "ready")
        ready, too_old = queue.drain(9.5, 10.0)
        self.assertEqual([item[2] for item in too_old], ["old"])
        self.assertEqual([item[2] for item in ready], ["ready"])
        with self.assertRaises(core.StreamTimingError):
            queue.add(10.0, 100.2, "duplicate")

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
            stale_timeout_s=1.0, stable_duration_s=0.5
        )
        watchdog.observe(1.0)
        watchdog.check(5.0)  # A first-sample pause is allowed during startup.
        self.assertAlmostEqual(watchdog.observe(5.1), 4.1)
        self.assertEqual(watchdog.consecutive_samples, 1)
        watchdog.observe(5.4)
        watchdog.observe(5.7)
        self.assertTrue(watchdog.ready)
        watchdog.check(6.6)
        with self.assertRaises(core.StreamTimingError):
            watchdog.check(6.8)

    def test_watchdog_does_not_arm_on_sample_count_without_duration(self):
        watchdog = core.StartupAwareWatchdog(
            0.0, stable_samples=3, startup_timeout_s=20.0,
            stale_timeout_s=1.0, stable_duration_s=3.0
        )
        watchdog.observe(1.0)
        watchdog.observe(1.1)
        watchdog.observe(1.2)
        self.assertFalse(watchdog.ready)
        watchdog.check(5.0)

    def test_watchdog_tolerates_measured_scan_gap_below_runtime_limit(self):
        watchdog = core.StartupAwareWatchdog(
            0.0, stable_samples=2, startup_timeout_s=20.0,
            stale_timeout_s=2.0, stable_duration_s=0.1
        )
        watchdog.observe(1.0)
        watchdog.observe(1.2)
        self.assertTrue(watchdog.ready)
        watchdog.check(3.0)
        self.assertAlmostEqual(watchdog.observe(2.542), 1.342)
        watchdog.check(4.4)
        with self.assertRaises(core.StreamTimingError):
            watchdog.check(4.6)

    def test_watchdog_fails_if_stream_never_stabilizes(self):
        watchdog = core.StartupAwareWatchdog(
            0.0, stable_samples=3, startup_timeout_s=20.0,
            stale_timeout_s=1.0, stable_duration_s=3.0
        )
        watchdog.observe(2.0)
        with self.assertRaisesRegex(core.StreamTimingError, "total=1"):
            watchdog.check(20.1)

    def test_session_has_explicit_baseline_and_imu_modes(self):
        text = SESSION.read_text(encoding="utf-8")
        self.assertIn('["rf2o", "imu_yaw", "imu_yaw_hold"]', text)
        self.assertIn("odom_topic:=/odom_rf2o_raw", text)
        self.assertIn("publish_tf:=false", text)
        self.assertIn("imu_yaw_odometry.py", text)
        self.assertIn("python3 -u {}/imu_yaw_odometry.py", text)
        self.assertIn("exec env PYTHONUNBUFFERED=1 {}", text)
        self.assertIn("Required ROS process exited during startup", text)

    def test_imu_transport_uses_deep_best_effort_qos_at_both_ends(self):
        node_text = NODE.read_text(encoding="utf-8")
        publisher_text = (ROOT / "local_nodes" /
                          "phidget_imu_publisher.py").read_text(encoding="utf-8")
        for text in (node_text, publisher_text):
            self.assertIn("IMU_QOS_DEPTH = 100", text)
            self.assertIn("QoSReliabilityPolicy.BEST_EFFORT", text)
            self.assertIn("QoSDurabilityPolicy.VOLATILE", text)
        self.assertIn("IMU_SOURCE_GAP", publisher_text)
        self.assertIn("max_device_gap_ms", publisher_text)

    def test_session_verifies_live_odom_messages_not_buffered_log_text(self):
        tree = ast.parse(SESSION.read_text(encoding="utf-8"))
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name == "MappingSession")
        method = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                      and node.name == "verify_odom_source")
        module = ast.Module(body=[method], type_ignores=[])
        with tempfile.NamedTemporaryFile("w", delete=False) as stream:
            stream.write("IMU_YAW_ODOM_READY raw=/odom_rf2o_raw\n")
            log_path = stream.name
        try:
            session = types.SimpleNamespace(
                odom_mode="imu_yaw",
                odom_message_count=0,
                last_odom={"stamp_s": 9.9},
                preflight_results={},
                process_log_paths={"imu_yaw_odom": log_path},
                require_processes_alive=lambda: None,
                get_clock=lambda: types.SimpleNamespace(
                    now=lambda: types.SimpleNamespace(nanoseconds=10000000000)
                ),
            )
            def spin_once(owner, timeout_sec):
                owner.odom_message_count += 1
            namespace = {
                "time": types.SimpleNamespace(monotonic=lambda: 0.0),
                "rclpy": types.SimpleNamespace(spin_once=spin_once),
            }
            exec(compile(ast.fix_missing_locations(module), "odom_check", "exec"), namespace)
            with contextlib.redirect_stdout(io.StringIO()):
                passed = namespace["verify_odom_source"](session)
        finally:
            Path(log_path).unlink()
        self.assertTrue(passed)
        result = session.preflight_results["odom_source"]
        self.assertEqual(result["new_messages_observed"], 5)
        self.assertTrue(result["qualification_marker_seen"])

    def test_python36_syntax(self):
        for path in (CORE, NODE, SESSION,
                     ROOT / "local_nodes" / "phidget_imu_publisher.py"):
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 6))


if __name__ == "__main__":
    unittest.main()