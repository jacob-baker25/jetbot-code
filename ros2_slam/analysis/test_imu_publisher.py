import ast
import importlib.util
import math
from pathlib import Path
import unittest
import sys
import types
from unittest import mock


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

    def test_backlog_preserves_20ms_sequence_and_reports_actual_old_age(self):
        mapper = imu_planar.DeviceTimeMapper()
        base = 1700000000000000000
        stamps = []
        for i in range(400):
            acquisition_s = i * .02
            # 1.35s delivery stall after sample251, then a fast catch-up burst.
            receipt_s = (acquisition_s if i <= 250 else
                         max(acquisition_s, 5.02 + 1.35 + (i - 251) * .002))
            stamps.append(mapper.stamp(i * 20., base + round(receipt_s * 1e9), receipt_s))
        self.assertTrue(all(b - a == 20000000 for a, b in zip(stamps, stamps[1:])))
        self.assertEqual(stamps[251], base + 5020000000)
        self.assertEqual(stamps[-1], base + 7980000000)

    def test_delayed_first_dequeue_uses_callback_receipt_not_processing_time(self):
        mapper = imu_planar.DeviceTimeMapper()
        # Main loop may process these much later; its current clock is not input.
        self.assertEqual(mapper.stamp(100., 10000000000, 20.), 10000000000)
        self.assertEqual(mapper.stamp(120., 10020000000, 20.02), 10020000000)

    def test_clock_steps_and_device_resets_are_rejected_not_retimed(self):
        for device, ros, mono in [(120., 12020000000, 20.02),
                                  (120., 8020000000, 20.02),
                                  (100., 10020000000, 20.02),
                                  (80., 10020000000, 20.02)]:
            mapper = imu_planar.DeviceTimeMapper()
            mapper.stamp(100., 10000000000, 20.)
            with self.assertRaises(ValueError): mapper.stamp(device, ros, mono)

    def test_clock_read_delay_is_not_mistaken_for_host_clock_step(self):
        mapper = imu_planar.DeviceTimeMapper()
        mapper.stamp(100., 10000000000, 20.)
        self.assertEqual(mapper.stamp(120., 11020000000, 20.52, .5), 10020000000)

    def test_publication_loop_is_quiet_and_uses_device_time_after_queued_delivery(self):
        spec = importlib.util.spec_from_file_location("publisher", str(PUBLISHER))
        pub = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {"imu_planar": imu_planar}):
            spec.loader.exec_module(pub)
        clock, published, active_prints, all_prints = [10.], [], [], []
        active = [False]
        class Spatial:
            def setDeviceSerialNumber(self, serial): pass
            def setOnErrorHandler(self, fn): pass
            def openWaitForAttachment(self, timeout): pass
            def setDataInterval(self, interval): pass
            def getDeviceSerialNumber(self): return 807962
            def getDataInterval(self): return 20
            def zeroGyro(self): pass
            def close(self): active[0] = False
            def setOnSpatialDataHandler(self, fn):
                active[0] = True
                for i in range(300):
                    clock[0] += .02
                    fn(self, [0, 0, 1], [0, 0, 0], [0, 0, 0], i * 20.)
        class Imu:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))
                self.orientation_covariance = [0.] * 9
                self.linear_acceleration_covariance = [0.] * 9
                self.angular_velocity = types.SimpleNamespace(z=0.)
        class Node:
            def __init__(self, name): pass
            def create_publisher(self, *args): return types.SimpleNamespace(publish=lambda msg: published.append(msg))
            def get_clock(self): return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=round(clock[0] * 1e9)))
            def destroy_node(self): pass
        def emit(*args, **kwargs):
            all_prints.append(args)
            if active[0]: active_prints.append(args)
        pub.print = emit
        pub.time = types.SimpleNamespace(monotonic=lambda: clock[0], sleep=lambda s: clock.__setitem__(0, clock[0] + s))
        modules = {
            "Phidget22.Devices.Spatial": types.SimpleNamespace(Spatial=Spatial),
            "rclpy": types.SimpleNamespace(init=lambda: None, ok=lambda: len(published) < 300,
                shutdown=lambda: None, spin_once=lambda *a, **kw: clock.__setitem__(0, clock[0] + .002)),
            "rclpy.node": types.SimpleNamespace(Node=Node),
            "rclpy.qos": types.SimpleNamespace(QoSProfile=lambda depth: types.SimpleNamespace(),
                QoSReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),
                QoSDurabilityPolicy=types.SimpleNamespace(VOLATILE=1)),
            "sensor_msgs.msg": types.SimpleNamespace(Imu=Imu),
        }
        with mock.patch.dict(sys.modules, modules), mock.patch.object(sys, "argv", ["publisher"]):
            pub.main()
        self.assertEqual(active_prints, [])
        self.assertEqual(len(published), 300)
        stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in published]
        self.assertTrue(all(b - a == 20000000 for a, b in zip(stamps, stamps[1:])))
        import json
        summary = json.loads(next(a[0].split(" ", 1)[1] for a in all_prints
                                  if a[0].startswith("IMU_TIMING_SUMMARY ")))
        self.assertGreater(summary["max_seconds"]["queue_wait_s"], 5.0)
        self.assertGreater(summary["event_count"], 128)
        self.assertEqual(summary["retained_event_count"], 128)

    def test_python36_syntax(self):
        for path in (HELPER, PUBLISHER):
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 6))


if __name__ == "__main__":
    unittest.main()
