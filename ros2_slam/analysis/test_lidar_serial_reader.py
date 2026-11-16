import ast
from collections import deque
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1] / 'local_nodes'


class DriverError(Exception):
    pass


class Vendor:
    def __init__(self, *args, **kwargs):
        self._serial = mock.Mock(timeout=kwargs.get('timeout', 2))
        self.scanning = [False, 5, 'normal']

    def stop(self):
        self.scanning[0] = False

    def start(self, scan_type='normal'):
        self.scanning = [True, 5, scan_type]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


vendor_module = types.ModuleType('rplidar')
vendor_module.RPLidar = Vendor
vendor_module.RPLidarException = DriverError
with mock.patch.dict(sys.modules, rplidar=vendor_module):
    reader = load('reader_under_test', ROOT / 'lidar_serial_reader.py')


class LidarReaderTest(unittest.TestCase):
    def test_packet_bytes_unchanged_without_polling(self):
        lidar = reader.BoundedRPLidar('fake', timeout=2)
        payload = bytes([0, 255, 128, 17, 42])
        lidar._serial.read.return_value = payload
        self.assertEqual(lidar._read_response(5), payload)
        lidar._serial.read.assert_called_once_with(5)
        lidar._serial.inWaiting.assert_not_called()

    def test_partial_packet_and_unbounded_timeout_fail(self):
        lidar = reader.BoundedRPLidar('fake', timeout=2)
        lidar._serial.read.return_value = b'12'
        with self.assertRaisesRegex(DriverError, 'expected 5 bytes, got 2'):
            lidar._read_response(5)
        self.assertEqual(lidar.read_timeouts, 1)
        for timeout in (None, 0, -1, float('inf'), float('nan')):
            lidar._serial.timeout = timeout
            with self.assertRaisesRegex(DriverError, 'positive finite'):
                lidar._read_response(5)
        self.assertEqual(lidar.read_calls, 1)

    def test_serial_exception_is_not_swallowed(self):
        lidar = reader.BoundedRPLidar('fake')
        lidar._serial.read.side_effect = OSError('disconnected')
        with self.assertRaisesRegex(OSError, 'disconnected'):
            lidar._read_response(5)
        self.assertEqual(lidar.read_calls, 1)

    def test_recovery_records_only_active_capture_and_bounds_memory(self):
        lidar = reader.BoundedRPLidar('fake')
        lidar.start()
        lidar.stop()
        self.assertEqual(lidar.recovery_count, 0)
        lidar.start()
        lidar.capture_active = True
        for _ in range(40):
            lidar.stop()
            lidar.start()
        summary = lidar.timing_summary()
        self.assertEqual(summary['recovery_count'], 40)
        self.assertEqual(len(summary['recoveries']), 32)
        self.assertTrue(all(x['success'] and x['duration_s'] >= 0
                            for x in summary['recoveries']))
        lidar.capture_active = False
        lidar.stop()
        self.assertEqual(lidar.recovery_count, 40)

    def test_failed_restart_is_recorded_and_propagated(self):
        lidar = reader.BoundedRPLidar('fake')
        lidar.start()
        lidar.capture_active = True
        lidar.stop()
        with mock.patch.object(Vendor, 'start', side_effect=DriverError('restart')):
            with self.assertRaises(DriverError):
                lidar.start()
        self.assertFalse(lidar.recoveries[-1]['success'])

    def test_python36_syntax(self):
        for name in ('lidar_serial_reader.py', 'rplidar_python_bridge.py'):
            ast.parse((ROOT / name).read_text(), feature_version=(3, 6))


class LidarBridgeFailureTest(unittest.TestCase):
    def setUp(self):
        ros = types.ModuleType('rclpy')
        ros.init = mock.Mock()
        ros.ok = mock.Mock(return_value=True)
        ros.shutdown = mock.Mock()
        node = types.ModuleType('rclpy.node')
        node.Node = object
        sensor = types.ModuleType('sensor_msgs.msg')
        sensor.LaserScan = object
        geometry = load('geometry_under_test', ROOT / 'lidar_scan_geometry.py')
        modules = {'rclpy': ros, 'rclpy.node': node, 'sensor_msgs.msg': sensor,
                   'rplidar': vendor_module, 'lidar_serial_reader': reader,
                   'lidar_scan_geometry': geometry}
        with mock.patch.dict(sys.modules, modules):
            self.bridge = load('bridge_under_test', ROOT / 'rplidar_python_bridge.py')

    def test_main_reports_failed_reader_as_nonzero(self):
        node = mock.Mock()
        node.run.side_effect = DriverError('short packet')
        with mock.patch.object(self.bridge, 'RPLidarBridge', return_value=node):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(self.bridge.main(), 1)
        node.shutdown_lidar.assert_called_once()
        node.destroy_node.assert_called_once()

    def test_run_propagates_error_and_emits_one_shutdown_summary(self):
        node = self.bridge.RPLidarBridge.__new__(self.bridge.RPLidarBridge)
        node.get_logger = mock.Mock(return_value=mock.Mock())
        node.lidar = mock.Mock()
        node.lidar.iter_scans.side_effect = DriverError('short packet')
        node.lidar.timing_summary.return_value = {'reader': 'blocking_serial_read'}
        node._shutdown_done = False
        node.scan_count = 0
        node.scan_delivery_max_s = node.publish_max_s = 0.
        node.slow_events = deque(maxlen=32)
        output = io.StringIO()
        with mock.patch.object(self.bridge.time, 'sleep'), contextlib.redirect_stdout(output):
            with self.assertRaises(DriverError):
                node.run()
            node.shutdown_lidar()
        self.assertEqual(output.getvalue().count('LIDAR_TIMING_SUMMARY'), 1)
        node.lidar.disconnect.assert_called_once()
        self.assertFalse(node.lidar.capture_active)


if __name__ == '__main__':
    unittest.main()
