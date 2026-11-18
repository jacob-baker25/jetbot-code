import ast
from collections import deque
import contextlib
import importlib.util
import io
import itertools
import math
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

    def start_motor(self):
        pass


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


vendor_module = types.ModuleType('rplidar')
vendor_module.RPLidar = Vendor
vendor_module.RPLidarException = DriverError
vendor_module._SCAN_TYPE = {'normal': {'size': 5, 'byte': b'\x20'}}


def fake_decode(raw):
    # Two vendor-contract golden packets, then deterministic opaque test packets.
    if raw in (bytes([61, 1, 45, 160, 15]), bytes([62, 1, 45, 160, 15])):
        return raw[0] == 61, 15, 90.0, 1000.0
    if raw[0] == 255:
        raise DriverError('malformed packet')
    return bool(raw[0]), raw[1], raw[2], raw[3] + 256*raw[4]


vendor_module._process_scan = fake_decode
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
            path = ROOT / name
            source = path.read_text()
            try:
                ast.parse(source, feature_version=(3, 6))
            except TypeError:
                compile(source, str(path), 'exec')


class BatchedReaderTest(unittest.TestCase):
    def test_contract_check_does_not_open_hardware_and_rejects_wrong_decoder(self):
        with mock.patch.object(Vendor, '__init__', side_effect=AssertionError('opened')):
            reader.check_batched_driver()
        with mock.patch.object(vendor_module, '_process_scan', return_value=(False, 0, 0, 0)):
            with self.assertRaisesRegex(DriverError, 'contract mismatch'):
                reader.BatchedRPLidar('fake')

    def test_batches_preserve_packet_order_with_40x_fewer_reads(self):
        lidar = reader.BatchedRPLidar('fake')
        packets = [bytes([0, 15, i % 256, 10, 1]) for i in range(400)]
        chunks = [b''.join(packets[i:i+40]) for i in range(0, 400, 40)]
        lidar._serial.read.side_effect = chunks
        lidar._serial.inWaiting.return_value = 0
        output = list(itertools.islice(lidar.iter_measures(), len(packets)))
        self.assertEqual(output, [fake_decode(p) for p in packets])
        self.assertEqual(lidar._serial.read.call_count, 10)
        self.assertTrue(all(c == mock.call(200) for c in lidar._serial.read.call_args_list))
        self.assertEqual(lidar._serial.inWaiting.call_count, 10)
        self.assertEqual(lidar.decoded_packets, 400)

    def test_short_batch_and_decoder_errors_are_fatal(self):
        for data, error in ((b'12345', 'expected 200 bytes'),
                            (bytes([255])*200, 'malformed packet')):
            lidar = reader.BatchedRPLidar('fake')
            lidar._serial.inWaiting.return_value = 0
            lidar._serial.read.return_value = data
            with self.assertRaisesRegex(DriverError, error):
                next(lidar.iter_measures())
        for mode, limit in (('express', 3000), ('normal', False), ('normal', 6000)):
            with self.assertRaisesRegex(DriverError, 'normal mode and 3000'):
                next(lidar.iter_measures(mode, limit))

    def test_overflow_retains_original_limit_and_records_recovery(self):
        lidar = reader.BatchedRPLidar('fake')
        lidar.capture_active = True
        lidar._serial.inWaiting.side_effect = [3000, 3082]
        lidar._serial.read.return_value = bytes([0, 1, 2, 3, 4])*40
        list(itertools.islice(lidar.iter_measures(), 41))
        summary = lidar.timing_summary()
        self.assertEqual(summary['recovery_count'], 1)
        self.assertEqual(summary['max_pending_bytes'], 3082)
        self.assertEqual(summary['overflow_events'][0]['pending_bytes'], 3082)
        self.assertEqual(summary['max_buffer_bytes'], 3000)
        self.assertEqual(lidar.recovery_generation, 1)

    def test_full_scans_only_and_no_mixing_across_recovery(self):
        lidar = reader.BatchedRPLidar('fake')
        def measures(*args):
            for _ in range(8): yield False, 1, 99, 100 # initial fragment
            yield True, 1, 0, 100
            for i in range(1, 8): yield False, 1, i, 100
            yield True, 1, 0, 200 # first complete scan delivered
            yield False, 1, 1, 200
            lidar.recovery_generation += 1
            for _ in range(8): yield False, 1, 99, 300 # restart fragment
            yield True, 1, 0, 400
            for i in range(1, 8): yield False, 1, i, 400
            yield True, 1, 0, 500
        lidar.iter_measures = measures
        scans = list(lidar.iter_scans())
        self.assertEqual(scans, [[(1, i, 100) for i in range(8)],
                                 [(1, i, 400) for i in range(8)]])
        self.assertEqual(lidar.discarded_partial_scans, 1)

    def test_prefetched_scan_boundary_cannot_bypass_overflow_limit(self):
        lidar = reader.BatchedRPLidar('fake')
        lidar.capture_active = True
        # 2,900 serial bytes plus the 200 prefetched bytes must trigger recovery.
        lidar._serial.inWaiting.side_effect = [0, 2900, 0, 0]
        old = bytes([1, 1, 1, 10, 0])
        new = bytes([1, 1, 1, 20, 0])
        lidar._serial.read.side_effect = [old*40, new*40]
        self.assertEqual(next(lidar.iter_measures()), fake_decode(new))
        self.assertEqual(lidar.discarded_batch_bytes, 200)
        self.assertEqual(lidar.recovery_count, 1)
        self.assertEqual(lidar.max_pending_bytes, 3100)


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
                   'lidar_scan_geometry': geometry,
                   'lidar_input_capture':load('input_capture_under_test',ROOT/'lidar_input_capture.py')}
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
        node.input_capture = mock.Mock()
        node.input_capture.summary.return_value = {'limit':0,'recorded':0}
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

    def test_input_recording_preserves_published_ranges_and_fractional_angles(self):
        node=self.bridge.RPLidarBridge.__new__(self.bridge.RPLidarBridge)
        stamp=types.SimpleNamespace(sec=12,nanosec=345)
        node.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:stamp))
        node.last_scan_time=0
        node.publisher=mock.Mock()
        node.input_capture=mock.Mock()
        raw=[(10,45.625,1234.5),(8,45.875,1250.0),(10,90.25,2000.0)]
        with mock.patch.object(self.bridge,'LaserScan',side_effect=lambda:types.SimpleNamespace(header=types.SimpleNamespace())):
            node.publish_scan(raw)
        msg=node.publisher.publish.call_args[0][0]
        self.assertEqual(msg.ranges[315],1.25)  # existing last-return-wins preserved
        self.assertEqual(msg.ranges[270],2.0)
        self.assertEqual(sum(math.isfinite(v) for v in msg.ranges),2)
        args=node.input_capture.record.call_args[0]
        self.assertIs(args[0],raw)
        self.assertIs(args[1],msg.ranges)
        self.assertEqual(args[0][0][1],45.625)


if __name__ == '__main__':
    unittest.main()
