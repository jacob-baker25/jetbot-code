import importlib.util
import ast
import json
import math
import os
import shutil
import subprocess
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('capture', str(ROOT/'local_nodes/capture_sensors.py'))
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
from summarize_capture import summarize


class CaptureTest(unittest.TestCase):
    def test_zeroing_waits_without_closing_sensor(self):
        import sys
        with mock.patch.dict(sys.modules, {'capture_sensors':capture}):
            spec = importlib.util.spec_from_file_location('probe',str(ROOT/'local_nodes/phidget_probe.py'))
            probe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(probe)
        events=[]
        device=mock.Mock()
        device.zeroGyro.side_effect=lambda:events.append('zero')
        probe.zero_before_capture(device,lambda seconds:events.append(seconds))
        self.assertEqual(events,['zero',3.0])
        device.close.assert_not_called()

    def test_python_deployment_preserves_previous_files(self):
        bash = ('C:/Program Files/Git/bin/bash.exe' if os.name == 'nt'
                else shutil.which('bash'))
        if not bash or not Path(bash).is_file():
            self.skipTest('Bash required for deployment test')
        with tempfile.TemporaryDirectory() as folder:
            workspace = Path(folder)
            target = workspace/'local_nodes'
            target.mkdir()
            (target/'mapping_session.py').write_text('previous content')
            env = dict(os.environ, AUTOCHAIR_WS=workspace.as_posix())
            subprocess.run([bash,(ROOT/'deploy_sensor_diagnostics.sh').as_posix()],
                           env=env,check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            for name in ['mapping_session.py','matcher_profiles.py','capture_sensors.py',
                         'phidget_probe.py','imu_planar.py','phidget_imu_publisher.py',
                         'imu_yaw_odometry_core.py','imu_yaw_odometry.py']:
                self.assertEqual((target/name).read_bytes(),(ROOT/'local_nodes'/name).read_bytes())
            backup=list((workspace/'backups').glob('*/mapping_session.py'))
            self.assertEqual(len(backup),1)
            self.assertEqual(backup[0].read_text(),'previous content')

    def test_python36_syntax(self):
        for path in [ROOT/'local_nodes/capture_sensors.py',
                     ROOT/'local_nodes/phidget_probe.py',
                     ROOT/'local_nodes/imu_planar.py',
                     ROOT/'local_nodes/phidget_imu_publisher.py',
                     ROOT/'local_nodes/imu_yaw_odometry_core.py',
                     ROOT/'local_nodes/imu_yaw_odometry.py',
                     ROOT/'local_nodes/mapping_session.py',
                     ROOT/'analysis/summarize_capture.py']:
            ast.parse(path.read_text(encoding='utf-8'), feature_version=(3,6))

    def test_lookup_preserves_tf_stamp_and_ros_clock_age(self):
        tree = ast.parse((ROOT/'local_nodes/mapping_session.py').read_text(encoding='utf-8'))
        method = next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)
                      and n.name == 'lookup_pose')
        env = {'math':math, 'Time':lambda:None,
               'quaternion_to_yaw':lambda *q:0,
               'LookupException':LookupError, 'ConnectivityException':ConnectionError,
               'ExtrapolationException':ValueError}
        module = ast.Module(body=[method], type_ignores=[])
        exec(compile(ast.fix_missing_locations(module),'lookup','exec'), env)
        transform = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10,nanosec=250000000)),
            transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=1,y=2),
                                            rotation=types.SimpleNamespace(x=0,y=0,z=0,w=1)))
        owner = types.SimpleNamespace(
            tf_buffer=types.SimpleNamespace(lookup_transform=lambda *args:transform),
            get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=11000000000)))
        result=env['lookup_pose'](owner,'map','base_link')
        self.assertEqual(result['stamp_s'],10.25)
        self.assertEqual(result['age_s'],.75)
        self.assertEqual(result['x'],1)

    def test_recursive_ros_conversion_and_nonfinite(self):
        class Message:
            def get_fields_and_field_types(self):
                return {'ranges': 'sequence<float>'}
            ranges = [1.2, float('inf'), float('-inf'), float('nan')]
        result = capture.plain(Message())
        self.assertEqual(result, {'ranges':[1.2,'inf','-inf','nan']})
        json.dumps(result, allow_nan=False)

    def test_limits_no_overwrite_and_reserve(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder)/'capture.jsonl')
            writer = capture.BoundedWriter(path, 30, 1)
            self.assertTrue(writer.write({'topic':'a'}))
            self.assertFalse(writer.write({'topic':'x'*100}))
            self.assertEqual(writer.reason, 'size_limit')
            writer.close()
            with self.assertRaises(FileExistsError):
                capture.BoundedWriter(path, 100, 1)
            with mock.patch.object(capture.shutil, 'disk_usage', return_value=mock.Mock(free=0)):
                with self.assertRaises(RuntimeError):
                    capture.BoundedWriter(str(Path(folder)/'no.jsonl'),100,1)

    def test_gyro_integration_and_timestamp_faults(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'capture.jsonl'
            records=[]
            for t in [0,.1,.2,.2,.15]:
                records.append({'topic':'/imu/data','elapsed_s':len(records)*.1,
                                'receipt_ros_s':1+t+.02,
                                'msg':{'header':{'frame_id':'imu','stamp':{'sec':1,'nanosec':int(t*1e9)}},
                                       'angular_velocity':{'z':math.pi/2},
                                       'orientation_covariance':[-1]+[0]*8,
                                       'angular_velocity_covariance':[0]*9}})
            path.write_text('\n'.join(json.dumps(r) for r in records)+'\npartial')
            result=summarize(str(path))
            imu=result['topics']['/imu/data']
            self.assertAlmostEqual(imu['integrated_gyro_z_deg'],18)
            self.assertEqual(imu['duplicate_stamps'],1)
            self.assertEqual(imu['backward_stamps'],1)
            self.assertEqual(imu['integration_skipped_intervals'],2)
            self.assertEqual(result['malformed_records'],1)

    def test_phidget_units_and_device_timestamp_integration(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'raw.jsonl'
            rows=[{'topic':'phidget_raw','elapsed_s':t/1000,
                   'msg':{'device_timestamp_ms':t,'angular_rate_deg_s':[0,0,90]}}
                  for t in [0,100,200]]
            path.write_text('\n'.join(json.dumps(r) for r in rows))
            info=summarize(str(path))['topics']['phidget_raw']
            self.assertAlmostEqual(info['raw_sensor_z_integral_deg'],18)
            self.assertEqual(info['gyro_z_mean_deg_s'],90)
