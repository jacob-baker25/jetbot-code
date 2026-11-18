import ast
import importlib.util
import json
from pathlib import Path
import shlex
import sys
import tempfile
from types import SimpleNamespace
import unittest
ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module
capture=load('input_capture',ROOT/'local_nodes/lidar_input_capture.py')
runner=load('capture_runner',ROOT/'run_hallway_trial.py')


class InputCaptureTest(unittest.TestCase):
    def record(self,obj):
        raw=[[10,12.625,1234.5]];ranges=[float('inf')]*360
        obj.record(raw,ranges,SimpleNamespace(sec=12,nanosec=345),.0174532925,123.0)
        return raw,ranges

    def test_bounded_and_saved_afterwards_without_modifying_input(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'raw.jsonl';c=capture.LidarInputCapture(path,2)
            raw,ranges=self.record(c)
            self.assertFalse(path.exists())
            raw[0][1]=99
            self.record(c);self.record(c)
            self.assertEqual(c.summary()['recorded'],2)
            self.assertEqual(c.summary()['after_limit'],1)
            c.save();c.save()
            rows=[json.loads(l) for l in path.read_text().splitlines()]
            self.assertEqual(len(rows),2)
            self.assertEqual(rows[0]['raw_clockwise_quality_angle_deg_distance_mm'][0][1],12.625)
            self.assertIsNone(rows[0]['published_ranges_m'][0])
            self.assertEqual(rows[0]['stamp'],dict(sec=12,nanosec=345))
            self.assertEqual(ranges,[float('inf')]*360)

    def test_disabled(self):
        c=capture.LidarInputCapture(None,0);self.record(c);c.save()
        self.assertEqual(c.summary()['recorded'],0)

    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'x';p.write_text('keep')
            with self.assertRaises(ValueError):capture.LidarInputCapture(p,10)
            self.assertEqual(p.read_text(),'keep')

    def test_bounds(self):
        for n in [-1,1201,1.5]:
            with self.assertRaises(ValueError):capture.LidarInputCapture('x',n)
        c=capture.LidarInputCapture('x',600);c.raw_points=250000
        self.record(c)
        self.assertEqual(c.budget_drops,1);self.assertEqual(len(c.rows),0)

    def test_quoted_command_and_verified_mode(self):
        command=runner.lidar_input_command('python3 bridge.py',600,"/tmp/a b/input.jsonl")
        self.assertEqual(shlex.split(command),['env','JETBOT_LIDAR_INPUT_SCANS=600',
                         'JETBOT_LIDAR_INPUT_PATH=/tmp/a b/input.jsonl','python3','bridge.py'])
        runner.require_lidar_input('LIDAR_INPUT_CAPTURE version=1 limit=600',600)
        with self.assertRaises(RuntimeError):runner.require_lidar_input('',600)
        with self.assertRaises(ValueError):runner.lidar_input_command('x',1201,'x')

    def test_python36_syntax(self):
        for path in [ROOT/'local_nodes/lidar_input_capture.py',ROOT/'run_hallway_trial.py',
                     ROOT/'analysis/replay_scan_sequence.py',ROOT/'analysis/scan_gap_candidate.py']:
            if sys.version_info >= (3,8):
                ast.parse(path.read_text(),feature_version=(3,6))
            else:
                ast.parse(path.read_text())


if __name__=='__main__':unittest.main()
