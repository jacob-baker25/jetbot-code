import ast
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


deploy = load('rf2o_deploy_test', ROOT / 'deploy_rf2o_timing.py')
runner = load('rf2o_runner_test', ROOT / 'run_hallway_trial.py')


class RF2OTimingTest(unittest.TestCase):
    def test_deployment_preserves_original_and_refuses_unexpected_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory); target = base / 'src'; source = ROOT / 'rf2o_laser_odometry'
            shutil.copytree(str(source), str(target))
            baseline = json.loads((ROOT / 'rf2o_timing_baseline.json').read_text())
            old = (target / deploy.FILES[0]).read_bytes()
            backup = deploy.deploy(source, target, base / 'backups', baseline)
            self.assertEqual((backup / deploy.FILES[0]).read_bytes(), old)
            changed = target / 'src/CLaserOdometry2D.cpp'
            changed.write_text('unreviewed odometry algorithm')
            before = {str(p.relative_to(target)):p.read_bytes() for p in target.rglob('*') if p.is_file()}
            with self.assertRaisesRegex(RuntimeError, 'Unexpected local changes'):
                deploy.deploy(source, target, base / 'backups', baseline)
            after = {str(p.relative_to(target)):p.read_bytes() for p in target.rglob('*') if p.is_file()}
            self.assertEqual(before, after)
            self.assertEqual(len(list((base / 'backups').iterdir())), 1)

    def test_missing_or_broken_summary_never_qualifies_diagnostics(self):
        marker = 'RF2O_TIMING_ENABLED version=1 rmw=test\n'
        self.assertFalse(runner.parse_rf2o_timing(marker)['complete'])
        broken = runner.parse_rf2o_timing(marker+'RF2O_TIMING_SUMMARY broken\n')
        self.assertTrue(broken['parse_errors'])
        good = runner.parse_rf2o_timing(marker+'RF2O_TIMING_SUMMARY {"version":1}\n')
        self.assertTrue(good['complete'])
        self.assertFalse(runner.parse_rf2o_timing(marker*2+'RF2O_TIMING_SUMMARY {}')['complete'])

    def test_python36_syntax(self):
        ast.parse((ROOT / 'deploy_rf2o_timing.py').read_text(), feature_version=(3, 6))

    def test_cpp14_diagnostics_if_compiler_available(self):
        compiler = shutil.which('g++') or shutil.which('clang++')
        if not compiler:
            self.skipTest('C++ compiler unavailable locally; deployment requires this test on Jetson')
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / 'timing_test.exe')
            subprocess.run([compiler, '-std=c++14', '-Wall', '-Wextra', '-pedantic',
                '-I'+str(ROOT / 'rf2o_laser_odometry/include'),
                str(ROOT / 'analysis/test_rf2o_timing.cpp'), '-o', executable], check=True)
            output = subprocess.check_output([executable], universal_newlines=True)
            summary = json.loads(output.split('RF2O_TIMING_SUMMARY ', 1)[1])
            self.assertEqual(len(summary['events']), 64)
            self.assertEqual(summary['event_count'], 102)
