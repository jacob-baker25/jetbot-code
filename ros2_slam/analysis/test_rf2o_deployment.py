import ast
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

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
            backup_root = base / 'backups'
            legacy = backup_root / 'earlier_backup'
            legacy.mkdir(parents=True)
            (legacy / 'package.xml').write_text('preserved older backup')
            real_copy = shutil.copy2
            def checked_copy(*args, **kwargs):
                self.assertTrue((backup_root / 'COLCON_IGNORE').is_file())
                return real_copy(*args, **kwargs)
            with mock.patch.object(deploy.shutil, 'copy2', side_effect=checked_copy):
                backup = deploy.deploy(source, target, backup_root, baseline)
            self.assertEqual((backup / deploy.FILES[0]).read_bytes(), old)
            self.assertEqual((legacy / 'package.xml').read_text(), 'preserved older backup')
            self.assertFalse((target / 'COLCON_IGNORE').exists())
            changed = target / 'src/CLaserOdometry2D.cpp'
            changed.write_text('unreviewed odometry algorithm')
            before = {str(p.relative_to(target)):p.read_bytes() for p in target.rglob('*') if p.is_file()}
            with self.assertRaisesRegex(RuntimeError, 'Unexpected local changes'):
                deploy.deploy(source, target, base / 'backups', baseline)
            after = {str(p.relative_to(target)):p.read_bytes() for p in target.rglob('*') if p.is_file()}
            self.assertEqual(before, after)
            self.assertEqual(len([p for p in backup_root.iterdir() if p.is_dir()]), 2)

    def test_missing_or_broken_summary_never_qualifies_diagnostics(self):
        marker = 'RF2O_TIMING_ENABLED version=1 rmw=test\n'
        self.assertFalse(runner.parse_rf2o_timing(marker)['complete'])
        broken = runner.parse_rf2o_timing(marker+'RF2O_TIMING_SUMMARY broken\n')
        self.assertTrue(broken['parse_errors'])
        good = runner.parse_rf2o_timing(marker+'RF2O_TIMING_SUMMARY {"version":1}\n')
        self.assertTrue(good['complete'])
        self.assertFalse(runner.parse_rf2o_timing(marker*2+'RF2O_TIMING_SUMMARY {}')['complete'])

    def test_upgrading_previous_deployed_node_is_accepted_and_backed_up(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory); target = base / 'src'
            source = ROOT / 'rf2o_laser_odometry'
            shutil.copytree(str(source), str(target))
            node = target / 'src/CLaserOdometry2DNode.cpp'
            old = subprocess.check_output(['git', '-c', 'safe.directory='+str(ROOT.parent).replace('\\', '/'),
                'show', '84b79bf:ros2_slam/rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp'], cwd=str(ROOT.parent))
            node.write_bytes(old)
            self.assertEqual(deploy.digest(node),
                '9da90f14ca8967e924ce363481db2721d5702dd03a958bfffae278a617bc60ea')
            before = node.read_bytes()
            baseline = json.loads((ROOT / 'rf2o_timing_baseline.json').read_text())
            backup = deploy.deploy(source, target, base / 'backups', baseline)
            self.assertEqual((backup / 'src/CLaserOdometry2DNode.cpp').read_bytes(), before)
            self.assertEqual(deploy.digest(node), deploy.digest(source / 'src/CLaserOdometry2DNode.cpp'))

    def test_upgrade_from_actual_pre_repair_revision(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory); target = base / 'src'
            source = ROOT / 'rf2o_laser_odometry'
            target.mkdir()
            for rel in deploy.FILES:
                data = subprocess.check_output(['git', '-c', 'safe.directory='+str(ROOT.parent).replace('\\', '/'),
                    'show', 'f2d288b:ros2_slam/rf2o_laser_odometry/'+rel], cwd=str(ROOT.parent))
                path = target / rel; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(data)
            before = {rel:(target/rel).read_bytes() for rel in deploy.FILES}
            baseline = json.loads((ROOT / 'rf2o_timing_baseline.json').read_text())
            backup = deploy.deploy(source, target, base/'backups', baseline)
            for rel in deploy.FILES:
                self.assertEqual((backup/rel).read_bytes(), before[rel])
                self.assertEqual(deploy.digest(target/rel), deploy.digest(source/rel))

    def test_python36_syntax(self):
        for path in ('deploy_rf2o_timing.py','analysis/run_rf2o_core_tests.py'):
            ast.parse((ROOT / path).read_text(), feature_version=(3, 6))

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
            deploy.check_main_scope(ROOT / 'rf2o_laser_odometry', directory, compiler)
