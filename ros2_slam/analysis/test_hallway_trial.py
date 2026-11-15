import ast
import importlib.util
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard = load("motion_guard", ROOT / "local_nodes/motion_freshness.py")
runner = load("trial_runner", ROOT / "run_hallway_trial.py")


def session_method(name, namespace):
    tree = ast.parse((ROOT / "local_nodes/mapping_session.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])),
                 "session_method", "exec"), namespace)
    return namespace[name]


class HallwayTrialTest(unittest.TestCase):
    def test_motion_rejects_old_future_missing_and_nonfinite_data(self):
        fresh = {"stamp_s": 9.9, "receipt_mono": 19.9}
        guard.require_fresh_motion(fresh, fresh, 10., 20.)
        for bad in (None, {"stamp_s": 9., "receipt_mono": 20.},
                    {"stamp_s": 10., "receipt_mono": 19.},
                    {"stamp_s": 11., "receipt_mono": 20.},
                    {"stamp_s": float("nan"), "receipt_mono": 20.}):
            for pair in ((bad, fresh), (fresh, bad)):
                with self.assertRaises(RuntimeError):
                    guard.require_fresh_motion(*pair, 10., 20.)

    def test_stale_drive_rejected_but_zero_command_always_allowed(self):
        commands = []
        def reject(): raise RuntimeError("stale")
        node = types.SimpleNamespace(motion_guard_enabled=True, require_motion_data=reject,
            robot=types.SimpleNamespace(set_motors=lambda a, b: commands.append((a, b))))
        drive = session_method("drive", dict(clamp=lambda x: x, MOTOR_DIR_L=1,
                                             MOTOR_DIR_R=1, BIAS_L=1, BIAS_R=1))
        with self.assertRaises(RuntimeError): drive(node, .2, .2)
        self.assertEqual(commands, [])
        drive(node, 0., 0.)
        self.assertEqual(commands, [(0., 0.)])

    def test_stationary_phase_rejects_motion(self):
        node = types.SimpleNamespace(stationary_only=True)
        check = session_method("require_motion_data", {})
        with self.assertRaisesRegex(RuntimeError, "disabled"): check(node)

    def test_stale_during_motion_stops_and_restores_terminal(self):
        stopped, restored = [], []
        def reject(): raise RuntimeError('MOTION_DATA_STALE scan')
        node = types.SimpleNamespace(moving=True, motion_guard_enabled=True,
            odom_mode='imu_yaw', require_motion_data=reject,
            require_processes_alive=lambda:None, print_controls=lambda:None,
            stop_robot=lambda:stopped.append(True))
        ns = dict(rclpy=types.SimpleNamespace(ok=lambda:True, spin_once=lambda *a, **kw:None),
            sys=types.SimpleNamespace(stdin=types.SimpleNamespace(fileno=lambda:0)),
            termios=types.SimpleNamespace(tcgetattr=lambda fd:'saved', TCSADRAIN=1,
                tcsetattr=lambda *a:restored.append(True)),
            tty=types.SimpleNamespace(setcbreak=lambda fd:None))
        with self.assertRaisesRegex(RuntimeError, 'MOTION_DATA_STALE'):
            session_method('teleop_loop', ns)(node)
        self.assertTrue(stopped)
        self.assertTrue(restored)

    def test_archive_snapshots_sources_and_preserves_failure_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory); root = base / "ram_trial"; root.mkdir()
            source = base / "installed"; source.mkdir()
            original = b"# preserved installed source\n"
            (source / "node.py").write_bytes(original)
            hashes = runner.snapshot_sources(source, root / "sources")
            self.assertIn("node.py", hashes)
            runner.write_json(root / "trial_manifest.json", {"status": "failed"})
            (root / "observer.jsonl").write_text('{"kind":"stop"}\n')
            target = base / "trial.tar.gz"
            runner.archive_trial(root, target)
            self.assertEqual((source / "node.py").read_bytes(), original)
            with tarfile.open(target) as bundle:
                self.assertEqual(json.load(bundle.extractfile("ram_trial/trial_manifest.json"))["status"], "failed")
                self.assertEqual(bundle.extractfile("ram_trial/sources/node.py").read(), original)
            saved = target.read_bytes()
            with self.assertRaises(FileExistsError): runner.archive_trial(root, target)
            self.assertEqual(target.read_bytes(), saved)
            self.assertTrue(root.exists())

    def test_full_runner_archives_on_session_failure_and_stops_observer(self):
        self.exercise_runner(failed=True)

    def test_full_runner_preserves_normal_completion(self):
        self.exercise_runner(failed=False)

    def exercise_runner(self, failed):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory); ram = base / "ram"; ram.mkdir()
            observer = types.SimpleNamespace(returncode=None)
            def send(sig): observer.returncode = 0
            observer.poll = lambda: observer.returncode
            observer.send_signal = send
            observer.wait = lambda timeout: observer.returncode
            def spawn(command, **kwargs):
                self.assertTrue(kwargs['start_new_session'])
                kwargs['stdout'].write('Passive observer ready\n'); kwargs['stdout'].flush()
                Path(command[2]).write_text('{"kind":"stop"}\n')
                return observer
            class Session:
                def __init__(self, *args): self.processes=[]; self.process_log_paths={}
            fake = types.SimpleNamespace(MappingSession=Session)
            def fail():
                node = fake.MappingSession()
                self.assertEqual(node.processes, [("observer", observer)])
                self.assertEqual(fake.LOCAL_NODES, str(ram / "sources"))
                if failed:
                    raise RuntimeError('simulated process failure')
                raise SystemExit(0)
            fake.main = fail
            import os
            old_path = list(sys.path)
            try:
                with mock.patch.object(runner.Path, 'home', return_value=base), \
                     mock.patch.object(runner.shutil, 'disk_usage', return_value=types.SimpleNamespace(free=10**9)), \
                     mock.patch.object(runner.tempfile, 'mkdtemp', return_value=str(ram)), \
                     mock.patch.object(runner.subprocess, 'Popen', side_effect=spawn), \
                     mock.patch.object(runner.subprocess, 'check_output', return_value='testcommit\n'), \
                     mock.patch.object(runner, 'summarize', return_value={'passed':False}), \
                     mock.patch.dict(sys.modules, {'mapping_session':fake}), \
                     mock.patch.dict(os.environ, {}, clear=False), \
                     mock.patch.object(sys, 'argv', ['trial', 'test_failure']):
                    self.assertEqual(runner.main(), 1 if failed else 0)
            finally:
                sys.path[:] = old_path
            self.assertEqual(observer.returncode, 0)
            target = base / 'autochair_ros2_ws/experiments/test_failure_bundle.tar.gz'
            self.assertTrue(target.exists())
            manifest = json.loads((ram / 'trial_manifest.json').read_text())
            self.assertEqual(manifest['status'], 'failed' if failed else 'complete')
            if failed:
                self.assertIn('simulated process failure', manifest['error'])

    def test_python36_syntax(self):
        for path in [ROOT / 'run_hallway_trial.py', ROOT / 'local_nodes/motion_freshness.py',
                     ROOT / 'local_nodes/mapping_session.py', ROOT / 'local_nodes/observe_imu_timing.py']:
            ast.parse(path.read_text(encoding='utf8'), feature_version=(3, 6))

    def test_percentile_pass_does_not_hide_runtime_gap_from_transport_check(self):
        import csv
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); session = root / 'sessions/run'; session.mkdir(parents=True)
            result = {'run_status':'complete', 'configuration':{'odom_mode':'imu_yaw'},
                'checkpoints':[{'label':'start','time_unix':1}, {'label':'final','time_unix':30}]}
            runner.write_json(session / 'results.json', result)
            rows = [{'kind':'start','quiet_memory':True,'include_odom':True}]
            trajectory=[]
            for i in range(150):
                stamp=7+i*.1; receipt=stamp+.08
                for kind,topic in [('imu','/imu/data_raw'),('scan','/scan'),
                                   ('odom','/odom_rf2o_raw'),('odom','/odom_rf2o')]:
                    rows.append({'kind':kind,'topic':topic,'stamp_ns':round(stamp*1e9),
                                 'unix_s':receipt, 'ros_age_s':.08,
                                 'receipt_gap_s':.1,'stamp_gap_s':.1})
                trajectory.append({'time_unix':receipt+.02,'rf2o_stamp_s':stamp,'rf2o_age_s':.1})
            with (session / 'trajectory.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(trajectory[0]));writer.writeheader();writer.writerows(trajectory)
            capture=root / 'observer.jsonl'
            capture.write_text('\n'.join(json.dumps(r) for r in rows))
            report=runner.summarize(root,ROOT / 'analysis/check_session_timing.py')
            self.assertTrue(report['transport_ready_for_supervised_turns'])
            # A single old result has little effect on the p95 recorder statistic.
            late=next(r for r in rows if r.get('topic')=='/odom_rf2o_raw')
            late['receipt_gap_s']=.8;late['ros_age_s']=.8
            capture.write_text('\n'.join(json.dumps(r) for r in rows))
            report=runner.summarize(root,ROOT / 'analysis/check_session_timing.py')
            self.assertTrue(report['passed'])
            self.assertFalse(report['transport_ready_for_supervised_turns'])
