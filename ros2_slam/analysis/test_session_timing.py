import ast
import contextlib
import importlib.util
import io
from pathlib import Path
import types
import unittest
import tempfile
import sys
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SESSION = ROOT / "local_nodes" / "mapping_session.py"
spec = importlib.util.spec_from_file_location("timing_check", str(ROOT / "analysis" / "check_session_timing.py"))
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


def method(name, globals_dict):
    tree = ast.parse(SESSION.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    definition = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[definition], type_ignores=[])
    namespace = dict(globals_dict)
    exec(compile(ast.fix_missing_locations(module), str(SESSION), "exec"), namespace)
    return namespace[name]


class SessionTimingTest(unittest.TestCase):
    def test_busy_callback_stream_does_not_delay_keyboard_or_watchdog(self):
        clock = [0.0]
        dispatched = []
        stopped = []
        terminal_restored = []
        def spin(node, timeout_sec):
            clock[0] += .001  # A ready callback: no need to idle.
            dispatched.append(clock[0])
        def poll(read, write, errors, timeout):
            clock[0] += timeout
            return (read if clock[0] >= .3 else [], [], [])
        ns = dict(rclpy=types.SimpleNamespace(ok=lambda: True, spin_once=spin),
                  sys=types.SimpleNamespace(stdin=types.SimpleNamespace(fileno=lambda: 0, read=lambda n: "q")),
                  select=types.SimpleNamespace(select=poll),
                  time=types.SimpleNamespace(monotonic=lambda: clock[0]), WATCHDOG_SECONDS=.15,
                  termios=types.SimpleNamespace(tcgetattr=lambda fd: "saved", TCSADRAIN=1,
                      tcsetattr=lambda *args: terminal_restored.append(args)),
                  tty=types.SimpleNamespace(setcbreak=lambda fd: None))
        node = types.SimpleNamespace(moving=True, last_motor_command=0.0, odom_mode="imu_yaw",
                                     require_processes_alive=lambda: None, print_controls=lambda: None)
        def stop():
            stopped.append(clock[0]); node.moving = False
        node.stop_robot = stop
        node.update_watchdog = lambda: method("update_watchdog", ns)(node)
        with contextlib.redirect_stdout(io.StringIO()):
            method("teleop_loop", ns)(node)
        self.assertGreater(len(dispatched), 100)  # Old 30ms wait dispatches only ~10.
        self.assertLessEqual(stopped[0], .152)
        self.assertTrue(terminal_restored)
        self.assertFalse(node.moving)

    def test_exception_stops_robot_and_restores_terminal(self):
        stopped, restored = [], []
        def fail(): raise RuntimeError("child died")
        node = types.SimpleNamespace(odom_mode="imu_yaw", require_processes_alive=fail,
                                     print_controls=lambda: None, stop_robot=lambda: stopped.append(True))
        ns = dict(sys=types.SimpleNamespace(stdin=types.SimpleNamespace(fileno=lambda: 0)),
                  rclpy=types.SimpleNamespace(ok=lambda: True),
                  termios=types.SimpleNamespace(tcgetattr=lambda fd: "saved", TCSADRAIN=1,
                      tcsetattr=lambda *args: restored.append(args)),
                  tty=types.SimpleNamespace(setcbreak=lambda fd: None))
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            method("teleop_loop", ns)(node)
        self.assertTrue(stopped and restored)

    def test_checkpoint_requires_all_three_fresh_pose_stamps(self):
        fn = method("checkpoint_ages", dict(math=__import__("math"), CHECKPOINT_MAX_AGE_SECONDS=.5))
        node = types.SimpleNamespace(get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=10000000000)))
        good = {k: {"stamp_s": 9.9} for k in ("rf2o", "slam", "raw_rf2o")}
        self.assertIsNotNone(fn(node, good))
        for bad in [8.9, 10.2, float("nan")]:
            snapshot = dict(good, raw_rf2o={"stamp_s": bad})
            self.assertIsNone(fn(node, snapshot))
        self.assertIsNone(fn(node, {"rf2o": good["rf2o"], "slam": good["slam"]}))

    def test_stale_checkpoint_is_not_appended_and_stops_robot(self):
        clock = [0.0]
        stopped, written = [], []
        node = types.SimpleNamespace(stop_robot=lambda: stopped.append(True),
            spin_for=lambda seconds: clock.__setitem__(0, clock[0] + seconds),
            snapshot_pose=lambda: None, checkpoint_ages=lambda snapshot: None,
            checkpoint_failures=[], checkpoints=[], write_results=lambda: written.append(True))
        ns = dict(time=types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0]),
                  rclpy=types.SimpleNamespace(ok=lambda: True), CHECKPOINT_SETTLE_SECONDS=.5,
                  CHECKPOINT_FRESH_WAIT_SECONDS=2)
        with contextlib.redirect_stdout(io.StringIO()):
            result = method("capture_checkpoint", ns)(node, "test")
        self.assertIsNone(result)
        self.assertEqual(node.checkpoints, [])
        self.assertEqual(len(node.checkpoint_failures), 1)
        self.assertTrue(stopped and written)
        self.assertLess(clock[0], 2.6)

    def test_checkpoint_waits_for_fresh_pose_then_records_raw_and_fused(self):
        clock = [0.0]
        snapshot = {k: {"stamp_s": 1.0, "x": 0, "y": 0, "yaw_deg": 0}
                    for k in ("rf2o", "slam", "raw_rf2o")}
        node = types.SimpleNamespace(stop_robot=lambda: None,
            spin_for=lambda seconds: clock.__setitem__(0, clock[0] + seconds),
            snapshot_pose=lambda: snapshot, checkpoint_ages=lambda s: (
                {k: .1 for k in snapshot} if clock[0] >= .6 else None),
            checkpoint_failures=[], checkpoints=[], write_results=lambda: None,
            print_checkpoint=lambda cp: None, odom_mode="imu_yaw")
        ns = dict(time=types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0]),
                  rclpy=types.SimpleNamespace(ok=lambda: True), CHECKPOINT_SETTLE_SECONDS=.5,
                  CHECKPOINT_FRESH_WAIT_SECONDS=2)
        cp = method("capture_checkpoint", ns)(node, "turn")
        self.assertGreaterEqual(cp["time_unix"], .6)
        self.assertIn("raw_rf2o", cp)
        self.assertEqual(len(node.checkpoints), 1)

    def test_quiet_observer_saves_pose_and_rate_without_live_writes(self):
        spec = importlib.util.spec_from_file_location("observer", str(ROOT / "local_nodes/observe_imu_timing.py"))
        observer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(observer)
        callbacks, calls, writes = {}, [0], []
        class FakeNode:
            def __init__(self, name): pass
            def create_subscription(self, kind, topic, cb, qos): callbacks[topic] = cb; return cb
            def get_clock(self): return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10000000000))
            def destroy_node(self): pass
        def spin(node, timeout_sec):
            calls[0] += 1
            pose = types.SimpleNamespace(position=types.SimpleNamespace(x=1.0, y=2.0),
                orientation=types.SimpleNamespace(x=0., y=0., z=0., w=1.))
            msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                sec=9, nanosec=calls[0] * 20000000)), angular_velocity=types.SimpleNamespace(z=.2),
                pose=types.SimpleNamespace(pose=pose))
            for callback in callbacks.values(): callback(msg)
        modules = {
            "rclpy": types.SimpleNamespace(init=lambda: None, ok=lambda: calls[0] < 5,
                                            spin_once=spin, shutdown=lambda: None),
            "rclpy.node": types.SimpleNamespace(Node=FakeNode),
            "rclpy.qos": types.SimpleNamespace(QoSProfile=lambda depth: types.SimpleNamespace(),
                QoSReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),
                QoSDurabilityPolicy=types.SimpleNamespace(VOLATILE=1)),
            "sensor_msgs.msg": types.SimpleNamespace(Imu=object),
            "nav_msgs.msg": types.SimpleNamespace(Odometry=object),
        }
        original_open = open
        class Writer:
            def __init__(self, stream): self.stream = stream
            def write(self, payload): writes.append(calls[0]); return self.stream.write(payload)
            def writelines(self, lines): writes.append(calls[0]); return self.stream.writelines(lines)
            def flush(self): writes.append(calls[0]); return self.stream.flush()
            def close(self): return self.stream.close()
        observer.open = lambda *a, **kw: Writer(original_open(*a, **kw))
        with tempfile.TemporaryDirectory() as folder:
            target = str(Path(folder) / "capture.jsonl")
            args = ["observer", target, "--quiet-memory", "--include-odom", "--seconds", "1"]
            with mock.patch.dict(sys.modules, modules), mock.patch.object(sys, "argv", args):
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    observer.main()
                self.assertNotIn("STATUS", output.getvalue())
                with self.assertRaises(FileExistsError): observer.main()
            import json
            records = [json.loads(line) for line in Path(target).read_text().splitlines()]
        self.assertEqual(writes, [5])
        self.assertEqual(len([r for r in records if r["kind"] == "odom"]), 10)
        self.assertEqual(next(r for r in records if r["kind"] == "imu")["yaw_rate_rad_s"], .2)
        self.assertEqual(next(r for r in records if r["kind"] == "odom")["x_m"], 1.)

    def fixture(self, extra_lag=.02):
        result = {"run_status": "complete", "checkpoints": [
            {"label": "start", "time_unix": 0}, {"label": "final", "time_unix": 30}]}
        capture = [{"kind": "start", "quiet_memory": True, "include_odom": True}]
        trajectory = []
        for i in range(150):
            stamp = 7 + i * .1
            capture.append({"kind": "odom", "topic": "/odom_rf2o", "stamp_ns": round(stamp * 1e9),
                            "unix_s": stamp + .08, "ros_age_s": .08})
            trajectory.append({"time_unix": stamp + .08 + extra_lag,
                               "rf2o_stamp_s": stamp, "rf2o_age_s": .08 + extra_lag})
        return result, trajectory, capture

    def test_independent_matching_accepts_fresh_rejects_one_second_backlog(self):
        self.assertTrue(timing.evaluate(*self.fixture())["passed"])
        self.assertFalse(timing.evaluate(*self.fixture(1.0))["passed"])

    def test_sparse_incomplete_and_future_data_do_not_pass(self):
        result, tr, cap = self.fixture()
        self.assertFalse(timing.evaluate(result, tr[:20], cap)["passed"])
        result["run_status"] = "failed_final_checkpoint"
        self.assertFalse(timing.evaluate(result, tr, cap)["passed"])
        result, tr, cap = self.fixture()
        tr[10]["rf2o_age_s"] = -.2
        self.assertFalse(timing.evaluate(result, tr, cap)["passed"])


if __name__ == "__main__":
    unittest.main()
