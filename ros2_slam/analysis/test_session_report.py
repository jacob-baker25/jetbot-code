import ast
import contextlib
import csv
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "local_nodes" / "session_report.py"
SESSION = ROOT / "local_nodes" / "mapping_session.py"
spec = importlib.util.spec_from_file_location("session_report", str(REPORT))
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def synthetic_rows():
    """10 Hz: still 6 s, LEFT 90 in 3 s (key stutter), still 6 s, RIGHT 90, still 6 s.
    Raw RF2O under-reads turns by 10%; SLAM ends at 88/-89 and lags 0.5 s."""
    rows = []
    t, odom, raw, slam, corr = 0.0, 0.0, 0.0, 0.0, 0.0
    phases = [(6.0, 0.0, "stop"), (3.0, 30.0, "left"), (6.0, 0.0, "stop"),
              (3.0, -30.0, "right"), (6.0, 0.0, "stop")]
    slam_targets = []
    for seconds, rate, command in phases:
        for step in range(int(seconds * 10)):
            # key-repeat stutter: brief "stop" rows inside the turn
            cmd = "stop" if command != "stop" and step % 10 == 9 else command
            odom += rate * 0.1
            raw += 0.9 * rate * 0.1
            slam_targets.append(odom * (88.0 / 90.0 if odom >= 0 else 89.0 / 90.0))
            slam = slam_targets[max(0, len(slam_targets) - 6)]
            corr = slam - odom
            rows.append([t, "00:00:00.0", cmd, "start" if t < 1 else "",
                         odom, rate, raw, slam, corr, 0.0, 0.0, 0.0, 0.0])
            t = round(t + 0.1, 3)
    return rows


def write_log(directory, rows, odom_mode="imu_yaw_hold"):
    with open(os.path.join(directory, report.HEADING_LOG_NAME), "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(report.HEADING_LOG_COLUMNS)
        for row in rows:
            writer.writerow([report.fmt(v, 3) if isinstance(v, float) else v for v in row])
    with open(os.path.join(directory, "results.json"), "w") as stream:
        json.dump({"configuration": {"odom_mode": odom_mode}}, stream)
    with open(os.path.join(directory, "imu.log"), "w") as stream:
        stream.write("Published 250 samples; bias_hold=on bias_dps=+0.0950 still_pct=71.0\n")


class SessionReportTest(unittest.TestCase):

    def test_unwrapper_is_continuous_across_180(self):
        unwrap = report.Unwrapper()
        values = [unwrap.update(a) for a in (170.0, 179.0, -178.0, -170.0)]
        self.assertEqual(values, [170.0, 179.0, 182.0, 190.0])
        self.assertIsNone(unwrap.update(None))

    def test_turns_compare_sources_over_same_settled_window(self):
        with tempfile.TemporaryDirectory() as directory:
            write_log(directory, synthetic_rows())
            rows = report.load_heading_log(os.path.join(directory, report.HEADING_LOG_NAME))
        turns = report.find_turns(rows)
        self.assertEqual([turn["direction"] for turn in turns], ["LEFT", "RIGHT"])
        self.assertAlmostEqual(turns[0]["odom_deg"], 90.0, delta=0.5)
        self.assertAlmostEqual(turns[0]["raw_deg"], 81.0, delta=0.5)
        self.assertAlmostEqual(turns[0]["slam_deg"], 88.0, delta=0.5)   # lag absorbed by settle
        self.assertEqual(report.agreement(turns[0]["slam_minus_odom_deg"]), "agree")
        self.assertAlmostEqual(turns[1]["odom_deg"], -90.0, delta=0.5)

    def test_agreement_thresholds(self):
        self.assertEqual(report.agreement(2.9), "agree")
        self.assertEqual(report.agreement(-4.9), "close")
        self.assertEqual(report.agreement(6.0), "DISAGREE")
        self.assertEqual(report.agreement(None), "no SLAM")

    def test_write_report_creates_files_in_session_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            write_log(directory, synthetic_rows())
            written = report.write_report(directory, plots=True, echo=False)
            names = sorted(os.path.basename(path) for path in written)
            text = Path(directory, report.REPORT_NAME).read_text()
        self.assertIn(report.REPORT_NAME, names)
        try:
            import matplotlib  # noqa: F401
            self.assertIn(report.HEADING_PLOT_NAME, names)
            self.assertIn(report.PATH_PLOT_NAME, names)
        except ImportError:
            pass
        self.assertIn("imu_yaw_hold", text)
        self.assertIn("bias +0.0950", text)
        self.assertIn("Turns: 2", text)

    def test_missing_or_empty_log_writes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(report.write_report(directory, echo=False), [])

    def test_session_writes_heading_log_and_report(self):
        text = SESSION.read_text(encoding="utf-8")
        self.assertIn("HEADING_LOG_COLUMNS", text)
        self.assertIn("self.write_heading_report()", text)
        self.assertIn('" --hold-still" if self.odom_mode == "imu_yaw_hold"', text)
        self.assertNotIn('self.odom_mode == "imu_yaw"', text)

    def test_live_heading_row_and_logging_failure_never_raises(self):
        import io
        import time
        import types
        tree = ast.parse(SESSION.read_text(encoding="utf-8"))
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name == "MappingSession")
        method = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                      and node.name == "record_heading_sample")
        module = ast.Module(body=[method], type_ignores=[])
        namespace = {"time": time, "fmt": report.fmt}
        exec(compile(ast.fix_missing_locations(module), "heading_row", "exec"), namespace)
        stream = io.StringIO()
        session = types.SimpleNamespace(
            heading_log_error=None, odom_mode="imu_yaw",
            last_raw_odom={"yaw_deg": 179.0}, session_start_unix=time.time() - 3.0,
            motion_command="left", heading_log_writer=csv.writer(stream),
            heading_unwrap={key: report.Unwrapper()
                            for key in ("odom", "raw", "slam", "correction")},
        )
        odom = {"x": 1.0, "y": 2.0, "yaw_deg": 45.0, "yaw_rate_dps": 30.0}
        slam = {"x": 1.1, "y": 2.1, "yaw_deg": 44.0}
        namespace["record_heading_sample"](session, odom, slam, {"yaw_deg": -1.0}, "start")
        row = next(csv.reader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(row), len(report.HEADING_LOG_COLUMNS))
        self.assertEqual(row[2:5], ["left", "start", "45.00"])
        self.assertEqual(row[6], "179.00")
        session.heading_log_writer = None          # simulate a broken writer
        with contextlib.redirect_stdout(io.StringIO()):
            namespace["record_heading_sample"](session, odom, slam, {"yaw_deg": 0.0}, "")
        self.assertIsNotNone(session.heading_log_error)

    def test_python36_syntax(self):
        ast.parse(REPORT.read_text(encoding="utf-8"), feature_version=(3, 6))


if __name__ == "__main__":
    unittest.main()
