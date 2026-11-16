#!/usr/bin/env python3
"""Run a supervised mapping trial from a source snapshot; archive after shutdown.

Python 3.6 / Eloquent. Does not deploy over installed local_nodes or build C++.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def snapshot_sources(source, destination):
    destination.mkdir()
    hashes = {}
    for path in sorted(source.glob("*.py")):
        data = path.read_bytes()
        (destination / path.name).write_bytes(data)
        hashes[path.name] = hashlib.sha256(data).hexdigest()
    return hashes


def archive_trial(root, target):
    """Never replace an existing result; keep RAM originals on any failure."""
    required = sum(p.stat().st_size for p in root.rglob("*") if p.is_file())
    if shutil.disk_usage(str(target.parent)).free < required + 8 * 1024 * 1024:
        raise RuntimeError("Not enough disk space to preserve trial; RAM files kept at " + str(root))
    # Exclusive creation protects earlier archives. Failed partials have a distinct name.
    partial = target.with_name(target.name + ".partial")
    if target.exists():
        raise FileExistsError(str(target))
    with partial.open("xb") as stream:
        with tarfile.open(fileobj=stream, mode="w:gz") as bundle:
            bundle.add(str(root), arcname=root.name)
    with tarfile.open(str(partial), "r:gz") as bundle:
        if not any(m.name.endswith("trial_manifest.json") for m in bundle.getmembers()):
            raise RuntimeError("Archive validation failed; RAM files retained")
    # Single launcher owns this name; avoid replacing a result if another writer appeared.
    if target.exists():
        raise FileExistsError(str(target))
    os.link(str(partial), str(target))  # Atomic no-replace on the destination filesystem.
    partial.unlink()


def summarize(root, analysis_path):
    """Evaluate recorded timing and expose worst gaps, including ones p95 hides."""
    import csv
    spec = importlib.util.spec_from_file_location("trial_timing", str(analysis_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    records = [json.loads(line) for line in (root / "observer.jsonl").read_text().splitlines()]
    results = list((root / "sessions").glob("*/results.json"))
    if len(results) != 1:
        return {"passed": False, "errors": ["Exactly one saved session is required"]}
    result = json.loads(results[0].read_text())
    with (results[0].parent / "trajectory.csv").open(newline="") as stream:
        trajectory = list(csv.DictReader(stream))
    report = module.evaluate(result, trajectory, records)
    # This describes the C++ binary that actually ran, not the Python snapshot.
    rf2o_log = results[0].parent / 'rf2o.log'
    if rf2o_log.exists():
        report['rf2o_diagnostics'] = parse_rf2o_timing(rf2o_log.read_text(errors='replace'))
        report['rf2o_validity'] = parse_rf2o_validity(rf2o_log.read_text(errors='replace'))
    report["checkpoint_heading_changes"] = module.checkpoint_deltas(result)
    report["streams"] = {}
    checkpoints = result.get("checkpoints", [])
    start = next((c["time_unix"] for c in checkpoints if c["label"] == "start"), None)
    final = next((c["time_unix"] for c in reversed(checkpoints) if c["label"] == "final"), None)
    for kind, topic in (("imu", "/imu/data_raw"), ("scan", "/scan"),
                        ("odom", "/odom_rf2o_raw"), ("odom", "/odom_rf2o")):
        rows = [r for r in records if r["kind"] == kind and
                (kind == "imu" or r.get("topic") == topic)]
        report["streams"][topic] = {
            "count": len(rows),
            "max_receipt_gap_s": max([r.get("receipt_gap_s") or 0 for r in rows] or [0]),
            "max_age_s": max([r["ros_age_s"] for r in rows] or [0]),
            "backward_or_duplicate_stamps": sum(r.get("stamp_gap_s") is not None and
                                                 r["stamp_gap_s"] <= 0 for r in rows),
        }
        active = [r for r in rows if start is not None and final is not None and
                  start <= r["unix_s"] <= final]
        report["streams"][topic]["active_count"] = len(active)
        report["streams"][topic]["active_max_receipt_gap_s"] = max(
            [r.get("receipt_gap_s") or 0 for r in active] or [0])
        report["streams"][topic]["active_max_age_s"] = max(
            [r["ros_age_s"] for r in active] or [0])
        report["streams"][topic]["active_min_age_s"] = min(
            [r["ros_age_s"] for r in active] or [0])
    required = ["/scan", "/odom_rf2o"]
    if result.get("configuration", {}).get("odom_mode") == "imu_yaw":
        required.extend(["/imu/data_raw", "/odom_rf2o_raw"])
    report["transport_ready_for_supervised_turns"] = report["passed"] and all(
        report["streams"][topic]["active_count"] >= 100 and
        report["streams"][topic]["active_max_receipt_gap_s"] <= .35 and
        report["streams"][topic]["active_max_age_s"] <= .35 and
        report["streams"][topic]["active_min_age_s"] >= -.05 and
        report["streams"][topic]["backward_or_duplicate_stamps"] == 0
        for topic in required)
    report["interpretation"] = "Timing PASS is not motion/map qualification; inspect worst gaps and physical measurements."
    return report


def parse_rf2o_timing(log):
    markers = [line for line in log.splitlines() if 'RF2O_TIMING_ENABLED version=1' in line]
    summaries = []
    errors = []
    for line in log.splitlines():
        if 'RF2O_TIMING_SUMMARY ' in line:
            try:
                summaries.append(json.loads(line.split('RF2O_TIMING_SUMMARY ', 1)[1]))
            except ValueError as exc:
                errors.append(str(exc))
    return {'enabled_markers': markers, 'summaries': summaries, 'parse_errors': errors,
            'complete': len(markers) == 1 and len(summaries) == 1 and not errors}


def parse_rf2o_validity(log):
    markers, summaries, inputs, errors = [], [], [], []
    for line in log.splitlines():
        if line.startswith('RF2O_ESTIMATE_POLICY '):
            markers.append(line)
        for label, target in (('RF2O_VALIDITY_SUMMARY ', summaries), ('RF2O_REJECT_INPUT ', inputs)):
            if line.startswith(label):
                try:
                    target.append(json.loads(line[len(label):]))
                except ValueError as exc:
                    errors.append(str(exc))
    return dict(markers=markers, summaries=summaries, rejected_inputs=inputs,
                parse_errors=errors, complete=len(markers)==1 and len(summaries)==1 and not errors)


def rf2o_deadband_command(command, mode):
    if mode is None:
        return command
    if mode not in ('legacy', 'off'):
        raise ValueError('Unknown RF2O deadband mode')
    if '--ros-args' not in command.split():
        command += ' --ros-args'
    return command + ' -p suppress_small_motion:=' + ('false' if mode=='off' else 'true')


def require_estimate_policy(log, mode):
    expected = 'RF2O_ESTIMATE_POLICY version=1 reference_max_s=0.35 deadband='+mode
    if expected not in log.splitlines():
        raise RuntimeError('RF2O estimate-policy marker missing/mismatched; build/deploy candidate before trial')


def rf2o_logging_command(command, level):
    if level not in ('info', 'warn'):
        raise ValueError('RF2O logging comparison supports info or warn')
    # rcl 0.8.5 accepts a plain severity, not logger:=severity. This applies
    # only to the RF2O process. WARN retains warnings and direct C++ timing.
    # INFO keeps the prior command exactly.
    if level == 'info':
        return command
    if '--ros-args' not in command.split():
        command += ' --ros-args'
    return command + ' --log-level warn'


def check_rcl_logging(level):
    if level == 'info':
        return
    # Exercise the installed shared rcl argument parser before starting nodes.
    # This is a compatibility check, not an RF2O runtime/accuracy test.
    subprocess.check_call([sys.executable, '-c',
        'import rclpy; rclpy.init(args=["rf2o_logging_check", "--ros-args", '
        '"--log-level", "warn"]); rclpy.shutdown()'], timeout=20)
    print('Installed rcl logging argument check: PASS', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name")
    parser.add_argument("--phase", choices=("stationary", "turns", "route"), default="stationary")
    parser.add_argument("--odom", choices=("imu_yaw", "rf2o"), default="imu_yaw")
    parser.add_argument("--rf2o-log-level", choices=("info", "warn"), default="info",
                        help="RF2O process only; warn suppresses routine per-scan INFO")
    parser.add_argument('--rf2o-deadband', choices=('legacy','off'), default=None,
                        help='Requires candidate estimate-policy marker; default leaves binary setting unchanged')
    args = parser.parse_args()
    if not re.match(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,70}$", args.name):
        parser.error("Use a short name containing letters, digits, underscores or hyphens")
    check_rcl_logging(args.rf2o_log_level)
    package = Path(__file__).resolve().parent
    output = Path.home() / "autochair_ros2_ws" / "experiments"
    output.mkdir(parents=True, exist_ok=True)
    target = output / (args.name + "_bundle.tar.gz")
    if target.exists() or target.with_name(target.name + ".partial").exists():
        parser.error("Output already exists; choose a new trial name")
    for directory in (Path("/dev/shm"), output):
        if shutil.disk_usage(str(directory)).free < 64 * 1024 * 1024:
            parser.error("Need 64 MiB free at " + str(directory))
    root = Path(tempfile.mkdtemp(prefix=args.name + "_", dir="/dev/shm"))
    hashes = snapshot_sources(package / "local_nodes", root / "sources")
    manifest = {"name": args.name, "phase": args.phase, "odom": args.odom,
                "rf2o_log_level": args.rf2o_log_level,
                "rf2o_log_scope": "process_default",
                "rf2o_deadband": args.rf2o_deadband,
                "start_unix": time.time(), "source_sha256": hashes,
                "source_directory": str(package), "output_directory": str(root),
                "installed_local_nodes_modified": False, "status": "starting"}
    try:
        manifest["git_head"] = subprocess.check_output(
            ["git", "-C", str(package), "rev-parse", "HEAD"], universal_newlines=True).strip()
    except (OSError, subprocess.CalledProcessError):
        manifest["git_head"] = None
    write_json(root / "trial_manifest.json", manifest)
    observer = None
    exit_code = 1
    observer_log = (root / "observer.log").open("w")
    try:
        observer = subprocess.Popen([
            sys.executable, str(root / "sources" / "observe_imu_timing.py"),
            str(root / "observer.jsonl"), "--seconds", "600", "--quiet-memory",
            "--include-odom", "--include-scan"], stdout=observer_log,
            stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            # MappingSession stops children with killpg. The observer must not
            # share the launcher's foreground group, or shutdown interrupts us.
            start_new_session=True)
        deadline = time.monotonic() + 20
        while "Passive observer ready" not in (root / "observer.log").read_text():
            if observer.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("Observer failed to become ready; see observer.log")
            time.sleep(.1)
        os.environ["JETBOT_STATIONARY_ONLY"] = "1" if args.phase == "stationary" else "0"
        sys.path.insert(0, str(root / "sources"))
        import mapping_session as session
        session.LOCAL_NODES = str(root / "sources")
        session.EXPERIMENT_ROOT = str(root / "sessions")
        base = session.MappingSession

        class TrialSession(base):
            def start_process(self, name, command):
                if name == 'rf2o':
                    command = rf2o_logging_command(command, args.rf2o_log_level)
                    command = rf2o_deadband_command(command, args.rf2o_deadband)
                    manifest['rf2o_command'] = command
                    write_json(root / 'trial_manifest.json', manifest)
                return super(TrialSession, self).start_process(name, command)

            def verify_odom_source(self):
                if not super(TrialSession, self).verify_odom_source():
                    return False
                if args.rf2o_deadband is not None:
                    require_estimate_policy(Path(self.process_log_paths['rf2o']).read_text(),
                                            args.rf2o_deadband)
                    print('RF2O estimate policy: PASS; deadband='+args.rf2o_deadband, flush=True)
                return True

            def __init__(self, *values):
                super(TrialSession, self).__init__(*values)
                # Observer termination (including its 10-minute cap) aborts the trial.
                self.processes.append(("observer", observer))
                self.process_log_paths["observer"] = str(root / "observer.log")

        session.MappingSession = TrialSession
        sys.argv = ["mapping_session.py", args.name, "euclidean_heading", "narrow", args.odom]
        print("Trial source snapshot: {}\nRAM output: {}\nPhase: {}".format(
            root / "sources", root, args.phase), flush=True)
        session.main()
        exit_code = 0
    except SystemExit as exc:
        exit_code = exc.code if isinstance(exc.code, int) else 1
    except (Exception, KeyboardInterrupt) as exc:
        manifest["error"] = repr(exc)
        print("Trial stopped: {}".format(exc), flush=True)
    finally:
        if observer is not None and observer.poll() is None:
            observer.send_signal(signal.SIGINT)
            try:
                observer.wait(timeout=15)
            except subprocess.TimeoutExpired:
                observer.terminate()
                observer.wait(timeout=5)
                manifest["observer_forced_stop"] = True
        observer_log.close()
        manifest.update(exit_code=exit_code, end_unix=time.time(),
                        status="complete" if exit_code == 0 else "failed")
        try:
            report = summarize(root, package / "analysis" / "check_session_timing.py")
            write_json(root / "timing_report.json", report)
            print("Recorded timing: {}".format("PASS" if report["passed"] else "FAIL"))
            print("Transport check for supervised turns: {}".format(
                "PASS" if report.get("transport_ready_for_supervised_turns") else "NOT YET"))
        except Exception as exc:
            manifest["analysis_error"] = repr(exc)
        write_json(root / "trial_manifest.json", manifest)
        archive_trial(root, target)
        print("Saved bundle: {}\nRAM originals retained: {}".format(target, root), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
