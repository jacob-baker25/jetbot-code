#!/usr/bin/env python3
"""Run a supervised mapping trial from a source snapshot; archive after shutdown.

Python 3.6 / Eloquent. Does not deploy over installed local_nodes or build C++.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import shlex
import subprocess
import sys
import tarfile
import tempfile
import time


REPLAY_TOPICS_BASE = (
    "/scan",
    "/scan_slam",
    "/odom_rf2o",
    "/tf",
    "/tf_static",
)


def replay_topics(odom_mode):
    if odom_mode not in ("imu_yaw", "rf2o"):
        raise ValueError("Unknown odometry mode")
    topics = list(REPLAY_TOPICS_BASE)
    if odom_mode == "imu_yaw":
        topics.extend(("/imu/data_raw", "/odom_rf2o_raw"))
    return tuple(topics)


def rosbag_record_command(path, topics):
    if not topics or any(not topic.startswith("/") for topic in topics):
        raise ValueError("Replay topics must be absolute ROS topic names")
    return "ros2 bag record -o {} {}".format(
        shlex.quote(str(path)), " ".join(shlex.quote(topic) for topic in topics))


def check_rosbag_cli():
    """Fail before hardware startup if the Eloquent rosbag CLI is unavailable."""
    subprocess.check_call(
        ["ros2", "bag", "record", "--help"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=20,
    )


def require_rosbag_ready(log, topics):
    """Require rosbag2 to finish discovery before the operator may move."""
    if "All requested topics are subscribed" not in log:
        raise RuntimeError("Replay recorder did not finish topic discovery")
    missing = [topic for topic in topics if topic not in log]
    if missing:
        raise RuntimeError("Replay recorder did not report topics: " + ", ".join(missing))


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finalize_replay_capture(bag_directory, archive_path, info_path):
    """Validate and archive an external RAM bag without placing it on eMMC."""
    if not bag_directory.is_dir():
        raise RuntimeError("Replay bag directory was not created: " + str(bag_directory))
    files = sorted(path for path in bag_directory.rglob("*") if path.is_file())
    database_files = [path for path in files if path.suffix == ".db3" and path.stat().st_size]
    if not database_files:
        raise RuntimeError("Replay bag has no nonempty sqlite3 database")
    info = subprocess.check_output(
        ["ros2", "bag", "info", str(bag_directory)],
        universal_newlines=True,
        stderr=subprocess.STDOUT,
        timeout=30,
    )
    if "Messages:" not in info or "Topic information:" not in info:
        raise RuntimeError("ros2 bag info did not report messages and topics")
    info_path.write_text(info, encoding="utf-8")
    with archive_path.open("xb") as stream:
        with tarfile.open(fileobj=stream, mode="w:gz") as archive:
            archive.add(str(bag_directory), arcname=bag_directory.name)
    with tarfile.open(str(archive_path), "r:gz") as archive:
        members = archive.getmembers()
        if not any(member.name.endswith(".db3") and member.size for member in members):
            raise RuntimeError("Replay archive validation failed")
    return {
        "status": "complete",
        "directory": str(bag_directory),
        "archive": str(archive_path),
        "archive_bytes": archive_path.stat().st_size,
        "archive_sha256": sha256_file(archive_path),
        "file_count": len(files),
        "database_bytes": sum(path.stat().st_size for path in database_files),
    }


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


def resolve_source_git_head(package, explicit=None):
    """Return the source commit and how it was obtained without noisy git stderr."""
    if explicit is not None:
        if not re.match(r'^[0-9a-fA-F]{40}$', explicit):
            raise ValueError('source git head must be 40 hexadecimal characters')
        return explicit.lower(), 'argument'
    try:
        value = subprocess.check_output(
            ['git', '-C', str(package), 'rev-parse', 'HEAD'],
            universal_newlines=True, stderr=subprocess.DEVNULL).strip()
        return value, 'repository'
    except (OSError, subprocess.CalledProcessError):
        return None, 'unavailable'


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
    lidar_log = results[0].parent / 'lidar.log'
    if lidar_log.exists():
        report['lidar_acquisition'] = parse_lidar_acquisition(lidar_log.read_text(errors='replace'))
    slam_log = results[0].parent / 'slam.log'
    if slam_log.exists():
        slam_text = slam_log.read_text(errors='replace')
        report['karto_sequential_guard'] = parse_karto_decisions(slam_text)
        report['karto_graph_diagnostics'] = parse_karto_graph_diagnostics(slam_text)
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


def parse_lidar_acquisition(log):
    markers, summaries, errors = [], [], []
    for line in log.splitlines():
        if line.startswith('LIDAR_ACQUISITION '):
            markers.append(line)
        if line.startswith('LIDAR_TIMING_SUMMARY '):
            try:
                summaries.append(json.loads(line.split(' ', 1)[1]))
            except ValueError as exc:
                errors.append(str(exc))
    return dict(markers=markers, summaries=summaries, parse_errors=errors,
                complete=len(markers)==1 and len(summaries)==1 and not errors)


def parse_karto_decisions(log):
    rows = []
    errors = []
    for line in log.splitlines():
        if 'KARTO_SEQ_DECISION ' not in line:
            continue
        fields = dict(re.findall(r'(\w+)=([^ ]+)', line.split('KARTO_SEQ_DECISION ', 1)[1]))
        try:
            row = {
                'accepted': int(fields['accepted']),
                'reason': fields['reason'],
                'response': float(fields['response']),
                'translation_correction': float(fields['translation_correction']),
                'rotation_correction': float(fields['rotation_correction']),
                'accepted_count': int(fields['accepted_count']),
                'rejected_count': int(fields['rejected_count']),
            }
            if row['accepted'] not in (0, 1) or not all(math.isfinite(row[key]) for key in (
                    'response', 'translation_correction', 'rotation_correction')):
                raise ValueError('nonfinite or invalid guarded decision')
            rows.append(row)
        except (KeyError, ValueError) as exc:
            errors.append(str(exc))
    accepted = [row for row in rows if row['accepted']]
    rejected = [row for row in rows if not row['accepted']]
    reasons = {}
    for row in rejected:
        reasons[row['reason']] = reasons.get(row['reason'], 0) + 1
    return {
        'decision_count': len(rows),
        'accepted': len(accepted),
        'rejected': len(rejected),
        'rejection_reasons': reasons,
        'minimum_accepted_response': min([row['response'] for row in accepted] or [None]),
        'maximum_rejected_response': max([row['response'] for row in rejected] or [None]),
        'maximum_accepted_translation_correction': max(
            [row['translation_correction'] for row in accepted] or [0]),
        'maximum_accepted_rotation_correction': max(
            [row['rotation_correction'] for row in accepted] or [0]),
        'final_accepted_count': rows[-1]['accepted_count'] if rows else 0,
        'final_rejected_count': rows[-1]['rejected_count'] if rows else 0,
        'parse_errors': errors,
        'complete': not errors and (not rows or (
            rows[-1]['accepted_count'] == len(accepted) and
            rows[-1]['rejected_count'] == len(rejected))),
    }


def parse_karto_graph_diagnostics(log):
    """Summarize behavior-neutral near-chain, loop, and optimizer diagnostics."""
    near = []
    loops = []
    optimizations = []
    errors = []
    for line in log.splitlines():
        marker = next((value for value in (
            'KARTO_NEAR_DECISION ', 'KARTO_LOOP_DECISION ',
            'KARTO_OPTIMIZATION_SUMMARY ') if value in line), None)
        if marker is None:
            continue
        fields = dict(re.findall(r'(\w+)=([^ ]+)', line.split(marker, 1)[1]))
        try:
            if marker == 'KARTO_OPTIMIZATION_SUMMARY ':
                row = {
                    'correction_count': int(fields['correction_count']),
                    'applied_count': int(fields['applied_count']),
                    'maximum_translation': float(fields['maximum_translation']),
                    'maximum_rotation': float(fields['maximum_rotation']),
                }
                numeric = (row['maximum_translation'], row['maximum_rotation'])
                if min(row['correction_count'], row['applied_count']) < 0:
                    raise ValueError('negative optimization count')
                if not all(math.isfinite(value) for value in numeric):
                    raise ValueError('nonfinite graph diagnostic')
                optimizations.append(row)
            else:
                row = {
                    'accepted': int(fields['accepted']),
                    'response': float(fields['response']),
                    'translation_correction': float(fields['translation_correction']),
                    'rotation_correction': float(fields['rotation_correction']),
                    'state_id': int(fields['state_id']),
                    'chain_size': int(fields['chain_size']),
                }
                if row['accepted'] not in (0, 1) or min(
                        row['state_id'], row['chain_size']) < 0:
                    raise ValueError('invalid graph decision')
                numeric = (row['response'], row['translation_correction'],
                           row['rotation_correction'])
                if not all(math.isfinite(value) for value in numeric):
                    raise ValueError('nonfinite graph diagnostic')
                if marker == 'KARTO_LOOP_DECISION ':
                    row['stage'] = fields['stage']
                    if row['stage'] not in ('coarse', 'fine'):
                        raise ValueError('invalid loop stage')
                    loops.append(row)
                else:
                    near.append(row)
        except (KeyError, ValueError) as exc:
            errors.append(str(exc))

    def decisions(rows):
        accepted = [row for row in rows if row['accepted']]
        return {
            'attempted': len(rows),
            'accepted': len(accepted),
            'rejected': len(rows) - len(accepted),
            'maximum_accepted_translation_correction': max(
                [row['translation_correction'] for row in accepted] or [0]),
            'maximum_accepted_rotation_correction': max(
                [row['rotation_correction'] for row in accepted] or [0]),
        }

    loop_summary = {}
    for stage in ('coarse', 'fine'):
        loop_summary[stage] = decisions(
            [row for row in loops if row['stage'] == stage])
    return {
        'near_chain': decisions(near),
        'loop': loop_summary,
        'optimization_count': len(optimizations),
        'maximum_optimization_translation': max(
            [row['maximum_translation'] for row in optimizations] or [0]),
        'maximum_optimization_rotation': max(
            [row['maximum_rotation'] for row in optimizations] or [0]),
        'parse_errors': errors,
        'complete': not errors,
    }


def lidar_reader_command(command, mode):
    if mode not in ('blocking', 'batched'):
        raise ValueError('Unknown lidar reader')
    return 'env JETBOT_LIDAR_READER='+mode+' '+command


def lidar_resampling_command(command, mode):
    if mode not in ('legacy', 'cartesian'):
        raise ValueError('Unknown lidar resampling')
    return 'env JETBOT_LIDAR_RESAMPLING='+mode+' '+command


def lidar_input_command(command, count, path):
    if not isinstance(count,int) or not 0<=count<=1200:
        raise ValueError('LiDAR input capture limit must be 0..1200')
    # Explicit zero prevents an inherited shell setting from enabling capture.
    return 'env JETBOT_LIDAR_INPUT_SCANS={} JETBOT_LIDAR_INPUT_PATH={} {}'.format(
        count,shlex.quote(str(path)),command)


def require_lidar_input(log, count):
    if 'LIDAR_INPUT_CAPTURE version=1 limit={}'.format(count) not in log.splitlines():
        raise RuntimeError('LiDAR input capture marker missing/mismatched')


def require_lidar_reader(log, mode):
    size = 200 if mode == 'batched' else 5
    expected = 'LIDAR_ACQUISITION version=1 mode={} batch_bytes={} max_buffer_bytes=3000'.format(mode, size)
    if expected not in log.splitlines():
        raise RuntimeError('Lidar acquisition marker missing/mismatched')


def require_lidar_resampling(log, mode):
    expected = 'LIDAR_RESAMPLING version=1 mode='+mode
    if expected not in log.splitlines():
        raise RuntimeError('Lidar resampling marker missing/mismatched')


def rf2o_deadband_command(command, mode):
    if mode is None:
        return command
    if mode not in ('legacy', 'off'):
        raise ValueError('Unknown RF2O deadband mode')
    if '--ros-args' not in command.split():
        command += ' --ros-args'
    return command + ' -p suppress_small_motion:=' + ('false' if mode=='off' else 'true')


def require_pyramid_policy(log):
    expected = 'RF2O_PYRAMID_POLICY version=1 skip_rank_deficient_coarse=true require_finest=true'
    if expected not in log.splitlines():
        raise RuntimeError('RF2O pyramid-policy marker missing; deploy and rebuild RF2O')


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
    parser.add_argument('--lidar-reader', choices=('blocking', 'batched'), default='blocking',
                        help='Batched normal packets reduce serial calls; blocking retains prior reader')
    parser.add_argument('--lidar-resampling', choices=('legacy', 'cartesian'), default='legacy',
                        help='Cartesian mode uses bounded same-scan fractional-angle chords')
    parser.add_argument('--lidar-input-scans', type=int, default=0,
                        help='Capture up to 1200 pre-binning scans in RAM; default disabled')
    parser.add_argument('--matcher-profile', choices=('narrow', 'guarded', 'guarded_strict'),
                        default='narrow', help='Guarded profiles require the candidate SLAM overlay; '
                        'default preserves narrow')
    parser.add_argument('--source-git-head', default=None,
                        help='Pinned 40-hex source commit for an exported source snapshot')
    parser.add_argument('--record-replay', action='store_true',
                        help='Record a passive rosbag2 replay dataset in /dev/shm; never writes it to eMMC')
    parser.add_argument('--turn-speed', type=float, default=0.18,
                        help='Initial in-place turn motor command, 0.08..0.30; [ and ] adjust it live')
    args = parser.parse_args()
    if not 0<=args.lidar_input_scans<=1200:
        parser.error('--lidar-input-scans must be 0..1200')
    if not math.isfinite(args.turn_speed) or not 0.08 <= args.turn_speed <= 0.30:
        parser.error('--turn-speed must be finite and within 0.08..0.30')
    if args.source_git_head is not None and not re.match(
            r'^[0-9a-fA-F]{40}$', args.source_git_head):
        parser.error('--source-git-head must be exactly 40 hexadecimal characters')
    if not re.match(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,70}$", args.name):
        parser.error("Use a short name containing letters, digits, underscores or hyphens")
    package = Path(__file__).resolve().parent
    if args.lidar_reader == 'batched':
        subprocess.check_call([sys.executable, str(package/'local_nodes/lidar_serial_reader.py'),
                               '--check-batched-driver'], timeout=20)
    check_rcl_logging(args.rf2o_log_level)
    if args.record_replay:
        check_rosbag_cli()
    output = Path.home() / "autochair_ros2_ws" / "experiments"
    output.mkdir(parents=True, exist_ok=True)
    target = output / (args.name + "_bundle.tar.gz")
    if target.exists() or target.with_name(target.name + ".partial").exists():
        parser.error("Output already exists; choose a new trial name")
    required_ram = (256 if args.record_replay else 64) * 1024 * 1024
    if shutil.disk_usage("/dev/shm").free < required_ram:
        parser.error("Need {} MiB free at /dev/shm".format(required_ram // 1024 // 1024))
    if shutil.disk_usage(str(output)).free < 64 * 1024 * 1024:
        parser.error("Need 64 MiB free at " + str(output))
    root = Path(tempfile.mkdtemp(prefix=args.name + "_", dir="/dev/shm"))
    # Deterministic names make the separately retained RAM artifact easy to
    # copy, while the no-overwrite checks still protect an earlier capture.
    replay_bag = Path("/dev/shm") / (args.name + "_replay_bag")
    replay_archive = Path("/dev/shm") / (args.name + "_replay_bag.tar.gz")
    topics = replay_topics(args.odom)
    if replay_bag.exists() or replay_archive.exists():
        parser.error("Replay output already exists in /dev/shm")
    hashes = snapshot_sources(package / "local_nodes", root / "sources")
    manifest = {"name": args.name, "phase": args.phase, "odom": args.odom,
                "rf2o_log_level": args.rf2o_log_level,
                "rf2o_log_scope": "process_default",
                "rf2o_deadband": args.rf2o_deadband,
                "lidar_reader": args.lidar_reader,
                "lidar_resampling": args.lidar_resampling,
                "lidar_input_scans": args.lidar_input_scans,
                "record_replay": args.record_replay,
                "replay_topics": list(topics) if args.record_replay else [],
                "matcher_profile": args.matcher_profile,
                "initial_turn_speed": args.turn_speed,
                "start_unix": time.time(), "source_sha256": hashes,
                "source_directory": str(package), "output_directory": str(root),
                "installed_local_nodes_modified": False, "status": "starting"}
    manifest["git_head"], manifest["git_head_source"] = resolve_source_git_head(
        package, args.source_git_head)
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
            def start_stack(self):
                super(TrialSession, self).start_stack()
                if args.record_replay:
                    command = rosbag_record_command(replay_bag, topics)
                    manifest['replay_command'] = command
                    write_json(root / 'trial_manifest.json', manifest)
                    self.start_process('rosbag', command)

            def start_process(self, name, command):
                if name == 'lidar':
                    command = lidar_reader_command(command, args.lidar_reader)
                    command = lidar_resampling_command(command, args.lidar_resampling)
                    command = lidar_input_command(command,args.lidar_input_scans,root/'lidar_input.jsonl')
                    manifest['lidar_command'] = command
                    write_json(root / 'trial_manifest.json', manifest)
                if name == 'rf2o':
                    command = rf2o_logging_command(command, args.rf2o_log_level)
                    command = rf2o_deadband_command(command, args.rf2o_deadband)
                    manifest['rf2o_command'] = command
                    write_json(root / 'trial_manifest.json', manifest)
                return super(TrialSession, self).start_process(name, command)

            def verify_odom_source(self):
                if not super(TrialSession, self).verify_odom_source():
                    return False
                require_pyramid_policy(Path(self.process_log_paths['rf2o']).read_text())
                require_lidar_reader(Path(self.process_log_paths['lidar']).read_text(), args.lidar_reader)
                require_lidar_resampling(Path(self.process_log_paths['lidar']).read_text(),
                                         args.lidar_resampling)
                require_lidar_input(Path(self.process_log_paths['lidar']).read_text(),args.lidar_input_scans)
                print('Lidar acquisition policy: PASS; mode='+args.lidar_reader, flush=True)
                print('Lidar resampling policy: PASS; mode='+args.lidar_resampling, flush=True)
                if args.rf2o_deadband is not None:
                    require_estimate_policy(Path(self.process_log_paths['rf2o']).read_text(),
                                            args.rf2o_deadband)
                    print('RF2O estimate policy: PASS; deadband='+args.rf2o_deadband, flush=True)
                if args.record_replay:
                    require_rosbag_ready(
                        Path(self.process_log_paths['rosbag']).read_text(errors='replace'),
                        topics)
                    print('Replay recorder: PASS; all requested topics subscribed', flush=True)
                return True

            def __init__(self, *values):
                super(TrialSession, self).__init__(*values)
                self.turn_speed = args.turn_speed
                # Observer termination (including its 10-minute cap) aborts the trial.
                self.processes.append(("observer", observer))
                self.process_log_paths["observer"] = str(root / "observer.log")

            def stop_stack(self):
                # Keep the observer monitored during the trial, but leave its
                # graceful SIGINT/save to the launcher after the ROS stack is
                # down. MappingSession previously stopped it first, so the
                # launcher's finally block could not verify/save its RAM data.
                self.processes[:] = [
                    pair for pair in self.processes if pair[1] is not observer
                ]
                return super(TrialSession, self).stop_stack()

        session.MappingSession = TrialSession
        sys.argv = ["mapping_session.py", args.name, "euclidean_heading",
                    args.matcher_profile, args.odom]
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
        if observer is not None:
            manifest["observer_returncode"] = observer.poll()
        manifest["observer_saved_bytes"] = (
            (root / "observer.jsonl").stat().st_size
            if (root / "observer.jsonl").exists() else 0
        )
        if args.record_replay:
            try:
                manifest["replay_capture"] = finalize_replay_capture(
                    replay_bag, replay_archive, root / "rosbag_info.txt")
                print("Replay archive: {}\nReplay SHA256: {}".format(
                    replay_archive, manifest["replay_capture"]["archive_sha256"]),
                    flush=True)
            except Exception as exc:
                manifest["replay_capture"] = {
                    "status": "failed",
                    "directory": str(replay_bag),
                    "error": repr(exc),
                }
                exit_code = exit_code or 1
        manifest.update(exit_code=exit_code, end_unix=time.time(),
                        status="complete" if exit_code == 0 else "failed")
        try:
            report = summarize(root, package / "analysis" / "check_session_timing.py")
            write_json(root / "timing_report.json", report)
            print("Recorded timing: {}".format("PASS" if report["passed"] else "FAIL"))
            print("Stream freshness: {} (pose/map accuracy still requires review)".format(
                "PASS" if report.get("transport_ready_for_supervised_turns") else "NOT YET"))
        except Exception as exc:
            manifest["analysis_error"] = repr(exc)
        write_json(root / "trial_manifest.json", manifest)
        archive_trial(root, target)
        print("Saved bundle: {}\nRAM originals retained: {}".format(target, root), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
