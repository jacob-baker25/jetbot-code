#!/usr/bin/env python3
"""Run one guarded SLAM replay entirely from RAM.

The caller must source ROS, the installed workspace, and the intended
slam_toolbox overlay first.  This runner owns every process group it starts,
archives logs and the final map even when a gate fails, and never opens robot
hardware.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tarfile
import time


ROOT = Path(__file__).resolve().parent
DEFAULT_PARAMS = (
    ROOT / "slam_toolbox" / "config"
    / "mapper_params_online_sync_guarded_strict.yaml"
)
CONFLICT_MARKERS = (
    "sync_slam_toolbox_node",
    "online_sync_launch.py",
    "static_transform_publisher",
    "replay_player.py",
    "rplidar_python_bridge.py",
    "rf2o_laser_odometry_node",
    "imu_yaw_odometry.py",
    "phidget_imu_publisher.py",
    "scan_delay_node.py",
)
PLAYER_SUMMARY_PREFIX = "REPLAY_PLAYER_SUMMARY "


class ReplayTrialError(RuntimeError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def validate_name(value):
    if not re.match(r"^[A-Za-z0-9][A-Za-z0-9_-]*$", value):
        raise argparse.ArgumentTypeError(
            "Use only letters, numbers, underscore, and hyphen in the run name")
    return value


def validate_commit(value):
    if not re.match(r"^[0-9a-f]{40}$", value):
        raise argparse.ArgumentTypeError("Source Git head must be 40 lowercase hex digits")
    return value


def find_conflicts(process_listing, own_pid):
    conflicts = []
    for raw in process_listing.splitlines():
        match = re.match(r"\s*(\d+)\s+(.*)$", raw)
        if not match or int(match.group(1)) == own_pid:
            continue
        command = match.group(2)
        if any(marker in command for marker in CONFLICT_MARKERS):
            conflicts.append({"pid": int(match.group(1)), "command": command})
    return conflicts


def parse_player_summary(text):
    summaries = []
    for line in text.splitlines():
        if line.startswith(PLAYER_SUMMARY_PREFIX):
            summaries.append(json.loads(line[len(PLAYER_SUMMARY_PREFIX):]))
    if len(summaries) != 1:
        raise ReplayTrialError(
            "Expected one replay-player summary, found {}".format(len(summaries)))
    return summaries[0]


def stream_process(command, log_path, start_new_session=True):
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1,
        start_new_session=start_new_session)
    lines = []
    try:
        with Path(log_path).open("w", encoding="utf-8") as log:
            for line in process.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                log.write(line)
                log.flush()
                lines.append(line)
        return process.wait(), "".join(lines)
    except BaseException:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        raise


class ManagedProcess(object):
    def __init__(self, name, command, log_path):
        self.name = name
        self.log_path = Path(log_path)
        self.log = self.log_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            start_new_session=True)
        self.group = self.process.pid

    def group_exists(self):
        try:
            os.killpg(self.group, 0)
            return True
        except ProcessLookupError:
            return False

    def signal_group(self, value):
        if self.group_exists():
            os.killpg(self.group, value)

    def stop(self):
        for value, wait_seconds in (
                (signal.SIGINT, 3.0),
                (signal.SIGTERM, 2.0),
                (signal.SIGKILL, 1.0)):
            if not self.group_exists():
                break
            self.signal_group(value)
            deadline = time.monotonic() + wait_seconds
            while self.group_exists() and time.monotonic() < deadline:
                time.sleep(0.05)
        try:
            self.process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass
        self.log.close()


def ros_node_ready(process, node_name="/slam_toolbox", timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.process.poll() is not None:
            return False
        result = subprocess.run(
            ["ros2", "node", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            universal_newlines=True)
        if node_name in result.stdout.splitlines():
            return True
        time.sleep(1.0)
    return False


def write_json(path, value):
    Path(path).write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")


def archive_directory(directory, archive):
    with tarfile.open(str(archive), "w:gz") as output:
        output.add(str(directory), arcname=directory.name)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("name", type=validate_name)
    result.add_argument("--source-git-head", type=validate_commit, required=True)
    result.add_argument("--database", type=Path, required=True)
    result.add_argument("--overlay-root", type=Path, required=True)
    result.add_argument("--params-file", type=Path, default=DEFAULT_PARAMS)
    result.add_argument("--output-root", type=Path, default=Path("/dev/shm"))
    result.add_argument("--gate-mode", default="euclidean_heading")
    result.add_argument("--rate", type=float, default=1.0)
    result.add_argument("--maximum-lateness", type=float, default=0.25)
    result.add_argument("--maximum-scan-rotation-degrees", type=float)
    result.add_argument("--maximum-scan-translation-meters", type=float)
    result.add_argument("--deskew-scans", action="store_true")
    result.add_argument("--laser-x", type=float, default=0.0)
    result.add_argument("--laser-y", type=float, default=0.0)
    result.add_argument("--laser-z", type=float, default=0.1778)
    result.add_argument("--laser-yaw", type=float, default=3.14159265)
    return result


def main(args=None):
    parsed = parser().parse_args(args)
    if parsed.rate <= 0 or parsed.maximum_lateness <= 0:
        parser().error("rate and maximum lateness must be positive")
    if (parsed.maximum_scan_rotation_degrees is not None
            and not 0 < parsed.maximum_scan_rotation_degrees <= 180):
        parser().error("maximum scan rotation must be in (0, 180] degrees")
    if (parsed.maximum_scan_translation_meters is not None
            and not 0 < parsed.maximum_scan_translation_meters <= 1):
        parser().error("maximum scan translation must be in (0, 1] meters")
    if (parsed.deskew_scans and (
            parsed.maximum_scan_rotation_degrees is not None
            or parsed.maximum_scan_translation_meters is not None)):
        parser().error("deskew and scan-motion filtering are mutually exclusive")

    database = parsed.database.resolve()
    params = parsed.params_file.resolve()
    overlay_root = parsed.overlay_root.resolve()
    output_root = parsed.output_root.resolve()
    output = output_root / parsed.name
    archive = output_root / (parsed.name + ".tar.gz")
    expected_prefix = overlay_root / "ws" / "install" / "slam_toolbox"

    for path, label in ((database, "database"), (params, "parameter file")):
        if not path.is_file():
            raise ReplayTrialError("Missing {}: {}".format(label, path))
    if output.exists() or archive.exists():
        raise ReplayTrialError("Output already exists for {}".format(parsed.name))
    if shutil.disk_usage(str(output_root)).free < 64 * 1024 * 1024:
        raise ReplayTrialError("Need 64 MiB free at {}".format(output_root))

    prefix = subprocess.check_output(
        ["ros2", "pkg", "prefix", "slam_toolbox"],
        universal_newlines=True).strip()
    if Path(prefix).resolve() != expected_prefix:
        raise ReplayTrialError(
            "Wrong slam_toolbox prefix: {} (expected {})".format(
                prefix, expected_prefix))

    listing = subprocess.check_output(
        ["ps", "-eo", "pid=,args="], universal_newlines=True)
    conflicts = find_conflicts(listing, os.getpid())
    if conflicts:
        raise ReplayTrialError("Conflicting processes: {}".format(conflicts))

    output.mkdir()
    manifest = {
        "version": 1,
        "name": parsed.name,
        "source_git_head": parsed.source_git_head,
        "runner_sha256": sha256(Path(__file__).resolve()),
        "start_unix": time.time(),
        "database": str(database),
        "database_sha256": sha256(database),
        "params_file": str(params),
        "params_sha256": sha256(params),
        "slam_toolbox_prefix": str(expected_prefix),
        "gate_mode": parsed.gate_mode,
        "rate": parsed.rate,
        "maximum_lateness_s": parsed.maximum_lateness,
        "maximum_scan_rotation_degrees": parsed.maximum_scan_rotation_degrees,
        "maximum_scan_translation_meters": (
            parsed.maximum_scan_translation_meters),
        "deskew_scans": parsed.deskew_scans,
        "laser_transform": {
            "x": parsed.laser_x,
            "y": parsed.laser_y,
            "z": parsed.laser_z,
            "yaw": parsed.laser_yaw,
        },
    }
    static_process = None
    slam_process = None
    player_rc = None
    save_rc = None
    player_summary = None
    run_error = None

    try:
        static_process = ManagedProcess(
            "static_tf",
            [
                "ros2", "run", "tf2_ros", "static_transform_publisher",
                str(parsed.laser_x), str(parsed.laser_y), str(parsed.laser_z),
                str(parsed.laser_yaw), "0", "0", "base_link", "laser_frame",
            ],
            output / "static_tf.log")
        slam_process = ManagedProcess(
            "slam",
            [
                "ros2", "launch", "slam_toolbox", "online_sync_launch.py",
                "use_sim_time:=false",
                "scan_distance_gate_mode:={}".format(parsed.gate_mode),
                "params_file:={}".format(params),
            ],
            output / "slam.log")
        if not ros_node_ready(slam_process):
            raise ReplayTrialError("SLAM node did not become ready")

        player_command = [
                "python3", str(ROOT / "local_nodes" / "replay_player.py"),
                "--database", str(database),
                "--rate", str(parsed.rate),
                "--start-delay", "3",
                "--linger", "2",
                "--maximum-lateness", str(parsed.maximum_lateness),
            ]
        if parsed.maximum_scan_rotation_degrees is not None:
            player_command.extend([
                "--maximum-scan-rotation-degrees",
                str(parsed.maximum_scan_rotation_degrees),
            ])
        if parsed.maximum_scan_translation_meters is not None:
            player_command.extend([
                "--maximum-scan-translation-meters",
                str(parsed.maximum_scan_translation_meters),
            ])
        if parsed.deskew_scans:
            player_command.extend([
                "--deskew-scans",
                "--laser-x", str(parsed.laser_x),
                "--laser-y", str(parsed.laser_y),
                "--laser-yaw", str(parsed.laser_yaw),
            ])
        player_rc, player_text = stream_process(
            player_command,
            output / "player.log")
        player_summary = parse_player_summary(player_text)

        time.sleep(7.0)
        saver = subprocess.Popen(
            [
                "python3", str(ROOT / "local_nodes" / "save_map_topic.py"),
                str(output / "map"),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            start_new_session=True)
        try:
            saver_text, unused = saver.communicate(timeout=20.0)
        except subprocess.TimeoutExpired:
            os.killpg(saver.pid, signal.SIGTERM)
            try:
                saver_text, unused = saver.communicate(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(saver.pid, signal.SIGKILL)
                saver_text, unused = saver.communicate()
        save_rc = saver.returncode
        sys.stdout.write(saver_text)
        (output / "save_map.log").write_text(saver_text, encoding="utf-8")
    except KeyboardInterrupt:
        run_error = "KeyboardInterrupt"
    except Exception as exc:
        run_error = "{}: {}".format(type(exc).__name__, exc)
    finally:
        if slam_process is not None:
            slam_process.stop()
        if static_process is not None:
            static_process.stop()

    map_pgm = output / "map.pgm"
    map_yaml = output / "map.yaml"
    passed = (
        run_error is None
        and player_rc == 0
        and save_rc == 0
        and player_summary is not None
        and player_summary.get("late_messages") == 0
        and map_pgm.is_file() and map_pgm.stat().st_size > 0
        and map_yaml.is_file() and map_yaml.stat().st_size > 0
    )
    manifest.update({
        "end_unix": time.time(),
        "player_returncode": player_rc,
        "save_returncode": save_rc,
        "player_summary": player_summary,
        "run_error": run_error,
        "map_pgm_sha256": sha256(map_pgm) if map_pgm.is_file() else None,
        "map_yaml_sha256": sha256(map_yaml) if map_yaml.is_file() else None,
        "passed": passed,
    })
    write_json(output / "replay_trial_manifest.json", manifest)
    archive_directory(output, archive)
    archive_hash = sha256(archive)
    print("REPLAY_TRIAL_RESULT " + json.dumps({
        "passed": passed,
        "archive": str(archive),
        "archive_sha256": archive_hash,
        "player_returncode": player_rc,
        "save_returncode": save_rc,
        "run_error": run_error,
    }, sort_keys=True), flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
