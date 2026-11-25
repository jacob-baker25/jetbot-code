#!/usr/bin/env python3
"""Audit deskew coverage and correction size without starting ROS."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "local_nodes"))

from lidar_deskew import (  # noqa: E402
    PoseTimeline, ScanDeskewer, pose_samples_from_connection, stamp_seconds)


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def open_read_only(path):
    path = Path(path).resolve()
    if not path.is_file():
        raise RuntimeError("Capture database does not exist: {}".format(path))
    return sqlite3.connect(path.as_uri() + "?mode=ro", uri=True)


def analyze(database, laser_x=0.0, laser_y=0.0, laser_yaw=math.pi):
    connection = open_read_only(database)
    try:
        timeline = PoseTimeline(pose_samples_from_connection(connection))
        deskewer = ScanDeskewer(
            timeline, laser_x=laser_x, laser_y=laser_y,
            laser_yaw=laser_yaw)
        scan_times = []
        translations = []
        rotations = []
        for encoded, in connection.execute(
                "SELECT payload FROM messages WHERE topic='/scan_slam' "
                "ORDER BY sequence"):
            payload = json.loads(encoded)
            duration = float(payload["scan_time"])
            end = stamp_seconds(payload["header"])
            start = end - duration
            scan_times.append(duration)
            if timeline.covers(start, end):
                start_pose = timeline.interpolate(start)
                end_pose = timeline.interpolate(end)
                translations.append(math.hypot(
                    end_pose[0] - start_pose[0],
                    end_pose[1] - start_pose[1]))
                rotations.append(abs(end_pose[2] - start_pose[2]))
            deskewer.deskew(payload)
        summary = deskewer.summary()
        summary.update({
            "database": str(Path(database).resolve()),
            "scan_time_s": {
                "median": percentile(scan_times, 0.50),
                "p95": percentile(scan_times, 0.95),
                "maximum": max(scan_times or [0.0]),
            },
            "scan_translation_m": {
                "median": percentile(translations, 0.50),
                "p95": percentile(translations, 0.95),
                "maximum": max(translations or [0.0]),
            },
            "scan_rotation_deg": {
                "median": math.degrees(percentile(rotations, 0.50)),
                "p95": math.degrees(percentile(rotations, 0.95)),
                "maximum": math.degrees(max(rotations or [0.0])),
            },
        })
        return summary
    finally:
        connection.close()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--laser-x", type=float, default=0.0)
    parser.add_argument("--laser-y", type=float, default=0.0)
    parser.add_argument("--laser-yaw", type=float, default=math.pi)
    parsed = parser.parse_args(args)
    print(json.dumps(analyze(
        parsed.database, parsed.laser_x, parsed.laser_y, parsed.laser_yaw),
        indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
