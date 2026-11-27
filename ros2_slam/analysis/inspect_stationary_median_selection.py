#!/usr/bin/env python3
"""Report stationary temporal-median scan selection without running ROS/SLAM."""
import argparse
import json
from pathlib import Path
import sqlite3
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "local_nodes"))

from lidar_deskew import PoseTimeline, pose_samples_from_connection
from stationary_scan_aggregation import StationaryMedianScanSelector


def inspect(database):
    database = Path(database).resolve()
    if not database.is_file():
        raise RuntimeError("Replay database does not exist: {}".format(database))
    connection = sqlite3.connect(database.as_uri() + "?mode=ro", uri=True)
    try:
        selector = StationaryMedianScanSelector.from_connection(
            connection,
            PoseTimeline(pose_samples_from_connection(connection)))
        return selector.summary()
    finally:
        connection.close()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parsed = parser.parse_args(args)
    print("STATIONARY_MEDIAN_SELECTION " + json.dumps(
        inspect(parsed.database), sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
