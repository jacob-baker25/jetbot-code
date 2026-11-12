#!/usr/bin/env python3
"""Compare session telemetry to independent quiet capture; Python 3.6, no ROS.

Thresholds are project acceptance targets, not hardware guarantees. Header
matching tolerates <1 microsecond of CSV float rounding. Measurements are
restricted to the teleop interval after the start checkpoint plus five seconds.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def percentile(values, fraction):
    values = sorted(values)
    return values[int((len(values) - 1) * fraction)] if values else None


def evaluate(result, trajectory, records):
    checkpoints = result.get("checkpoints", [])
    start = next((c for c in checkpoints if c["label"] == "start"), None)
    final = next((c for c in reversed(checkpoints) if c["label"] == "final"), None)
    errors = []
    if result.get("run_status") != "complete" or start is None or final is None:
        errors.append("Run must be complete with start and final checkpoints")
    quiet = next((r for r in records if r.get("kind") == "start"), {})
    if not quiet.get("quiet_memory") or not quiet.get("include_odom"):
        errors.append("Use --quiet-memory --include-odom capture")
    fused = {r["stamp_ns"]: r for r in records
             if r.get("kind") == "odom" and r.get("topic") == "/odom_rf2o"}
    keys = sorted(fused)
    pairs = []
    if start and final:
        for row in trajectory:
            recorded = float(row["time_unix"])
            if not start["time_unix"] + 5 <= recorded <= final["time_unix"]:
                continue
            stamp = float(row["rf2o_stamp_s"]) * 1e9
            index = bisect.bisect_left(keys, stamp)
            choices = keys[max(0, index - 1):index + 1]
            if not choices:
                continue
            key = min(choices, key=lambda k: abs(k - stamp))
            if abs(key - stamp) >= 1000:
                continue
            observed = fused[key]
            values = (recorded, float(row["rf2o_age_s"]),
                      recorded - observed["unix_s"], observed["ros_age_s"])
            if not all(math.isfinite(v) for v in values):
                errors.append("Nonfinite matched timing value")
                continue
            pairs.append(values)
    span = pairs[-1][0] - pairs[0][0] if len(pairs) > 1 else 0.0
    if len(pairs) < 100 or span < 10:
        errors.append("Need at least 100 matched trajectory rows spanning 10 seconds")
    metrics = {
        "matched_rows": len(pairs), "matched_span_s": span,
        "session_age_median_s": percentile([p[1] for p in pairs], .5),
        "session_age_p95_s": percentile([p[1] for p in pairs], .95),
        "session_extra_lag_median_s": percentile([p[2] for p in pairs], .5),
        "observer_fused_age_p95_s": percentile([p[3] for p in pairs], .95),
    }
    for key, limit in (("session_age_median_s", .25), ("session_age_p95_s", .5),
                       ("session_extra_lag_median_s", .1), ("observer_fused_age_p95_s", .25)):
        if metrics[key] is not None and metrics[key] > limit:
            errors.append("{} exceeds {:.3f}s target".format(key, limit))
    if any(p[1] < -.05 or p[3] < -.05 or p[2] < -.05 for p in pairs):
        errors.append("Future-dated measurement or inconsistent receiver clocks")
    return {"passed": not errors, "errors": sorted(set(errors)), "metrics": metrics}


def checkpoint_deltas(result):
    checkpoints = result.get("checkpoints", [])
    if not checkpoints:
        return []
    first = checkpoints[0]
    output = []
    for cp in checkpoints:
        row = {"label": cp["label"], "time_unix": cp["time_unix"],
               "pose_age_s": cp.get("pose_age_s")}
        for key in ("raw_rf2o", "rf2o", "slam"):
            if key in cp and key in first:
                delta = cp[key]["yaw_deg"] - first[key]["yaw_deg"]
                row[key + "_heading_change_deg"] = (delta + 180) % 360 - 180
        output.append(row)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    with (args.session / "results.json").open() as stream:
        result = json.load(stream)
    with (args.session / "trajectory.csv").open(newline="") as stream:
        trajectory = list(csv.DictReader(stream))
    with args.capture.open() as stream:
        records = [json.loads(line) for line in stream]
    report = evaluate(result, trajectory, records)
    report["checkpoint_heading_changes"] = checkpoint_deltas(result)
    report["interpretation"] = (
        "PASS qualifies recorded timing in the matched window only, not mapping accuracy "
        "or continuous sensor availability. rf2o fields mean the selected odometry "
        "(fused in imu_yaw mode); raw_rf2o is the separate lidar estimate. "
        "Compare settled headings with physical marks; gyro and fused yaw are not independent."
    )
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
