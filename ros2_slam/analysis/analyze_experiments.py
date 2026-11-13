#!/usr/bin/env python3

"""Summarize mapping-session artifacts without ROS or third-party packages.

The script deliberately keeps estimated start-to-final displacement separate
from physical error.  A closed estimated trajectory is not ground truth unless
the robot was independently returned to a marked start pose and orientation.
"""

import argparse
import csv
import fnmatch
import json
import math
import os
import re
import statistics
import sys


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
POSE_PATTERN = re.compile(
    r"odom=\((%s),(%s),(%s)\).*?"
    r"search=\((%s),(%s),(%s)\).*?"
    r"best=\((%s),(%s),(%s)\).*?response=(%s)"
    % ((NUMBER,) * 10)
)
RUNTIME_MODE_PATTERN = re.compile(r"Scan distance gate mode:\s*(\S+)")
GATE_DISTANCE_PATTERN = re.compile(r"dist2=(%s)" % NUMBER)
GATE_REASON_PATTERN = re.compile(r"reason=(\S+)")


def wrap_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def distance(a, b):
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


def mean_or_none(values):
    return statistics.mean(values) if values else None


def max_or_none(values):
    return max(values) if values else None


def read_text(path):
    try:
        with open(path, "r", errors="replace") as source:
            return source.read()
    except (IOError, OSError):
        return ""


def read_json(path):
    try:
        with open(path, "r") as source:
            return json.load(source)
    except (IOError, OSError, ValueError):
        return None


def infer_requested_mode(name):
    lowered = name.lower()
    if "euclidean_heading" in lowered or "heading_gate" in lowered:
        return "euclidean_heading"
    if "euclidean" in lowered:
        return "euclidean"
    if "legacy" in lowered:
        return "legacy"
    return None


def trajectory_metrics(path):
    rows = []
    try:
        with open(path, "r", newline="") as source:
            for row in csv.DictReader(source):
                rows.append(row)
    except (IOError, OSError, ValueError):
        return {}

    if len(rows) < 2:
        return {"trajectory_samples": len(rows)}

    def value(row, key):
        return float(row[key])

    rf_path = 0.0
    slam_path = 0.0
    max_slam_step = 0.0
    max_correction_step = 0.0
    max_correction_yaw_step = 0.0

    for previous, current in zip(rows, rows[1:]):
        rf_path += math.hypot(
            value(current, "rf2o_x_m") - value(previous, "rf2o_x_m"),
            value(current, "rf2o_y_m") - value(previous, "rf2o_y_m"),
        )
        slam_step = math.hypot(
            value(current, "slam_x_m") - value(previous, "slam_x_m"),
            value(current, "slam_y_m") - value(previous, "slam_y_m"),
        )
        slam_path += slam_step
        max_slam_step = max(max_slam_step, slam_step)
        correction_step = math.hypot(
            value(current, "map_to_odom_x_m")
            - value(previous, "map_to_odom_x_m"),
            value(current, "map_to_odom_y_m")
            - value(previous, "map_to_odom_y_m"),
        )
        max_correction_step = max(max_correction_step, correction_step)
        correction_yaw_step = abs(
            wrap_degrees(
                value(current, "map_to_odom_yaw_deg")
                - value(previous, "map_to_odom_yaw_deg")
            )
        )
        max_correction_yaw_step = max(
            max_correction_yaw_step, correction_yaw_step
        )

    first = rows[0]
    last = rows[-1]
    elapsed = value(last, "elapsed_s") - value(first, "elapsed_s")
    sample_rate = (len(rows) - 1) / elapsed if elapsed > 0.0 else None

    return {
        "trajectory_samples": len(rows),
        "trajectory_duration_s": elapsed,
        "sample_rate_hz": sample_rate,
        "rf2o_path_m": rf_path,
        "slam_path_m": slam_path,
        "max_slam_step_m": max_slam_step,
        "max_map_to_odom_step_m": max_correction_step,
        "max_map_to_odom_yaw_step_deg": max_correction_yaw_step,
        "final_map_to_odom_m": math.hypot(
            value(last, "map_to_odom_x_m"),
            value(last, "map_to_odom_y_m"),
        ),
        "final_map_to_odom_yaw_deg": value(last, "map_to_odom_yaw_deg"),
    }


def log_metrics(slam_text, rf2o_text):
    responses = []
    translation_corrections = []
    heading_corrections = []

    for line in slam_text.splitlines():
        if "KARTO_SEQ_MATCH" not in line:
            continue
        match = POSE_PATTERN.search(line)
        if not match:
            continue
        values = [float(value) for value in match.groups()]
        search_x, search_y, search_heading = values[3:6]
        best_x, best_y, best_heading = values[6:9]
        responses.append(values[9])
        translation_corrections.append(
            math.hypot(best_x - search_x, best_y - search_y)
        )
        heading_corrections.append(
            abs(math.degrees(
                math.atan2(
                    math.sin(best_heading - search_heading),
                    math.cos(best_heading - search_heading),
                )
            ))
        )

    gate_distances = []
    gate_reasons = {}
    for line in slam_text.splitlines():
        if "SCAN_GATE_ACCEPT" not in line:
            continue
        distance_match = GATE_DISTANCE_PATTERN.search(line)
        if distance_match:
            gate_distances.append(math.sqrt(max(0.0, float(distance_match.group(1)))))
        reason_match = GATE_REASON_PATTERN.search(line)
        reason = reason_match.group(1) if reason_match else "unspecified"
        gate_reasons[reason] = gate_reasons.get(reason, 0) + 1

    mode_matches = RUNTIME_MODE_PATTERN.findall(slam_text)
    gate_accepts = slam_text.count("SCAN_GATE_ACCEPT")
    karto_states = slam_text.count("KARTO_SCAN_FINAL")

    return {
        "runtime_mode": mode_matches[-1] if mode_matches else None,
        "gate_accepts": gate_accepts,
        "gate_accept_reasons": gate_reasons,
        "heading_gate_accepts": (
            gate_reasons.get("heading", 0)
            + gate_reasons.get("distance+heading", 0)
        ),
        "gate_distance_mean_m": mean_or_none(gate_distances),
        "gate_distance_min_m": min(gate_distances) if gate_distances else None,
        "gate_distance_max_m": max_or_none(gate_distances),
        "karto_matches": len(responses),
        "karto_states": karto_states,
        "processed_state_efficiency": (
            float(karto_states) / float(gate_accepts + 1)
            if gate_accepts or karto_states
            else None
        ),
        "loop_closures": slam_text.count("KARTO_LOOP_ACCEPT"),
        "match_response_mean": mean_or_none(responses),
        "match_response_min": min(responses) if responses else None,
        "match_response_max": max_or_none(responses),
        "weak_matches_below_0_20": sum(1 for value in responses if value < 0.20),
        "match_translation_correction_mean_m": mean_or_none(
            translation_corrections
        ),
        "match_translation_correction_max_m": max_or_none(
            translation_corrections
        ),
        "match_heading_correction_mean_deg": mean_or_none(heading_corrections),
        "match_heading_correction_max_deg": max_or_none(heading_corrections),
        "tf_pose_failures": slam_text.count("Failed to compute odom pose"),
        "rf2o_eigensolver_warnings": sum(
            1 for line in rf2o_text.splitlines()
            if "eigensolver" in line.lower()
        ),
    }


def checkpoint_metrics(results):
    checkpoints = results.get("checkpoints", []) if results else []
    if len(checkpoints) < 2:
        return {"checkpoint_count": len(checkpoints)}
    first = checkpoints[0]
    last = checkpoints[-1]
    rf_distance = distance(first["rf2o"], last["rf2o"])
    slam_distance = distance(first["slam"], last["slam"])
    return {
        "checkpoint_count": len(checkpoints),
        "rf2o_start_final_m": rf_distance,
        "rf2o_start_final_heading_deg": wrap_degrees(
            last["rf2o"]["yaw_deg"] - first["rf2o"]["yaw_deg"]
        ),
        "slam_start_final_m": slam_distance,
        "slam_start_final_heading_deg": wrap_degrees(
            last["slam"]["yaw_deg"] - first["slam"]["yaw_deg"]
        ),
        "slam_minus_rf2o_start_final_m": slam_distance - rf_distance,
    }


def analyze_directory(path):
    name = os.path.basename(os.path.normpath(path))
    results = read_json(os.path.join(path, "results.json"))
    slam_text = read_text(os.path.join(path, "slam.log"))
    rf2o_text = read_text(os.path.join(path, "rf2o.log"))
    logs = log_metrics(slam_text, rf2o_text)

    requested_mode = None
    if results:
        requested_mode = results.get("configuration", {}).get(
            "scan_distance_gate_mode"
        )
    requested_inferred = False
    if not requested_mode:
        requested_mode = infer_requested_mode(name)
        requested_inferred = requested_mode is not None

    runtime_mode = (
        results.get("runtime_gate_mode") if results else None
    ) or logs["runtime_mode"]
    checkpoints = results.get("checkpoints", []) if results else []
    completed = bool(checkpoints and checkpoints[-1].get("label") == "final")
    status = results.get("run_status") if results else None
    profile = (results or {}).get("configuration", {}).get("matcher_profile")
    odom_mode = (results or {}).get("configuration", {}).get("odom_mode", "rf2o")
    config_passed = (results or {}).get("preflight", {}).get("karto_config") or {}
    odom_source_passed = (results or {}).get("preflight", {}).get("odom_source") or {}
    if not status:
        status = "complete" if completed else "incomplete"
    if requested_mode and runtime_mode and requested_mode != runtime_mode:
        validity = "exclude_mode_mismatch"
    elif profile and not config_passed.get("passed"):
        validity = "exclude_karto_config"
    elif odom_mode == "imu_yaw" and not odom_source_passed.get("passed"):
        validity = "exclude_odom_source"
    elif status != "complete":
        validity = "exclude_incomplete"
    elif not completed:
        validity = "exclude_incomplete"
    else:
        validity = "valid_complete"

    row = {
        "name": name,
        "path": os.path.abspath(path),
        "requested_mode": requested_mode,
        "requested_mode_inferred": requested_inferred,
        "runtime_mode": runtime_mode,
        "matcher_profile": profile or "historical_unverified",
        "odom_mode": odom_mode,
        "run_status": status,
        "validity": validity,
    }
    if results and results.get("session_start_unix") and results.get("session_end_unix"):
        row["session_duration_s"] = (
            results["session_end_unix"] - results["session_start_unix"]
        )
    row.update(checkpoint_metrics(results))
    row.update(trajectory_metrics(os.path.join(path, "trajectory.csv")))
    row.update(logs)
    # A missing marker must not erase a mode already recorded in results.json.
    row["runtime_mode"] = runtime_mode
    return row


def discover(root, patterns):
    directories = []
    for current, _, files in os.walk(root):
        if "results.json" not in files and "slam.log" not in files:
            continue
        name = os.path.basename(current)
        if patterns and not any(fnmatch.fnmatch(name, pattern) for pattern in patterns):
            continue
        directories.append(current)
    return sorted(directories)


def fmt(value, digits=3):
    if value is None or value == "":
        return "—"
    if isinstance(value, float):
        return ("{:.%df}" % digits).format(value)
    return str(value)


def markdown_report(rows, root):
    lines = [
        "# Mapping experiment analysis",
        "",
        "Source root: `{}`".format(os.path.abspath(root)),
        "",
        "Start-to-final values below are estimator closure values, not physical "
        "error unless the final pose was independently measured.",
        "",
        "## Run classification",
        "",
        "| Run | Requested | Runtime | Matcher profile | Odom mode | Validity | Karto states | Loops | "
        "RF2O closure (m) | SLAM closure (m) | SLAM heading (deg) |",
        "|---|---:|---:|---|---|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        requested = fmt(row.get("requested_mode"))
        if row.get("requested_mode_inferred"):
            requested += "*"
        lines.append(
            "| {name} | {requested} | {runtime} | {profile} | {odom} | {validity} | {states} | "
            "{loops} | {rf} | {slam} | {heading} |".format(
                name=row["name"],
                requested=requested,
                runtime=fmt(row.get("runtime_mode")),
                profile=row["matcher_profile"],
                odom=row["odom_mode"],
                validity=row["validity"],
                states=fmt(row.get("karto_states"), 0),
                loops=fmt(row.get("loop_closures"), 0),
                rf=fmt(row.get("rf2o_start_final_m")),
                slam=fmt(row.get("slam_start_final_m")),
                heading=fmt(row.get("slam_start_final_heading_deg"), 1),
            )
        )
    if any(row.get("requested_mode_inferred") for row in rows):
        lines.extend(["", r"\* Requested mode inferred from the directory name."])

    valid = [row for row in rows if row["validity"] == "valid_complete"]
    modes = sorted(set((row["runtime_mode"], row["matcher_profile"], row["odom_mode"])
                       for row in valid if row.get("runtime_mode")))
    lines.extend(["", "## Descriptive aggregates (valid completed runs only)", ""])
    lines.append(
        "| Runtime mode | Matcher profile | Odom mode | n | Mean Karto states | Mean match response | "
        "Mean RF2O closure (m) | Mean SLAM closure (m) | "
        "Mean abs SLAM heading (deg) |"
    )
    lines.append("|---|---|---|---:|---:|---:|---:|---:|---:|")
    for mode, profile, odom_mode in modes:
        subset = [row for row in valid if row.get("runtime_mode") == mode
                  and row["matcher_profile"] == profile
                  and row["odom_mode"] == odom_mode]
        lines.append(
            "| {mode} | {profile} | {odom} | {count} | {states} | {response} | {rf} | {slam} | "
            "{heading} |".format(
                mode=mode,
                profile=profile,
                odom=odom_mode,
                count=len(subset),
                states=fmt(mean_or_none([row["karto_states"] for row in subset])),
                response=fmt(mean_or_none([
                    row["match_response_mean"] for row in subset
                    if row.get("match_response_mean") is not None
                ])),
                rf=fmt(mean_or_none([
                    row["rf2o_start_final_m"] for row in subset
                    if row.get("rf2o_start_final_m") is not None
                ])),
                slam=fmt(mean_or_none([
                    row["slam_start_final_m"] for row in subset
                    if row.get("slam_start_final_m") is not None
                ])),
                heading=fmt(mean_or_none([
                    abs(row["slam_start_final_heading_deg"]) for row in subset
                    if row.get("slam_start_final_heading_deg") is not None
                ]), 1),
            )
        )

    lines.extend([
        "",
        "## Interpretation guardrails",
        "",
        "- Mode comparisons are descriptive; unequal manual paths and n≈2 do not "
        "support a significance claim.",
        "- A smaller estimated closure is not automatically a smaller physical "
        "error. Use a taped start pose and record longitudinal, lateral, and "
        "heading offsets.",
        "- `KARTO_SCAN_FINAL` is the processed pose-graph state count. It is more "
        "meaningful than wrapper gate accepts because downstream Karto may reject "
        "a wrapper-accepted scan.",
        "",
    ])
    return "\n".join(lines)


def write_csv(path, rows):
    keys = sorted(set(key for row in rows for key in row.keys()))
    with open(path, "w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            flat = dict(row)
            flat["gate_accept_reasons"] = json.dumps(
                flat.get("gate_accept_reasons", {}), sort_keys=True
            )
            writer.writerow(flat)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", help="Directory containing experiment folders")
    parser.add_argument(
        "--name", action="append", default=[],
        help="Optional experiment-directory glob; may be repeated",
    )
    parser.add_argument("--markdown", help="Write the Markdown report here")
    parser.add_argument("--csv", help="Write one metrics row per run here")
    args = parser.parse_args()

    directories = discover(args.root, args.name)
    if not directories:
        parser.error("no experiment directories found")
    rows = [analyze_directory(path) for path in directories]
    report = markdown_report(rows, args.root)

    if args.markdown:
        with open(args.markdown, "w", newline="") as output:
            output.write(report)
    else:
        sys.stdout.write(report)

    if args.csv:
        write_csv(args.csv, rows)


if __name__ == "__main__":
    main()
