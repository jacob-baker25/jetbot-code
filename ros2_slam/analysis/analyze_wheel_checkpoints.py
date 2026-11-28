#!/usr/bin/env python3
"""Align raw Waveshare encoder frames to mapping-session checkpoints."""
from __future__ import print_function

import argparse
import bisect
import json
import math
from pathlib import Path


def wrap_degrees(value):
    return (value + 180.0) % 360.0 - 180.0


def distance_2d(first, second, x_key="x", y_key="y"):
    return math.hypot(
        second[x_key] - first[x_key], second[y_key] - first[y_key])


def load_frames(path):
    frames = []
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("version") != 1 or "receipt_unix" not in row:
                raise ValueError("invalid frame row {}".format(line_number))
            frames.append(row)
    if not frames:
        raise ValueError("encoder capture contains no frames")
    if any(frames[index]["receipt_unix"] < frames[index - 1]["receipt_unix"]
           for index in range(1, len(frames))):
        raise ValueError("encoder receipt timestamps are not monotonic")
    return frames


def nearest_frame(frames, stamps, target):
    index = bisect.bisect_left(stamps, target)
    candidates = []
    if index < len(frames):
        candidates.append(frames[index])
    if index:
        candidates.append(frames[index - 1])
    return min(candidates, key=lambda row: abs(row["receipt_unix"] - target))


def relative_pose(current, start):
    x = current["x_m"] - start["x_m"]
    y = current["y_m"] - start["y_m"]
    heading = start["yaw_rad"]
    return {
        "x_m": x,
        "y_m": y,
        "distance_m": math.hypot(x, y),
        "forward_m": math.cos(heading) * x + math.sin(heading) * y,
        "lateral_m": -math.sin(heading) * x + math.cos(heading) * y,
        "yaw_deg": wrap_degrees(math.degrees(
            current["yaw_rad"] - start["yaw_rad"])),
    }


def qualify_transport(summary, minimum_rate_hz=45.0, maximum_gap_s=0.1):
    parser = summary.get("parser", {})
    error_fields = {
        "decode_failures": summary.get("decode_failures"),
        "buffer_overflows": parser.get("buffer_overflows"),
        "checksum_failures": parser.get("checksum_failures"),
        "discarded_bytes": parser.get("discarded_bytes"),
        "invalid_sizes": parser.get("invalid_sizes"),
    }
    missing = sorted(key for key, value in error_fields.items() if value is None)
    if missing:
        raise ValueError("capture summary is missing: {}".format(
            ", ".join(missing)))
    rate = summary.get("decoded_rate_hz")
    gap = summary.get("maximum_receipt_gap_s")
    if rate is None or gap is None:
        raise ValueError("capture summary lacks rate or receipt-gap data")
    errors_passed = all(value == 0 for value in error_fields.values())
    return {
        "minimum_rate_hz": minimum_rate_hz,
        "observed_rate_hz": rate,
        "rate_passed": rate >= minimum_rate_hz,
        "maximum_allowed_gap_s": maximum_gap_s,
        "maximum_observed_gap_s": gap,
        "gap_passed": gap <= maximum_gap_s,
        "error_counts": error_fields,
        "errors_passed": errors_passed,
        "passed": rate >= minimum_rate_hz and gap <= maximum_gap_s and
        errors_passed,
    }


def qualify_path(report, expected_distances_m, error_per_meter=0.05,
                 maximum_lateral_error_m=0.05, maximum_closure_m=0.05):
    checkpoints = report["checkpoints"]
    if len(checkpoints) != len(expected_distances_m):
        raise ValueError(
            "expected {} distances for {} checkpoints".format(
                len(checkpoints), len(expected_distances_m)))
    if any(value < 0 for value in expected_distances_m):
        raise ValueError("expected distances must be nonnegative")

    measurements = []
    for checkpoint, expected in zip(checkpoints, expected_distances_m):
        wheel = checkpoint["wheel"]
        allowed = expected * error_per_meter
        error = abs(wheel["forward_m"] - expected)
        measurements.append({
            "index": checkpoint["index"],
            "label": checkpoint["label"],
            "expected_distance_m": expected,
            "measured_forward_m": wheel["forward_m"],
            "measured_lateral_m": wheel["lateral_m"],
            "absolute_distance_error_m": error,
            "maximum_allowed_distance_error_m": allowed,
            "distance_passed": expected == 0.0 or error <= allowed,
        })

    distance_passed = all(
        row["distance_passed"] for row in measurements
        if row["expected_distance_m"] > 0.0)
    maximum_lateral = max(
        abs(row["measured_lateral_m"]) for row in measurements)
    closure = checkpoints[-1]["wheel"]["distance_m"]
    return {
        "expected_distances_m": list(expected_distances_m),
        "distance_error_per_meter": error_per_meter,
        "measurements": measurements,
        "distance_passed": distance_passed,
        "maximum_allowed_lateral_error_m": maximum_lateral_error_m,
        "maximum_observed_lateral_error_m": maximum_lateral,
        "lateral_passed": maximum_lateral <= maximum_lateral_error_m,
        "maximum_allowed_closure_m": maximum_closure_m,
        "observed_closure_m": closure,
        "closure_passed": closure <= maximum_closure_m,
        "passed": distance_passed and
        maximum_lateral <= maximum_lateral_error_m and
        closure <= maximum_closure_m,
    }


def analyze(results, frames, maximum_age_s=0.1):
    checkpoints = results.get("checkpoints", [])
    if len(checkpoints) < 2:
        raise ValueError("at least two checkpoints are required")
    stamps = [row["receipt_unix"] for row in frames]
    matched = []
    start_frame = nearest_frame(frames, stamps, checkpoints[0]["time_unix"])
    start_wheel = start_frame["pose"]
    start_rf2o = checkpoints[0]["rf2o"]
    start_raw = checkpoints[0]["raw_rf2o"]

    for checkpoint in checkpoints:
        frame = nearest_frame(frames, stamps, checkpoint["time_unix"])
        age = frame["receipt_unix"] - checkpoint["time_unix"]
        wheel = relative_pose(frame["pose"], start_wheel)
        rf2o = {
            "x_m": checkpoint["rf2o"]["x"] - start_rf2o["x"],
            "y_m": checkpoint["rf2o"]["y"] - start_rf2o["y"],
        }
        rf2o["distance_m"] = math.hypot(rf2o["x_m"], rf2o["y_m"])
        raw = {
            "x_m": checkpoint["raw_rf2o"]["x"] - start_raw["x"],
            "y_m": checkpoint["raw_rf2o"]["y"] - start_raw["y"],
        }
        raw["distance_m"] = math.hypot(raw["x_m"], raw["y_m"])
        matched.append({
            "index": checkpoint["index"],
            "label": checkpoint["label"],
            "checkpoint_unix": checkpoint["time_unix"],
            "wheel_sequence": frame["sequence"],
            "wheel_receipt_offset_s": age,
            "wheel_receipt_age_abs_s": abs(age),
            "wheel": wheel,
            "wheel_motor": frame["motor"],
            "fused_rf2o": rf2o,
            "raw_rf2o": raw,
        })

    segments = []
    for previous, current in zip(matched, matched[1:]):
        segments.append({
            "from": previous["label"],
            "to": current["label"],
            "wheel_distance_m": distance_2d(
                previous["wheel"], current["wheel"], "x_m", "y_m"),
            "fused_rf2o_distance_m": distance_2d(
                previous["fused_rf2o"], current["fused_rf2o"], "x_m", "y_m"),
            "raw_rf2o_distance_m": distance_2d(
                previous["raw_rf2o"], current["raw_rf2o"], "x_m", "y_m"),
        })

    maximum_age = max(row["wheel_receipt_age_abs_s"] for row in matched)
    return {
        "version": 1,
        "checkpoint_count": len(matched),
        "encoder_frame_count": len(frames),
        "maximum_checkpoint_alignment_age_s": maximum_age,
        "maximum_allowed_alignment_age_s": maximum_age_s,
        "alignment_passed": maximum_age <= maximum_age_s,
        "checkpoints": matched,
        "segments": segments,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--maximum-age", type=float, default=0.1)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--expected-distances", help=(
        "comma-separated distances from the start in meters"))
    parser.add_argument("--distance-error-per-meter", type=float, default=0.05)
    parser.add_argument("--maximum-lateral-error", type=float, default=0.05)
    parser.add_argument("--maximum-closure", type=float, default=0.05)
    parser.add_argument("--minimum-rate", type=float, default=45.0)
    parser.add_argument("--maximum-gap", type=float, default=0.1)
    args = parser.parse_args(argv)
    report = analyze(
        json.loads(args.results.read_text()),
        load_frames(args.frames),
        args.maximum_age)
    if args.summary:
        report["transport_qualification"] = qualify_transport(
            json.loads(args.summary.read_text()),
            args.minimum_rate,
            args.maximum_gap)
    if args.expected_distances:
        expected = [float(value) for value in
                    args.expected_distances.split(",")]
        report["path_qualification"] = qualify_path(
            report,
            expected,
            args.distance_error_per_meter,
            args.maximum_lateral_error,
            args.maximum_closure)
    report["passed"] = report["alignment_passed"] and all(
        report[key]["passed"] for key in
        ("transport_qualification", "path_qualification") if key in report)
    encoded = json.dumps(report, indent=2, allow_nan=False, sort_keys=True)
    if args.output:
        args.output.write_text(encoded + "\n")
    print("WHEEL_CHECKPOINT_ANALYSIS " + json.dumps(
        report, allow_nan=False, sort_keys=True))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
