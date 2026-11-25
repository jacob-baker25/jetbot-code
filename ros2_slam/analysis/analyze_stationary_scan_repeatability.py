#!/usr/bin/env python3
"""Audit stationary LiDAR repeatability without running or changing SLAM."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import statistics
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "local_nodes"))

from lidar_deskew import (  # noqa: E402
    PoseTimeline, pose_samples_from_connection, stamp_seconds)


class RepeatabilityError(RuntimeError):
    pass


def open_read_only(path):
    path = Path(path).resolve()
    if not path.is_file():
        raise RepeatabilityError("Capture database does not exist: {}".format(path))
    # sqlite3 on Windows does not decode the percent-escaped spaces produced
    # by Path.as_uri(), while the same URI works on Linux.  Forward slashes and
    # literal spaces are accepted on both platforms.
    uri = "file:" + str(path).replace("\\", "/") + "?mode=ro"
    return sqlite3.connect(uri, uri=True)


def percentile(values, fraction):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def wrapped_angle_delta(first, second):
    return math.atan2(math.sin(second - first), math.cos(second - first))


def finite_range(value, minimum, maximum):
    if value is None:
        return None
    value = float(value)
    if not math.isfinite(value) or value < minimum or value > maximum:
        return None
    return value


def group_stationary_samples(samples, maximum_gap_s=0.30,
                             maximum_episode_translation_m=0.03,
                             maximum_episode_rotation_rad=math.radians(3.0),
                             minimum_scans=8):
    episodes = []
    current = []
    for sample in samples:
        if current:
            anchor = current[0]
            gap = sample["stamp"] - current[-1]["stamp"]
            translation = math.hypot(
                sample["pose"][0] - anchor["pose"][0],
                sample["pose"][1] - anchor["pose"][1])
            rotation = abs(wrapped_angle_delta(
                anchor["pose"][2], sample["pose"][2]))
            if (gap > maximum_gap_s
                    or translation > maximum_episode_translation_m
                    or rotation > maximum_episode_rotation_rad):
                if len(current) >= minimum_scans:
                    episodes.append(current)
                current = []
        current.append(sample)
    if len(current) >= minimum_scans:
        episodes.append(current)
    return episodes


def summarize_episode(samples, minimum_observation_fraction=0.60,
                      stable_range_m=0.05):
    if not samples:
        raise RepeatabilityError("Cannot summarize an empty episode")
    bin_count = len(samples[0]["scan"].get("ranges", []))
    if bin_count < 2:
        raise RepeatabilityError("Stationary scan has fewer than two bins")
    range_bins = [[] for unused in range(bin_count)]
    quality_values = []
    valid_per_scan = []
    for sample in samples:
        scan = sample["scan"]
        ranges = scan.get("ranges", [])
        intensities = scan.get("intensities", [])
        if len(ranges) != bin_count:
            raise RepeatabilityError("Scan bin count changed within episode")
        minimum = float(scan.get("range_min", 0.0))
        maximum = float(scan.get("range_max", float("inf")))
        valid = 0
        for index, raw in enumerate(ranges):
            value = finite_range(raw, minimum, maximum)
            if value is None:
                continue
            range_bins[index].append(value)
            valid += 1
            if index < len(intensities) and intensities[index] is not None:
                quality = float(intensities[index])
                if math.isfinite(quality):
                    quality_values.append(quality)
        valid_per_scan.append(valid)

    required = int(math.ceil(minimum_observation_fraction * len(samples)))
    median_ranges = [None] * bin_count
    mads = []
    stable_bins = 0
    observed_bins = 0
    for index, values in enumerate(range_bins):
        if len(values) < required:
            continue
        center = statistics.median(values)
        mad = statistics.median([abs(value - center) for value in values])
        median_ranges[index] = center
        mads.append(mad)
        observed_bins += 1
        if mad <= stable_range_m:
            stable_bins += 1

    pose = (
        statistics.median(sample["pose"][0] for sample in samples),
        statistics.median(sample["pose"][1] for sample in samples),
        statistics.median(sample["pose"][2] for sample in samples),
    )
    pose_translation_radius = max(
        math.hypot(sample["pose"][0] - pose[0],
                   sample["pose"][1] - pose[1])
        for sample in samples)
    pose_rotation_radius = max(
        abs(wrapped_angle_delta(pose[2], sample["pose"][2]))
        for sample in samples)
    first_scan = samples[0]["scan"]
    public = {
        "start_stamp": samples[0]["stamp"],
        "end_stamp": samples[-1]["stamp"],
        "duration_s": samples[-1]["stamp"] - samples[0]["stamp"],
        "scan_count": len(samples),
        "pose": {"x_m": pose[0], "y_m": pose[1],
                 "yaw_deg": math.degrees(pose[2])},
        "pose_translation_radius_m": pose_translation_radius,
        "pose_rotation_radius_deg": math.degrees(pose_rotation_radius),
        "valid_bins_per_scan": {
            "median": percentile(valid_per_scan, 0.50),
            "p10": percentile(valid_per_scan, 0.10),
        },
        "well_observed_bins": observed_bins,
        "stable_bins": stable_bins,
        "stable_bin_fraction": (
            stable_bins / float(observed_bins) if observed_bins else 0.0),
        "range_mad_m": {
            "median": percentile(mads, 0.50),
            "p95": percentile(mads, 0.95),
            "maximum": max(mads) if mads else None,
        },
        "quality": {
            "count": len(quality_values),
            "p10": percentile(quality_values, 0.10),
            "median": percentile(quality_values, 0.50),
        },
    }
    private = {
        "median_ranges": median_ranges,
        "angle_min": float(first_scan["angle_min"]),
        "angle_increment": float(first_scan["angle_increment"]),
        "range_min": float(first_scan.get("range_min", 0.0)),
        "range_max": float(first_scan.get("range_max", float("inf"))),
        "pose_tuple": pose,
    }
    return public, private


def rotate(x, y, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def endpoint_cells(episode, resolution=0.05, laser_x=0.0,
                   laser_y=0.0, laser_yaw=math.pi):
    return cells_for_pose(
        local_endpoints(episode, laser_x, laser_y, laser_yaw),
        episode["pose_tuple"], resolution)


def local_endpoints(episode, laser_x=0.0, laser_y=0.0,
                    laser_yaw=math.pi):
    points = []
    offset_x, offset_y = rotate(laser_x, laser_y, 0.0)
    for index, distance in enumerate(episode["median_ranges"]):
        if distance is None:
            continue
        angle = (laser_yaw + episode["angle_min"]
                 + index * episode["angle_increment"])
        points.append((
            offset_x + distance * math.cos(angle),
            offset_y + distance * math.sin(angle)))
    return points


def cells_for_pose(points, pose, resolution):
    cells = set()
    for local_x, local_y in points:
        rotated_x, rotated_y = rotate(local_x, local_y, pose[2])
        x = pose[0] + rotated_x
        y = pose[1] + rotated_y
        cells.add((int(math.floor(x / resolution)),
                   int(math.floor(y / resolution))))
    return cells


def neighborhood_overlap(first, second, radius=1):
    if not first:
        return 0, 0.0
    matched = 0
    for x, y in first:
        found = False
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                if (x + offset_x, y + offset_y) in second:
                    found = True
                    break
            if found:
                break
        if found:
            matched += 1
    return matched, matched / float(len(first))


def closest_revisit(public_episodes, private_episodes, minimum_separation_s,
                    maximum_translation_m, maximum_rotation_rad,
                    resolution, laser_x, laser_y, laser_yaw):
    candidates = []
    for first in range(len(public_episodes)):
        for second in range(first + 1, len(public_episodes)):
            elapsed = (public_episodes[second]["start_stamp"]
                       - public_episodes[first]["end_stamp"])
            if elapsed < minimum_separation_s:
                continue
            first_pose = private_episodes[first]["pose_tuple"]
            second_pose = private_episodes[second]["pose_tuple"]
            translation = math.hypot(
                second_pose[0] - first_pose[0],
                second_pose[1] - first_pose[1])
            rotation = abs(wrapped_angle_delta(first_pose[2], second_pose[2]))
            if (translation > maximum_translation_m
                    or rotation > maximum_rotation_rad):
                continue
            score = (translation / maximum_translation_m
                     + rotation / maximum_rotation_rad)
            candidates.append((score, first, second, elapsed,
                               translation, rotation))
    if not candidates:
        return None
    unused_score, first, second, elapsed, translation, rotation = min(candidates)
    first_cells = endpoint_cells(
        private_episodes[first], resolution, laser_x, laser_y, laser_yaw)
    second_cells = endpoint_cells(
        private_episodes[second], resolution, laser_x, laser_y, laser_yaw)
    exact_first, exact_first_fraction = neighborhood_overlap(
        first_cells, second_cells, 0)
    exact_second, exact_second_fraction = neighborhood_overlap(
        second_cells, first_cells, 0)
    near_first, near_first_fraction = neighborhood_overlap(
        first_cells, second_cells, 1)
    near_second, near_second_fraction = neighborhood_overlap(
        second_cells, first_cells, 1)
    return {
        "first_episode": first,
        "second_episode": second,
        "elapsed_s": elapsed,
        "pose_translation_m": translation,
        "pose_rotation_deg": math.degrees(rotation),
        "first_endpoint_cells": len(first_cells),
        "second_endpoint_cells": len(second_cells),
        "exact": {
            "first_count": exact_first,
            "first_fraction": exact_first_fraction,
            "second_count": exact_second,
            "second_fraction": exact_second_fraction,
        },
        "within_one_cell": {
            "first_count": near_first,
            "first_fraction": near_first_fraction,
            "second_count": near_second,
            "second_fraction": near_second_fraction,
        },
    }


def optimize_revisit_alignment(first_episode, second_episode,
                               resolution=0.05, laser_x=0.0, laser_y=0.0,
                               laser_yaw=math.pi,
                               translation_window_m=0.20,
                               translation_step_m=0.025,
                               rotation_window_rad=math.radians(10.0),
                               rotation_step_rad=math.radians(1.0)):
    """Find a bounded rigid correction maximizing symmetric endpoint overlap."""
    first_points = local_endpoints(
        first_episode, laser_x, laser_y, laser_yaw)
    second_points = local_endpoints(
        second_episode, laser_x, laser_y, laser_yaw)
    first_cells = cells_for_pose(
        first_points, first_episode["pose_tuple"], resolution)
    recorded_pose = second_episode["pose_tuple"]
    best = None
    translation_steps = int(round(translation_window_m / translation_step_m))
    rotation_steps = int(round(rotation_window_rad / rotation_step_rad))
    evaluated = 0
    for x_step in range(-translation_steps, translation_steps + 1):
        delta_x = x_step * translation_step_m
        for y_step in range(-translation_steps, translation_steps + 1):
            delta_y = y_step * translation_step_m
            for yaw_step in range(-rotation_steps, rotation_steps + 1):
                delta_yaw = yaw_step * rotation_step_rad
                candidate_pose = (
                    recorded_pose[0] + delta_x,
                    recorded_pose[1] + delta_y,
                    recorded_pose[2] + delta_yaw)
                second_cells = cells_for_pose(
                    second_points, candidate_pose, resolution)
                unused_count, first_fraction = neighborhood_overlap(
                    first_cells, second_cells, 1)
                unused_count, second_fraction = neighborhood_overlap(
                    second_cells, first_cells, 1)
                symmetric = min(first_fraction, second_fraction)
                mean_fraction = 0.5 * (first_fraction + second_fraction)
                correction_size = (
                    math.hypot(delta_x, delta_y) / translation_window_m
                    + abs(delta_yaw) / rotation_window_rad)
                score = (symmetric, mean_fraction, -correction_size)
                evaluated += 1
                if best is None or score > best[0]:
                    best = (score, delta_x, delta_y, delta_yaw,
                            first_fraction, second_fraction, len(second_cells))
    if best is None:
        raise RepeatabilityError("Alignment search evaluated no candidates")
    (unused_score, delta_x, delta_y, delta_yaw, first_fraction,
     second_fraction, second_count) = best
    return {
        "evaluated_poses": evaluated,
        "translation_window_m": translation_window_m,
        "translation_step_m": translation_step_m,
        "rotation_window_deg": math.degrees(rotation_window_rad),
        "rotation_step_deg": math.degrees(rotation_step_rad),
        "best_correction": {
            "x_m": delta_x,
            "y_m": delta_y,
            "yaw_deg": math.degrees(delta_yaw),
            "translation_m": math.hypot(delta_x, delta_y),
        },
        "within_one_cell": {
            "first_fraction": first_fraction,
            "second_fraction": second_fraction,
            "symmetric_minimum_fraction": min(
                first_fraction, second_fraction),
        },
        "first_endpoint_cells": len(first_cells),
        "second_endpoint_cells": second_count,
    }


def analyze(database, output, translation_threshold=0.003,
            rotation_threshold=math.radians(1.0), minimum_episode_scans=8,
            stable_range_m=0.05, minimum_observation_fraction=0.60,
            resolution=0.05, laser_x=0.0, laser_y=0.0,
            laser_yaw=math.pi):
    connection = open_read_only(database)
    samples = []
    observed = 0
    without_coverage = 0
    try:
        timeline = PoseTimeline(pose_samples_from_connection(connection))
        cursor = connection.execute(
            "SELECT payload FROM messages WHERE topic='/scan_slam' "
            "ORDER BY sequence")
        for encoded, in cursor:
            observed += 1
            scan = json.loads(encoded)
            end = stamp_seconds(scan["header"])
            start = end - float(scan["scan_time"])
            if not timeline.covers(start, end):
                without_coverage += 1
                continue
            start_pose = timeline.interpolate(start)
            end_pose = timeline.interpolate(end)
            translation = math.hypot(
                end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
            rotation = abs(end_pose[2] - start_pose[2])
            if (translation <= translation_threshold
                    and rotation <= rotation_threshold):
                samples.append({
                    "stamp": end,
                    "pose": end_pose,
                    "scan": scan,
                    "translation": translation,
                    "rotation": rotation,
                })
    finally:
        connection.close()

    grouped = group_stationary_samples(
        samples, minimum_scans=minimum_episode_scans)
    public_episodes = []
    private_episodes = []
    for episode in grouped:
        public, private = summarize_episode(
            episode, minimum_observation_fraction, stable_range_m)
        public_episodes.append(public)
        private_episodes.append(private)

    revisit = closest_revisit(
        public_episodes, private_episodes,
        minimum_separation_s=10.0,
        maximum_translation_m=0.20,
        maximum_rotation_rad=math.radians(15.0),
        resolution=resolution,
        laser_x=laser_x, laser_y=laser_y, laser_yaw=laser_yaw)
    alignment = None
    if revisit is not None:
        alignment = optimize_revisit_alignment(
            private_episodes[revisit["first_episode"]],
            private_episodes[revisit["second_episode"]],
            resolution=resolution, laser_x=laser_x, laser_y=laser_y,
            laser_yaw=laser_yaw)
    stable_fractions = [episode["stable_bin_fraction"]
                        for episode in public_episodes]
    p95_mads = [episode["range_mad_m"]["p95"]
                for episode in public_episodes
                if episode["range_mad_m"]["p95"] is not None]
    within_repeatable = bool(stable_fractions) and (
        percentile(stable_fractions, 0.50) >= 0.85
        and percentile(p95_mads, 0.50) <= stable_range_m)
    revisit_repeatable = bool(revisit) and min(
        revisit["within_one_cell"]["first_fraction"],
        revisit["within_one_cell"]["second_fraction"]) >= 0.75
    aligned_revisit_repeatable = bool(alignment) and (
        alignment["within_one_cell"]["symmetric_minimum_fraction"] >= 0.80)
    if not within_repeatable:
        recommendation = "sensor_returns_or_stationary_scene"
    elif revisit_repeatable:
        recommendation = "translation_pose_and_mapper_observability"
    elif aligned_revisit_repeatable:
        recommendation = "inter_stop_pose_error_supported"
    else:
        recommendation = "extrinsic_environment_or_inter_stop_pose"

    report = {
        "version": 1,
        "database": str(Path(database).resolve()),
        "thresholds": {
            "translation_m_per_scan": translation_threshold,
            "rotation_deg_per_scan": math.degrees(rotation_threshold),
            "minimum_episode_scans": minimum_episode_scans,
            "minimum_observation_fraction": minimum_observation_fraction,
            "stable_range_m": stable_range_m,
            "endpoint_resolution_m": resolution,
        },
        "scan_counts": {
            "observed": observed,
            "without_pose_coverage": without_coverage,
            "stationary": len(samples),
            "stationary_in_episodes": sum(len(item) for item in grouped),
        },
        "episode_count": len(public_episodes),
        "episodes": public_episodes,
        "aggregate": {
            "median_stable_bin_fraction": percentile(stable_fractions, 0.50),
            "median_episode_p95_range_mad_m": percentile(p95_mads, 0.50),
            "within_episode_repeatable": within_repeatable,
            "revisit_repeatable": revisit_repeatable,
            "aligned_revisit_repeatable": aligned_revisit_repeatable,
            "recommended_next_branch": recommendation,
        },
        "closest_revisit": revisit,
        "bounded_revisit_alignment": alignment,
        "limitations": [
            "Uses resampled 360-bin scans rather than original raw acquisition order.",
            "Recorded odometry is used to select and align episodes; it is not ground truth.",
            "A repeatable return can still be geometrically biased by calibration.",
        ],
    }
    output = Path(output)
    if output.exists():
        raise RepeatabilityError("Output already exists: {}".format(output))
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--translation-threshold", type=float, default=0.003)
    parser.add_argument(
        "--rotation-threshold-degrees", type=float, default=1.0)
    parser.add_argument("--minimum-episode-scans", type=int, default=8)
    parser.add_argument("--stable-range", type=float, default=0.05)
    parser.add_argument("--minimum-observation-fraction", type=float, default=0.60)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--laser-x", type=float, default=0.0)
    parser.add_argument("--laser-y", type=float, default=0.0)
    parser.add_argument("--laser-yaw", type=float, default=math.pi)
    parsed = parser.parse_args(args)
    if (parsed.translation_threshold <= 0
            or parsed.rotation_threshold_degrees <= 0
            or parsed.minimum_episode_scans < 2
            or parsed.stable_range <= 0
            or not 0 < parsed.minimum_observation_fraction <= 1
            or parsed.resolution <= 0):
        parser.error("thresholds and resolution must be positive and fractions bounded")
    report = analyze(
        parsed.database, parsed.output,
        translation_threshold=parsed.translation_threshold,
        rotation_threshold=math.radians(parsed.rotation_threshold_degrees),
        minimum_episode_scans=parsed.minimum_episode_scans,
        stable_range_m=parsed.stable_range,
        minimum_observation_fraction=parsed.minimum_observation_fraction,
        resolution=parsed.resolution,
        laser_x=parsed.laser_x, laser_y=parsed.laser_y,
        laser_yaw=parsed.laser_yaw)
    print("STATIONARY_SCAN_REPEATABILITY " + json.dumps({
        "output": str(parsed.output.resolve()),
        "scan_counts": report["scan_counts"],
        "episode_count": report["episode_count"],
        "aggregate": report["aggregate"],
        "closest_revisit": report["closest_revisit"],
        "bounded_revisit_alignment": report["bounded_revisit_alignment"],
    }, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
