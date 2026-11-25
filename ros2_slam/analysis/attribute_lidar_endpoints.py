#!/usr/bin/env python3
"""Attribute recorded LiDAR endpoints to route passes and robot motion.

The diagnostic does not run SLAM and never changes the capture.  It projects
both the recorded scans (as the current mapper saw them) and deskewed scans into
the recorded odometry frame.  Corrected endpoints are then grouped by outbound,
turnaround, and return pass as well as by motion during each revolution.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sqlite3
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "local_nodes"))

from lidar_deskew import (  # noqa: E402
    PoseTimeline, ScanDeskewer, pose_samples_from_connection, stamp_seconds)


PASS_NAMES = ("prelude", "outbound", "turnaround", "return", "postlude")
MOTION_NAMES = (
    "stationary", "translating", "turning", "turning_translating")
PASS_COLORS = {
    "prelude": (150, 150, 150),
    "outbound": (0, 150, 230),
    "turnaround": (245, 145, 20),
    "return": (220, 45, 150),
    "postlude": (90, 90, 90),
}
MOTION_COLORS = {
    "stationary": (45, 105, 210),
    "translating": (30, 165, 80),
    "turning": (220, 50, 45),
    "turning_translating": (240, 145, 20),
}


class AttributionError(RuntimeError):
    pass


def open_read_only(path):
    path = Path(path).resolve()
    if not path.is_file():
        raise AttributionError("Capture database does not exist: {}".format(path))
    return sqlite3.connect(path.as_uri() + "?mode=ro", uri=True)


def wrapped_angle_delta(first, second):
    return math.atan2(math.sin(second - first), math.cos(second - first))


def checkpoint_pose(checkpoint):
    value = checkpoint.get("rf2o") or checkpoint.get("slam")
    if value is None:
        raise AttributionError("Checkpoint has no RF2O or SLAM pose")
    return float(value["x"]), float(value["y"]), math.radians(
        float(value["yaw_deg"]))


def load_route(results_json):
    try:
        value = json.loads(Path(results_json).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise AttributionError("Cannot read checkpoint results: {}".format(exc))
    checkpoints = value.get("checkpoints", [])
    if len(checkpoints) < 3:
        raise AttributionError("At least three checkpoints are required")
    times = [float(item["time_unix"]) for item in checkpoints]
    if any(current <= previous
           for previous, current in zip(times, times[1:])):
        raise AttributionError("Checkpoint times must increase")

    start_x, start_y, unused_yaw = checkpoint_pose(checkpoints[0])
    # The closed out-and-back route reaches its most distant checkpoint just
    # before the turnaround.  Exclude the final point from this inference.
    distances = []
    for index, checkpoint in enumerate(checkpoints[:-1]):
        x, y, unused_heading = checkpoint_pose(checkpoint)
        distances.append((math.hypot(x - start_x, y - start_y), index))
    turnaround_start = max(distances)[1]
    if turnaround_start == 0:
        raise AttributionError("Could not infer a non-start turnaround")
    turnaround_end = turnaround_start
    if turnaround_start + 1 < len(checkpoints):
        first = checkpoint_pose(checkpoints[turnaround_start])
        second = checkpoint_pose(checkpoints[turnaround_start + 1])
        separation = math.hypot(second[0] - first[0], second[1] - first[1])
        rotation = abs(wrapped_angle_delta(first[2], second[2]))
        if separation <= 0.25 and rotation >= math.radians(45.0):
            turnaround_end += 1
    return {
        "experiment_name": value.get("experiment_name"),
        "checkpoints": checkpoints,
        "start_time": times[0],
        "turnaround_start_time": times[turnaround_start],
        "turnaround_end_time": times[turnaround_end],
        "final_time": times[-1],
        "turnaround_start_index": turnaround_start,
        "turnaround_end_index": turnaround_end,
        "turnaround_start_label": checkpoints[turnaround_start].get("label"),
        "turnaround_end_label": checkpoints[turnaround_end].get("label"),
    }


def pass_for_stamp(stamp, route):
    if stamp < route["start_time"]:
        return "prelude"
    if stamp <= route["turnaround_start_time"]:
        return "outbound"
    if stamp <= route["turnaround_end_time"]:
        return "turnaround"
    if stamp <= route["final_time"]:
        return "return"
    return "postlude"


def motion_for_delta(translation, rotation, translation_threshold,
                     rotation_threshold):
    translating = translation >= translation_threshold
    turning = rotation >= rotation_threshold
    if translating and turning:
        return "turning_translating"
    if turning:
        return "turning"
    if translating:
        return "translating"
    return "stationary"


def scan_gate_decision(previous_pose, current_pose, distance_threshold,
                       heading_threshold):
    """Reproduce the guarded euclidean-heading scan insertion gate."""
    if previous_pose is None:
        return True, "initial"
    distance_squared = (
        (current_pose[0] - previous_pose[0]) ** 2
        + (current_pose[1] - previous_pose[1]) ** 2)
    heading = abs(wrapped_angle_delta(previous_pose[2], current_pose[2]))
    # The runtime reports 0.249999 for a configured 0.5 m threshold.
    if distance_squared >= distance_threshold ** 2 - 1e-6:
        return True, "distance"
    if heading >= heading_threshold:
        return True, "heading"
    return False, "below_threshold"


def rotate(x, y, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def world_endpoints(scan, pose, laser_x, laser_y, laser_yaw,
                    maximum_range):
    offset_x, offset_y = rotate(laser_x, laser_y, pose[2])
    origin_x = pose[0] + offset_x
    origin_y = pose[1] + offset_y
    world_yaw = pose[2] + laser_yaw
    angle_min = float(scan["angle_min"])
    angle_increment = float(scan["angle_increment"])
    minimum_range = float(scan.get("range_min", 0.0))
    points = []
    for index, raw_distance in enumerate(scan.get("ranges", [])):
        if raw_distance is None:
            continue
        distance = float(raw_distance)
        if (not math.isfinite(distance) or distance < minimum_range
                or distance > maximum_range):
            continue
        angle = world_yaw + angle_min + index * angle_increment
        points.append((
            origin_x + distance * math.cos(angle),
            origin_y + distance * math.sin(angle),
        ))
    return points


def cell_for_point(point, resolution):
    return (int(math.floor(point[0] / resolution)),
            int(math.floor(point[1] / resolution)))


def increment_cell(cells, cell, category):
    if cell not in cells:
        cells[cell] = {}
    cells[cell][category] = cells[cell].get(category, 0) + 1


def active_cells(cells, category, minimum_hits):
    return {cell for cell, counts in cells.items()
            if counts.get(category, 0) >= minimum_hits}


def neighborhood_overlap(first, second, radius=1):
    if not first:
        return 0, 0.0
    matched = 0
    for x, y in first:
        if any((x + dx, y + dy) in second
               for dx in range(-radius, radius + 1)
               for dy in range(-radius, radius + 1)):
            matched += 1
    return matched, matched / float(len(first))


def image_bounds(*cell_sets):
    combined = set()
    for values in cell_sets:
        combined.update(values)
    if not combined:
        raise AttributionError("No endpoints remained after filtering")
    xs = [cell[0] for cell in combined]
    ys = [cell[1] for cell in combined]
    return min(xs) - 2, max(xs) + 2, min(ys) - 2, max(ys) + 2


def category_color(counts, names, colors, minimum_hits):
    active = [name for name in names if counts.get(name, 0) >= minimum_hits]
    if not active:
        return None
    if len(active) > 1:
        # Black is deliberately reserved for cells repeatedly observed by
        # multiple passes/classes.
        return (0, 0, 0)
    return colors[active[0]]


def write_ppm(path, cells, names, colors, minimum_hits, bounds):
    minimum_x, maximum_x, minimum_y, maximum_y = bounds
    width = maximum_x - minimum_x + 1
    height = maximum_y - minimum_y + 1
    pixels = bytearray([255] * (width * height * 3))
    for (cell_x, cell_y), counts in cells.items():
        color = category_color(counts, names, colors, minimum_hits)
        if color is None:
            continue
        x = cell_x - minimum_x
        y = maximum_y - cell_y
        offset = (y * width + x) * 3
        pixels[offset:offset + 3] = bytes(color)
    with Path(path).open("wb") as stream:
        stream.write("P6\n{} {}\n255\n".format(width, height).encode("ascii"))
        stream.write(pixels)


def write_density_pgm(path, counts, bounds, minimum_hits):
    minimum_x, maximum_x, minimum_y, maximum_y = bounds
    width = maximum_x - minimum_x + 1
    height = maximum_y - minimum_y + 1
    pixels = bytearray([255] * (width * height))
    for (cell_x, cell_y), count in counts.items():
        if count < minimum_hits:
            continue
        x = cell_x - minimum_x
        y = maximum_y - cell_y
        # Log scaling keeps single walls visible without letting repeatedly
        # sampled stationary cells dominate the image.
        value = max(0, 210 - int(42.0 * math.log(count, 2)))
        pixels[y * width + x] = value
    with Path(path).open("wb") as stream:
        stream.write("P5\n{} {}\n255\n".format(width, height).encode("ascii"))
        stream.write(pixels)


def write_cell_csv(path, cells, raw_counts, corrected_counts, resolution):
    names = list(PASS_NAMES) + list(MOTION_NAMES)
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["cell_x", "cell_y", "x_m", "y_m", "raw_hits",
             "corrected_hits"] + names)
        for cell in sorted(cells):
            counts = cells[cell]
            writer.writerow([
                cell[0], cell[1],
                (cell[0] + 0.5) * resolution,
                (cell[1] + 0.5) * resolution,
                raw_counts.get(cell, 0), corrected_counts.get(cell, 0),
            ] + [counts.get(name, 0) for name in names])


def analyze(database, results_json, output, resolution=0.05,
            maximum_range=12.0, minimum_hits=2,
            translation_threshold=0.003,
            rotation_threshold=math.radians(1.0),
            gate_distance=0.5, gate_heading=0.5,
            keyframe_minimum_hits=1,
            laser_x=0.0, laser_y=0.0, laser_yaw=math.pi):
    output = Path(output).resolve()
    if output.exists():
        raise AttributionError("Output already exists: {}".format(output))
    if (resolution <= 0 or maximum_range <= 0 or minimum_hits <= 0
            or gate_distance <= 0 or gate_heading <= 0
            or keyframe_minimum_hits <= 0):
        raise AttributionError("Resolution, ranges, gates, and hits must be positive")
    route = load_route(results_json)
    connection = open_read_only(database)
    output.mkdir(parents=True)
    try:
        timeline = PoseTimeline(pose_samples_from_connection(connection))
        deskewer = ScanDeskewer(
            timeline, laser_x=laser_x, laser_y=laser_y,
            laser_yaw=laser_yaw)
        pass_cells = {}
        motion_cells = {}
        raw_counts = {}
        corrected_counts = {}
        keyframe_pass_cells = {}
        keyframe_motion_cells = {}
        keyframe_counts = {}
        scan_counts = {name: 0 for name in PASS_NAMES + MOTION_NAMES}
        point_counts = {name: 0 for name in PASS_NAMES + MOTION_NAMES}
        keyframe_scan_counts = {
            name: 0 for name in PASS_NAMES + MOTION_NAMES}
        keyframe_point_counts = {
            name: 0 for name in PASS_NAMES + MOTION_NAMES}
        gate_reasons = {
            "initial": 0, "distance": 0, "heading": 0,
            "below_threshold": 0,
        }
        previous_keyframe_pose = None
        scans_without_end_pose = 0

        cursor = connection.execute(
            "SELECT payload FROM messages WHERE topic='/scan_slam' "
            "ORDER BY sequence")
        for encoded, in cursor:
            scan = json.loads(encoded)
            end = stamp_seconds(scan["header"])
            start = end - float(scan["scan_time"])
            if not timeline.covers(end, end):
                scans_without_end_pose += 1
                continue
            end_pose = timeline.interpolate(end)
            if timeline.covers(start, end):
                start_pose = timeline.interpolate(start)
                translation = math.hypot(
                    end_pose[0] - start_pose[0],
                    end_pose[1] - start_pose[1])
                rotation = abs(end_pose[2] - start_pose[2])
            else:
                translation = 0.0
                rotation = 0.0
            pass_name = pass_for_stamp(end, route)
            motion_name = motion_for_delta(
                translation, rotation, translation_threshold,
                rotation_threshold)
            scan_counts[pass_name] += 1
            scan_counts[motion_name] += 1

            raw_points = world_endpoints(
                scan, end_pose, laser_x, laser_y, laser_yaw, maximum_range)
            corrected_scan = deskewer.deskew(scan)
            corrected_points = world_endpoints(
                corrected_scan, end_pose, laser_x, laser_y, laser_yaw,
                maximum_range)
            point_counts[pass_name] += len(corrected_points)
            point_counts[motion_name] += len(corrected_points)

            for point in raw_points:
                cell = cell_for_point(point, resolution)
                raw_counts[cell] = raw_counts.get(cell, 0) + 1
            for point in corrected_points:
                cell = cell_for_point(point, resolution)
                corrected_counts[cell] = corrected_counts.get(cell, 0) + 1
                increment_cell(pass_cells, cell, pass_name)
                increment_cell(motion_cells, cell, motion_name)

            accepted, gate_reason = scan_gate_decision(
                previous_keyframe_pose, end_pose, gate_distance,
                gate_heading)
            gate_reasons[gate_reason] += 1
            if accepted:
                previous_keyframe_pose = end_pose
                keyframe_scan_counts[pass_name] += 1
                keyframe_scan_counts[motion_name] += 1
                keyframe_point_counts[pass_name] += len(corrected_points)
                keyframe_point_counts[motion_name] += len(corrected_points)
                for point in corrected_points:
                    cell = cell_for_point(point, resolution)
                    keyframe_counts[cell] = keyframe_counts.get(cell, 0) + 1
                    increment_cell(keyframe_pass_cells, cell, pass_name)
                    increment_cell(keyframe_motion_cells, cell, motion_name)

        all_cells = set(raw_counts) | set(corrected_counts)
        bounds = image_bounds(all_cells)
        write_ppm(
            output / "endpoints_by_pass.ppm", pass_cells, PASS_NAMES,
            PASS_COLORS, minimum_hits, bounds)
        write_ppm(
            output / "endpoints_by_motion.ppm", motion_cells, MOTION_NAMES,
            MOTION_COLORS, minimum_hits, bounds)
        write_density_pgm(
            output / "raw_endpoint_density.pgm", raw_counts, bounds,
            minimum_hits)
        write_density_pgm(
            output / "deskewed_endpoint_density.pgm", corrected_counts,
            bounds, minimum_hits)
        write_ppm(
            output / "keyframes_by_pass.ppm", keyframe_pass_cells,
            PASS_NAMES, PASS_COLORS, keyframe_minimum_hits, bounds)
        write_ppm(
            output / "keyframes_by_motion.ppm", keyframe_motion_cells,
            MOTION_NAMES, MOTION_COLORS, keyframe_minimum_hits, bounds)
        write_density_pgm(
            output / "keyframe_endpoint_density.pgm", keyframe_counts,
            bounds, keyframe_minimum_hits)
        combined_cells = {}
        for cell in set(pass_cells) | set(motion_cells) | set(raw_counts):
            combined_cells[cell] = {}
            combined_cells[cell].update(pass_cells.get(cell, {}))
            combined_cells[cell].update(motion_cells.get(cell, {}))
        write_cell_csv(
            output / "cell_attribution.csv", combined_cells, raw_counts,
            corrected_counts, resolution)
        keyframe_combined_cells = {}
        for cell in set(keyframe_pass_cells) | set(keyframe_motion_cells):
            keyframe_combined_cells[cell] = {}
            keyframe_combined_cells[cell].update(
                keyframe_pass_cells.get(cell, {}))
            keyframe_combined_cells[cell].update(
                keyframe_motion_cells.get(cell, {}))
        write_cell_csv(
            output / "keyframe_cell_attribution.csv",
            keyframe_combined_cells, {}, keyframe_counts, resolution)

        outbound = active_cells(pass_cells, "outbound", minimum_hits)
        returning = active_cells(pass_cells, "return", minimum_hits)
        exact = outbound & returning
        outbound_near_count, outbound_near_fraction = neighborhood_overlap(
            outbound, returning, radius=1)
        return_near_count, return_near_fraction = neighborhood_overlap(
            returning, outbound, radius=1)
        raw_active = {cell for cell, count in raw_counts.items()
                      if count >= minimum_hits}
        corrected_active = {cell for cell, count in corrected_counts.items()
                            if count >= minimum_hits}
        raw_corrected_exact = raw_active & corrected_active
        keyframe_outbound = active_cells(
            keyframe_pass_cells, "outbound", keyframe_minimum_hits)
        keyframe_return = active_cells(
            keyframe_pass_cells, "return", keyframe_minimum_hits)
        keyframe_exact = keyframe_outbound & keyframe_return
        keyframe_outbound_near_count, keyframe_outbound_near_fraction = (
            neighborhood_overlap(keyframe_outbound, keyframe_return, radius=1))
        keyframe_return_near_count, keyframe_return_near_fraction = (
            neighborhood_overlap(keyframe_return, keyframe_outbound, radius=1))
        manifest = {
            "version": 1,
            "database": str(Path(database).resolve()),
            "results_json": str(Path(results_json).resolve()),
            "output": str(output),
            "resolution_m": resolution,
            "maximum_range_m": maximum_range,
            "minimum_hits_per_cell": minimum_hits,
            "keyframe_minimum_hits_per_cell": keyframe_minimum_hits,
            "translation_threshold_m_per_scan": translation_threshold,
            "rotation_threshold_deg_per_scan": math.degrees(
                rotation_threshold),
            "laser_transform": {
                "x": laser_x, "y": laser_y, "yaw": laser_yaw,
            },
            "route": {key: value for key, value in route.items()
                      if key != "checkpoints"},
            "scan_counts": scan_counts,
            "corrected_point_counts": point_counts,
            "keyframe_scan_counts": keyframe_scan_counts,
            "keyframe_corrected_point_counts": keyframe_point_counts,
            "scan_gate": {
                "distance_threshold_m": gate_distance,
                "heading_threshold_rad": gate_heading,
                "heading_threshold_deg": math.degrees(gate_heading),
                "accepted_scans": (
                    gate_reasons["initial"] + gate_reasons["distance"]
                    + gate_reasons["heading"]),
                "reasons": gate_reasons,
            },
            "active_cell_counts_by_pass": {
                name: len(active_cells(pass_cells, name, minimum_hits))
                for name in PASS_NAMES
            },
            "active_cell_counts_by_motion": {
                name: len(active_cells(motion_cells, name, minimum_hits))
                for name in MOTION_NAMES
            },
            "scans_without_end_pose": scans_without_end_pose,
            "deskew": deskewer.summary(),
            "grid": {
                "minimum_cell_x": bounds[0],
                "maximum_cell_x": bounds[1],
                "minimum_cell_y": bounds[2],
                "maximum_cell_y": bounds[3],
                "width_cells": bounds[1] - bounds[0] + 1,
                "height_cells": bounds[3] - bounds[2] + 1,
            },
            "pass_overlap": {
                "outbound_cells": len(outbound),
                "return_cells": len(returning),
                "exact_shared_cells": len(exact),
                "exact_outbound_fraction": (
                    len(exact) / float(len(outbound)) if outbound else 0.0),
                "exact_return_fraction": (
                    len(exact) / float(len(returning)) if returning else 0.0),
                "within_one_cell_outbound_count": outbound_near_count,
                "within_one_cell_outbound_fraction": outbound_near_fraction,
                "within_one_cell_return_count": return_near_count,
                "within_one_cell_return_fraction": return_near_fraction,
            },
            "raw_deskew_cell_overlap": {
                "raw_cells": len(raw_active),
                "deskewed_cells": len(corrected_active),
                "exact_shared_cells": len(raw_corrected_exact),
                "raw_shared_fraction": (
                    len(raw_corrected_exact) / float(len(raw_active))
                    if raw_active else 0.0),
                "deskewed_shared_fraction": (
                    len(raw_corrected_exact) / float(len(corrected_active))
                    if corrected_active else 0.0),
            },
            "keyframe_pass_overlap": {
                "outbound_cells": len(keyframe_outbound),
                "return_cells": len(keyframe_return),
                "exact_shared_cells": len(keyframe_exact),
                "exact_outbound_fraction": (
                    len(keyframe_exact) / float(len(keyframe_outbound))
                    if keyframe_outbound else 0.0),
                "exact_return_fraction": (
                    len(keyframe_exact) / float(len(keyframe_return))
                    if keyframe_return else 0.0),
                "within_one_cell_outbound_count": (
                    keyframe_outbound_near_count),
                "within_one_cell_outbound_fraction": (
                    keyframe_outbound_near_fraction),
                "within_one_cell_return_count": keyframe_return_near_count,
                "within_one_cell_return_fraction": keyframe_return_near_fraction,
            },
            "image_legend": {
                "pass_colors_rgb": PASS_COLORS,
                "motion_colors_rgb": MOTION_COLORS,
                "black": "cell observed by multiple active categories",
                "white": "no category reached the minimum hit count",
            },
        }
        (output / "attribution_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False)
            + "\n", encoding="utf-8")
        return manifest
    finally:
        connection.close()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--results-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--maximum-range", type=float, default=12.0)
    parser.add_argument("--minimum-hits", type=int, default=2)
    parser.add_argument("--translation-threshold", type=float, default=0.003)
    parser.add_argument(
        "--rotation-threshold-degrees", type=float, default=1.0)
    parser.add_argument("--gate-distance", type=float, default=0.5)
    parser.add_argument("--gate-heading", type=float, default=0.5)
    parser.add_argument("--keyframe-minimum-hits", type=int, default=1)
    parser.add_argument("--laser-x", type=float, default=0.0)
    parser.add_argument("--laser-y", type=float, default=0.0)
    parser.add_argument("--laser-yaw", type=float, default=math.pi)
    parsed = parser.parse_args(args)
    if parsed.rotation_threshold_degrees <= 0:
        parser.error("rotation threshold must be positive")
    manifest = analyze(
        parsed.database, parsed.results_json, parsed.output,
        resolution=parsed.resolution,
        maximum_range=parsed.maximum_range,
        minimum_hits=parsed.minimum_hits,
        translation_threshold=parsed.translation_threshold,
        rotation_threshold=math.radians(
            parsed.rotation_threshold_degrees),
        gate_distance=parsed.gate_distance,
        gate_heading=parsed.gate_heading,
        keyframe_minimum_hits=parsed.keyframe_minimum_hits,
        laser_x=parsed.laser_x, laser_y=parsed.laser_y,
        laser_yaw=parsed.laser_yaw)
    print("LIDAR_ENDPOINT_ATTRIBUTION " + json.dumps({
        "output": manifest["output"],
        "turnaround_start_label": manifest["route"][
            "turnaround_start_label"],
        "turnaround_end_label": manifest["route"][
            "turnaround_end_label"],
        "pass_overlap": manifest["pass_overlap"],
        "keyframe_pass_overlap": manifest["keyframe_pass_overlap"],
        "scan_gate": manifest["scan_gate"],
        "scan_counts": manifest["scan_counts"],
        "keyframe_scan_counts": manifest["keyframe_scan_counts"],
    }, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
