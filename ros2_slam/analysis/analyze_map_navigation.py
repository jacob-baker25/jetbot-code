#!/usr/bin/env python3
"""Measure whether a ROS occupancy map preserves clearance and connectivity.

This intentionally scores navigation properties, not visual attractiveness. It
uses only the Python 3.6 standard library so it can run on the Jetson or host.
"""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path


def read_map_yaml(path):
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    required = ("image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh")
    missing = [key for key in required if key not in values]
    if missing:
        raise ValueError("Map YAML missing: " + ", ".join(missing))
    origin_text = values["origin"].strip()
    if not (origin_text.startswith("[") and origin_text.endswith("]")):
        raise ValueError("Map origin must be a three-value list")
    origin = [float(value.strip()) for value in origin_text[1:-1].split(",")]
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise ValueError("Map origin must contain three finite values")
    image = Path(values["image"].strip("'\""))
    if not image.is_absolute():
        image = path.parent / image
    result = {
        "image": image,
        "resolution": float(values["resolution"]),
        "origin": origin,
        "negate": int(values["negate"]),
        "occupied_thresh": float(values["occupied_thresh"]),
        "free_thresh": float(values["free_thresh"]),
    }
    if result["resolution"] <= 0 or result["negate"] not in (0, 1):
        raise ValueError("Invalid map resolution or negate value")
    return result


def pgm_tokens(stream):
    token = bytearray()
    while True:
        byte = stream.read(1)
        if not byte:
            if token:
                yield bytes(token)
            return
        if byte == b"#" and not token:
            stream.readline()
            continue
        if byte.isspace():
            if token:
                yield bytes(token)
                token = bytearray()
            continue
        token.extend(byte)


def read_pgm(path):
    with path.open("rb") as stream:
        tokens = pgm_tokens(stream)
        magic = next(tokens)
        width, height, maximum = (int(next(tokens)) for unused in range(3))
        if width <= 0 or height <= 0 or not 0 < maximum <= 255:
            raise ValueError("Unsupported PGM dimensions or maximum value")
        if magic == b"P5":
            # Tokenization has consumed the delimiter after maxval, so the
            # stream now points at the first raster byte.
            pixels = stream.read(width * height)
            if len(pixels) != width * height:
                raise ValueError("Truncated PGM raster")
            return width, height, pixels, maximum
        if magic == b"P2":
            values = bytes(int(next(tokens)) for unused in range(width * height))
            return width, height, values, maximum
        raise ValueError("Only P2 and P5 PGM maps are supported")


class OccupancyMap(object):
    FREE = 0
    UNKNOWN = 1
    OCCUPIED = 2

    def __init__(self, yaml_path):
        self.config = read_map_yaml(yaml_path)
        self.width, self.height, pixels, maximum = read_pgm(self.config["image"])
        self.cells = []
        for value in pixels:
            normalized = float(value) / float(maximum)
            probability = normalized if self.config["negate"] else 1.0 - normalized
            if probability > self.config["occupied_thresh"]:
                self.cells.append(self.OCCUPIED)
            elif probability < self.config["free_thresh"]:
                self.cells.append(self.FREE)
            else:
                self.cells.append(self.UNKNOWN)

    def index(self, column, row):
        return row * self.width + column

    def in_bounds(self, cell):
        column, row = cell
        return 0 <= column < self.width and 0 <= row < self.height

    def world_to_cell(self, x_value, y_value):
        origin_x, origin_y, yaw = self.config["origin"]
        dx, dy = x_value - origin_x, y_value - origin_y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        column = int(math.floor(local_x / self.config["resolution"]))
        grid_y = int(math.floor(local_y / self.config["resolution"]))
        return column, self.height - 1 - grid_y


def disk_offsets(radius_cells):
    result = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                result.append((dx, dy))
    return result


def inflated_blocked(grid, clearance_m, unknown_blocked=True):
    radius = int(math.ceil(clearance_m / grid.config["resolution"]))
    blocked = bytearray(grid.width * grid.height)
    offsets = disk_offsets(radius)
    sources = []
    for row in range(grid.height):
        for column in range(grid.width):
            state = grid.cells[grid.index(column, row)]
            if state == grid.OCCUPIED or (unknown_blocked and state == grid.UNKNOWN):
                sources.append((column, row))
    for column, row in sources:
        for dx, dy in offsets:
            target = (column + dx, row + dy)
            if grid.in_bounds(target):
                blocked[grid.index(*target)] = 1
    return blocked, radius


def reachable_cells(grid, blocked, start):
    if not grid.in_bounds(start) or blocked[grid.index(*start)]:
        return set()
    seen = {start}
    queue = deque([start])
    while queue:
        column, row = queue.popleft()
        for target in ((column - 1, row), (column + 1, row),
                       (column, row - 1), (column, row + 1)):
            if (target not in seen and grid.in_bounds(target) and
                    not blocked[grid.index(*target)]):
                seen.add(target)
                queue.append(target)
    return seen


def read_trajectory(path, grid):
    cells = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                point = grid.world_to_cell(float(row["slam_x_m"]), float(row["slam_y_m"]))
            except (KeyError, ValueError):
                continue
            if grid.in_bounds(point) and (not cells or point != cells[-1]):
                cells.append(point)
    return cells


def parse_point(value):
    try:
        x_value, y_value = (float(part.strip()) for part in value.split(",", 1))
    except (ValueError, TypeError):
        raise argparse.ArgumentTypeError("Use X,Y in map-frame meters")
    if not math.isfinite(x_value) or not math.isfinite(y_value):
        raise argparse.ArgumentTypeError("Coordinates must be finite")
    return x_value, y_value


def analyze(yaml_path, robot_width, safety_margin, trajectory=None,
            start_world=None, goals_world=None):
    grid = OccupancyMap(yaml_path)
    clearance = robot_width / 2.0 + safety_margin
    blocked, radius_cells = inflated_blocked(grid, clearance, unknown_blocked=True)
    counts = {
        "free": grid.cells.count(grid.FREE),
        "unknown": grid.cells.count(grid.UNKNOWN),
        "occupied": grid.cells.count(grid.OCCUPIED),
    }
    trajectory_cells = read_trajectory(trajectory, grid) if trajectory else []
    if start_world is not None:
        start = grid.world_to_cell(*start_world)
    elif trajectory_cells:
        start = trajectory_cells[0]
    else:
        start = None
    reachable = reachable_cells(grid, blocked, start) if start is not None else set()
    goal_rows = []
    for point in goals_world or []:
        cell = grid.world_to_cell(*point)
        goal_rows.append({
            "world": list(point),
            "cell": list(cell),
            "in_bounds": grid.in_bounds(cell),
            "reachable": cell in reachable,
        })
    trajectory_blocked = sum(
        bool(blocked[grid.index(*cell)]) for cell in trajectory_cells)
    return {
        "map_yaml": str(yaml_path),
        "image": str(grid.config["image"]),
        "width_cells": grid.width,
        "height_cells": grid.height,
        "resolution_m": grid.config["resolution"],
        "cell_counts": counts,
        "robot_width_m": robot_width,
        "safety_margin_m": safety_margin,
        "required_center_clearance_m": clearance,
        "inflation_radius_cells": radius_cells,
        "unknown_treated_as_blocked": True,
        "start_cell": list(start) if start is not None else None,
        "reachable_cell_count": len(reachable),
        "goals": goal_rows,
        "trajectory": {
            "unique_cells": len(trajectory_cells),
            "blocked_cells": trajectory_blocked,
            "blocked_fraction": (
                float(trajectory_blocked) / len(trajectory_cells)
                if trajectory_cells else None),
        },
        "interpretation": (
            "Preliminary navigation geometry check. A pass still requires measured openings, "
            "held-out localization, a Nav2 plan, and supervised execution."
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--robot-width", type=float, required=True)
    parser.add_argument("--safety-margin", type=float, required=True,
                        help="Additional clearance on each side of the robot")
    parser.add_argument("--trajectory", type=Path)
    parser.add_argument("--start", type=parse_point)
    parser.add_argument("--goal", type=parse_point, action="append", default=[])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    for label, value in (("robot width", args.robot_width),
                         ("safety margin", args.safety_margin)):
        if not math.isfinite(value) or value < 0:
            parser.error(label + " must be finite and nonnegative")
    if args.robot_width == 0:
        parser.error("robot width must be greater than zero")
    report = analyze(args.map_yaml, args.robot_width, args.safety_margin,
                     args.trajectory, args.start, args.goal)
    output = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    else:
        print(output, end="")


if __name__ == "__main__":
    main()
