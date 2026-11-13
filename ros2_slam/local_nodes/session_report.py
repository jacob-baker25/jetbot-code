#!/usr/bin/env python3
"""Heading log, turn-by-turn report and graphs for one mapping session.

mapping_session.py writes heading_log.csv (10 rows/s) into the experiment
directory and calls write_report() when the stack stops. The report compares,
for every turn, how far each source says the robot turned:

  odom   the selected odometry yaw: integrated IMU yaw in imu_yaw modes,
         RF2O yaw in rf2o mode
  raw    raw RF2O (laser odometry) yaw
  slam   Slam Toolbox map->base_link yaw
  corr   Slam Toolbox map->odom yaw: how much SLAM had to correct odometry

Outputs, next to heading_log.csv:
  heading_report.txt   the summary and turn table (also printed)
  heading_plot.png     headings, turn rate and SLAM correction over time
  path_plot.png        odometry and SLAM paths from above

Gyro-integrated yaw is the source of fused heading, not independent truth;
compare against the physical floor reference recorded on paper.

Re-run on any session directory:
  python3 session_report.py ~/autochair_ros2_ws/experiments/<session>
  python3 session_report.py <session> --no-plots
"""

import argparse
import csv
import json
import math
import os
import re


HEADING_LOG_NAME = "heading_log.csv"
REPORT_NAME = "heading_report.txt"
HEADING_PLOT_NAME = "heading_plot.png"
PATH_PLOT_NAME = "path_plot.png"

HEADING_LOG_COLUMNS = [
    "time_s",
    "clock",
    "command",
    "checkpoint",
    "odom_yaw_deg",
    "odom_turn_rate_dps",
    "raw_rf2o_yaw_deg",
    "slam_yaw_deg",
    "slam_correction_deg",
    "odom_x_m",
    "odom_y_m",
    "slam_x_m",
    "slam_y_m",
]

TURN_RATE_DPS = 8.0         # odometry turning faster than this = in a turn
TURN_JOIN_S = 1.0           # key-repeat stutter shorter than this joins one turn
TURN_MIN_DEG = 10.0         # ignore wiggles smaller than this
STILL_STEP_DEG = 0.2        # odometry yaw change per row below this = settled
SETTLE_S = 2.0              # let SLAM catch up after a turn (scan delay + matching)
AGREE_DEG = 3.0             # provisional 90-degree turn target (IMU_AND_MAP_TEST_PLAN.md)
CLOSE_DEG = 5.0             # RECORDER_REPAIR_AND_TURN_TEST.md inspection threshold


# ============================================================
# WRITING (used live by mapping_session.py)
# ============================================================

class Unwrapper:
    """Keep a heading continuous past +/-180 degrees."""

    def __init__(self):
        self.last = None
        self.total = None

    def update(self, angle_deg):
        if angle_deg is None or not math.isfinite(angle_deg):
            return None
        if self.last is None:
            self.total = angle_deg
        else:
            self.total += (angle_deg - self.last + 180.0) % 360.0 - 180.0
        self.last = angle_deg
        return self.total


def fmt(value, digits):
    if value is None or not math.isfinite(value):
        return ""
    return "{:.{}f}".format(value, digits)


# ============================================================
# LOADING
# ============================================================

def _float(row, key):
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def load_heading_log(path):
    rows = []
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            t = _float(row, "time_s")
            if t is None:
                continue        # half-written last line after a forced stop
            item = {"t": t, "command": row.get("command", ""),
                    "checkpoint": row.get("checkpoint", "")}
            for key in HEADING_LOG_COLUMNS[4:]:
                item[key] = _float(row, key)
            rows.append(item)
    return rows


def load_odom_mode(experiment_dir):
    try:
        with open(os.path.join(experiment_dir, "results.json")) as stream:
            return json.load(stream).get("configuration", {}).get("odom_mode", "unknown")
    except (IOError, OSError, ValueError):
        return "unknown"


def load_bias_status(experiment_dir):
    """Last bias status the IMU publisher printed, if bias hold was on."""
    try:
        with open(os.path.join(experiment_dir, "imu.log"), errors="replace") as stream:
            text = stream.read()
    except (IOError, OSError):
        return None
    found = re.findall(r"bias_hold=on bias_dps=([-+0-9.]+) still_pct=([0-9.]+)", text)
    if not found:
        return None
    bias, still = found[-1]
    return {"bias_dps": float(bias), "still_pct": float(still)}


# ============================================================
# ANALYSIS
# ============================================================

def odom_rates(rows):
    """Odometry yaw rate per row from successive yaws (deg/s)."""
    rates = [0.0]
    for prev, cur in zip(rows, rows[1:]):
        dt = cur["t"] - prev["t"]
        if dt <= 0 or prev["odom_yaw_deg"] is None or cur["odom_yaw_deg"] is None:
            rates.append(0.0)
        else:
            rates.append((cur["odom_yaw_deg"] - prev["odom_yaw_deg"]) / dt)
    return rates


def _step(rows, i, j):
    a, b = rows[i]["odom_yaw_deg"], rows[j]["odom_yaw_deg"]
    return abs(b - a) if a is not None and b is not None else 0.0


def _delta(rows, key, a, b):
    va, vb = rows[a][key], rows[b][key]
    if va is None or vb is None:
        return None
    return vb - va


def find_turns(rows):
    """List of turns with each source's heading change over the same window."""
    rates = odom_rates(rows)
    spans, start, last_fast = [], None, None
    for i, row in enumerate(rows):
        if abs(rates[i]) >= TURN_RATE_DPS:
            if start is None:
                start = i
            last_fast = i
        elif start is not None and row["t"] - rows[last_fast]["t"] > TURN_JOIN_S:
            spans.append((start, last_fast))
            start = None
    if start is not None:
        spans.append((start, last_fast))

    turns = []
    for number, (a, b) in enumerate(spans):
        # Widen to where odometry yaw stopped changing.
        a0 = max(a - 1, 0)
        while a0 > 0 and _step(rows, a0 - 1, a0) > STILL_STEP_DEG:
            a0 -= 1
        b1 = min(b + 1, len(rows) - 1)
        while b1 < len(rows) - 1 and _step(rows, b1, b1 + 1) > STILL_STEP_DEG:
            b1 += 1
        # Let SLAM catch up, while the robot stays still and before the next turn.
        next_start = spans[number + 1][0] if number + 1 < len(spans) else len(rows)
        end = b1
        while (end + 1 < next_start and rows[end + 1]["t"] - rows[b1]["t"] <= SETTLE_S
               and rows[end + 1]["command"] in ("stop", "")
               and _step(rows, end, end + 1) <= STILL_STEP_DEG):
            end += 1
        odom = _delta(rows, "odom_yaw_deg", a0, end)
        if odom is None or abs(odom) < TURN_MIN_DEG:
            continue
        slam = _delta(rows, "slam_yaw_deg", a0, end)
        turns.append({
            "start_s": rows[a0]["t"],
            "duration_s": rows[b1]["t"] - rows[a0]["t"],
            "direction": "LEFT" if odom > 0 else "RIGHT",
            "odom_deg": odom,
            "raw_deg": _delta(rows, "raw_rf2o_yaw_deg", a0, end),
            "slam_deg": slam,
            "slam_minus_odom_deg": None if slam is None else slam - odom,
            "correction_deg": _delta(rows, "slam_correction_deg", a0, end),
            "peak_rate_dps": max(abs(r) for r in rates[a0:b1 + 1]),
            "start_index": a0,
            "end_index": end,
        })
    return turns


def stationary_drift(rows, turns):
    """Heading change per minute while commanded stopped and outside turns."""
    in_turn = set()
    for turn in turns:
        in_turn.update(range(turn["start_index"], turn["end_index"] + 1))
    totals = {"odom_yaw_deg": 0.0, "slam_yaw_deg": 0.0}
    still_s = 0.0
    for i in range(1, len(rows)):
        prev, cur = rows[i - 1], rows[i]
        if i in in_turn or (i - 1) in in_turn:
            continue
        if prev["command"] != "stop" or cur["command"] != "stop":
            continue
        dt = cur["t"] - prev["t"]
        if dt <= 0 or dt > 0.5:
            continue
        for key in totals:
            if prev[key] is not None and cur[key] is not None:
                totals[key] += cur[key] - prev[key]
        still_s += dt
    if still_s < 5.0:
        return None
    return {
        "still_s": still_s,
        "odom_deg_per_min": totals["odom_yaw_deg"] / still_s * 60.0,
        "slam_deg_per_min": totals["slam_yaw_deg"] / still_s * 60.0,
    }


def agreement(diff):
    if diff is None:
        return "no SLAM"
    if abs(diff) <= AGREE_DEG:
        return "agree"
    if abs(diff) <= CLOSE_DEG:
        return "close"
    return "DISAGREE"


# ============================================================
# REPORT
# ============================================================

def _cell(value, width=8):
    return ("{:+{}.1f}".format(value, width) if value is not None else " " * (width - 1) + "-")


def build_report(rows, turns, odom_mode="unknown", bias=None, session=""):
    odom_label = "IMU yaw" if odom_mode.startswith("imu_yaw") else "RF2O"
    lines = []
    lines.append("=== Heading report: {} ===".format(session))
    lines.append("Odometry mode: {}  (odom column = {})".format(odom_mode, odom_label))
    duration = rows[-1]["t"] - rows[0]["t"] if len(rows) > 1 else 0.0
    lines.append("Rows: {}  over {:.1f} s".format(len(rows), duration))
    if bias is not None:
        lines.append("IMU bias hold: learned bias {:+.4f} deg/s, robot still {:.0f}% of samples"
                     .format(bias["bias_dps"], bias["still_pct"]))
    drift = stationary_drift(rows, turns)
    if drift:
        lines.append("Parked drift ({:.0f} s stopped): odom {:+.2f} deg/min, SLAM {:+.2f} deg/min"
                     .format(drift["still_s"], drift["odom_deg_per_min"], drift["slam_deg_per_min"]))
    else:
        lines.append("Parked drift: not measured (less than 5 s stopped outside turns)")
    lines.append("")
    lines.append("Turns: {}   (degrees, + = left / counterclockwise)".format(len(turns)))
    if turns:
        lines.append("  #  start_s  dur_s  dir      odom   rawRF2O      SLAM  SLAM-odom  SLAMcorr  verdict")
        for number, turn in enumerate(turns, 1):
            lines.append("  {:<2} {:7.1f} {:6.1f}  {:<5} {} {} {} {} {}   {}".format(
                number, turn["start_s"], turn["duration_s"], turn["direction"],
                _cell(turn["odom_deg"]), _cell(turn["raw_deg"], 9), _cell(turn["slam_deg"], 9),
                _cell(turn["slam_minus_odom_deg"], 10), _cell(turn["correction_deg"], 9),
                agreement(turn["slam_minus_odom_deg"])))
        lines.append("")
        lines.append("verdict compares SLAM with odometry for the same window: agree <= {:.0f} deg,"
                     " close <= {:.0f} deg.".format(AGREE_DEG, CLOSE_DEG))
        lines.append("SLAMcorr is how much SLAM rotated map->odom during the turn: near 0 means SLAM")
        lines.append("accepted the odometry turn; large means scan matching overrode it.")
        lines.append("Neither source is ground truth; compare with the physical floor reference.")
    return "\n".join(lines) + "\n"


# ============================================================
# GRAPHS
# ============================================================

def write_plots(experiment_dir, rows, turns, odom_mode="unknown"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return [], "matplotlib not installed (sudo apt-get install python3-matplotlib); no graphs"

    odom_label = "odometry (IMU yaw)" if odom_mode.startswith("imu_yaw") else "odometry (RF2O)"
    t = [row["t"] for row in rows]

    def series(key):
        return [row[key] if row[key] is not None else float("nan") for row in rows]

    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, series("odom_yaw_deg"), label=odom_label, color="#1F3A5F", linewidth=1.6)
    axes[0].plot(t, series("raw_rf2o_yaw_deg"), label="raw RF2O", color="#999999", linewidth=1.0)
    axes[0].plot(t, series("slam_yaw_deg"), label="SLAM", color="#C0392B", linewidth=1.2, linestyle="--")
    for number, turn in enumerate(turns, 1):
        end_t = rows[turn["end_index"]]["t"]
        axes[0].axvspan(turn["start_s"], end_t, color="#C98A1B", alpha=0.15)
        axes[0].annotate("{}: {:+.0f}".format(number, turn["odom_deg"]),
                         ((turn["start_s"] + end_t) / 2.0, 1.0), xycoords=("data", "axes fraction"),
                         ha="center", va="top", fontsize=8)
    axes[0].set_ylabel("Heading (deg, + = left)")
    axes[0].legend(loc="best", fontsize=8)

    axes[1].plot(t, odom_rates(rows), color="#2E7D5B", linewidth=1.0)
    for i, row in enumerate(rows[:-1]):
        if row["command"] in ("left", "right"):
            axes[1].axvspan(row["t"], rows[i + 1]["t"], linewidth=0,
                            color="#3A7BD5" if row["command"] == "left" else "#D5763A", alpha=0.12)
    axes[1].set_ylabel("Odometry turn rate (deg/s)\nshaded: A/D pressed")

    axes[2].plot(t, series("slam_correction_deg"), color="#7D3C98", linewidth=1.2)
    axes[2].set_ylabel("SLAM correction\nmap->odom yaw (deg)")
    axes[2].set_xlabel("Time since session start (s)")
    for axis in axes:
        axis.grid(alpha=0.3)
    figure.suptitle("{}  ({})".format(os.path.basename(os.path.normpath(experiment_dir)), odom_mode))
    figure.tight_layout()
    heading_path = os.path.join(experiment_dir, HEADING_PLOT_NAME)
    figure.savefig(heading_path, dpi=110)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 8))
    axis.plot(series("odom_x_m"), series("odom_y_m"), label=odom_label, color="#1F3A5F")
    axis.plot(series("slam_x_m"), series("slam_y_m"), label="SLAM", color="#C0392B", linestyle="--")
    marked = set()
    for row in rows:
        label = row["checkpoint"]
        if label and label not in marked and row["slam_x_m"] is not None:
            marked.add(label)
            axis.plot(row["slam_x_m"], row["slam_y_m"], "o", color="#C0392B", markersize=4)
            axis.annotate(label, (row["slam_x_m"], row["slam_y_m"]), fontsize=7,
                          xytext=(4, 4), textcoords="offset points")
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(alpha=0.3)
    axis.legend(loc="best", fontsize=8)
    axis.set_title("Path from above (dots: SLAM pose at each checkpoint)")
    path_path = os.path.join(experiment_dir, PATH_PLOT_NAME)
    figure.savefig(path_path, dpi=110)
    plt.close(figure)
    return [heading_path, path_path], None


def write_report(experiment_dir, plots=True, echo=True):
    """Write heading_report.txt (+ graphs). Returns the list of files written."""
    log_path = os.path.join(experiment_dir, HEADING_LOG_NAME)
    if not os.path.isfile(log_path):
        return []
    rows = load_heading_log(log_path)
    if len(rows) < 2:
        return []
    odom_mode = load_odom_mode(experiment_dir)
    turns = find_turns(rows)
    text = build_report(rows, turns, odom_mode, load_bias_status(experiment_dir),
                        os.path.basename(os.path.normpath(experiment_dir)))
    report_path = os.path.join(experiment_dir, REPORT_NAME)
    with open(report_path, "w") as stream:
        stream.write(text)
    written = [report_path]
    if echo:
        print(text)
    if plots:
        paths, problem = write_plots(experiment_dir, rows, turns, odom_mode)
        written.extend(paths)
        if problem and echo:
            print(problem)
    return written


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("experiment_dir")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    written = write_report(args.experiment_dir, plots=not args.no_plots)
    if not written:
        raise SystemExit("No usable {} in {}".format(HEADING_LOG_NAME, args.experiment_dir))
    for path in written:
        print("Wrote " + path)


if __name__ == "__main__":
    main()
