#!/usr/bin/env python3
"""Deterministic planar deskew for captured JetBot LaserScan payloads.

This module deliberately has no ROS imports so its geometry can be tested and
audited offline.  Captured scan headers are end-of-revolution stamps from the
current bridge.  The bridge converts clockwise hardware bins to ROS bins with
``ros_bin = (-hardware_bin) % count``; this module reverses that mapping when
assigning per-beam acquisition times.
"""
import bisect
import math


NANOSECONDS = 1000000000


class DeskewError(RuntimeError):
    pass


def stamp_seconds(header):
    return int(header["sec"]) + int(header["nanosec"]) / float(NANOSECONDS)


def quaternion_yaw(value):
    return math.atan2(
        2.0 * (float(value["w"]) * float(value["z"])
               + float(value["x"]) * float(value["y"])),
        1.0 - 2.0 * (float(value["y"]) ** 2
                     + float(value["z"]) ** 2))


class PoseTimeline(object):
    """Strictly ordered planar poses with linear SE(2) interpolation."""

    def __init__(self, samples):
        if len(samples) < 2:
            raise DeskewError("Deskew requires at least two odometry samples")
        ordered = sorted(
            (float(stamp), float(x), float(y), float(yaw))
            for stamp, x, y, yaw in samples)
        self.times = [sample[0] for sample in ordered]
        if any(current <= previous
               for previous, current in zip(self.times, self.times[1:])):
            raise DeskewError("Odometry stamps must increase for deskew")
        self.xs = [sample[1] for sample in ordered]
        self.ys = [sample[2] for sample in ordered]
        self.yaws = [ordered[0][3]]
        previous_raw = ordered[0][3]
        for unused_stamp, unused_x, unused_y, current_raw in ordered[1:]:
            delta = math.atan2(
                math.sin(current_raw - previous_raw),
                math.cos(current_raw - previous_raw))
            self.yaws.append(self.yaws[-1] + delta)
            previous_raw = current_raw

    def covers(self, start, end):
        return self.times[0] <= start <= end <= self.times[-1]

    def interpolate(self, stamp):
        if stamp < self.times[0] or stamp > self.times[-1]:
            raise DeskewError("Odometry does not cover requested stamp")
        upper = bisect.bisect_left(self.times, stamp)
        if upper == 0:
            return self.xs[0], self.ys[0], self.yaws[0]
        if upper == len(self.times):
            return self.xs[-1], self.ys[-1], self.yaws[-1]
        if self.times[upper] == stamp:
            return self.xs[upper], self.ys[upper], self.yaws[upper]
        lower = upper - 1
        ratio = ((stamp - self.times[lower])
                 / (self.times[upper] - self.times[lower]))
        return (
            self.xs[lower] + ratio * (self.xs[upper] - self.xs[lower]),
            self.ys[lower] + ratio * (self.ys[upper] - self.ys[lower]),
            self.yaws[lower] + ratio * (self.yaws[upper] - self.yaws[lower]),
        )


def pose_samples_from_connection(connection):
    import json
    samples = []
    for encoded, in connection.execute(
            "SELECT payload FROM messages WHERE topic='/odom_rf2o' "
            "ORDER BY sequence"):
        payload = json.loads(encoded)
        samples.append((
            stamp_seconds(payload["header"]),
            float(payload["position"]["x"]),
            float(payload["position"]["y"]),
            quaternion_yaw(payload["orientation"]),
        ))
    return samples


def _rotate(x, y, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def acquisition_rank(ros_index, count):
    """Return clockwise hardware acquisition rank for a ROS CCW bin."""
    return (-int(ros_index)) % int(count)


class ScanDeskewer(object):
    """Deskew scan payloads to their scan-end laser pose."""

    def __init__(self, timeline, laser_x=0.0, laser_y=0.0,
                 laser_yaw=math.pi):
        self.timeline = timeline
        self.laser_x = float(laser_x)
        self.laser_y = float(laser_y)
        self.laser_yaw = float(laser_yaw)
        self.scans_observed = 0
        self.scans_corrected = 0
        self.scans_without_coverage = 0
        self.input_points = 0
        self.output_points = 0
        self.rebin_collisions = 0
        self.maximum_scan_translation = 0.0
        self.maximum_scan_rotation = 0.0
        self.maximum_point_correction = 0.0

    def deskew(self, scan):
        self.scans_observed += 1
        ranges = scan.get("ranges", [])
        count = len(ranges)
        if count < 2:
            raise DeskewError("Laser scan must contain at least two bins")
        scan_time = float(scan["scan_time"])
        if not math.isfinite(scan_time) or scan_time <= 0:
            raise DeskewError("Laser scan_time must be finite and positive")
        end = stamp_seconds(scan["header"])
        start = end - scan_time
        if not self.timeline.covers(start, end):
            self.scans_without_coverage += 1
            return scan

        start_pose = self.timeline.interpolate(start)
        end_pose = self.timeline.interpolate(end)
        scan_translation = math.hypot(
            end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        scan_rotation = abs(end_pose[2] - start_pose[2])
        self.maximum_scan_translation = max(
            self.maximum_scan_translation, scan_translation)
        self.maximum_scan_rotation = max(
            self.maximum_scan_rotation, scan_rotation)

        output_ranges = [None] * count
        input_intensities = list(scan.get("intensities", []))
        output_intensities = [None] * count if input_intensities else []
        angle_min = float(scan["angle_min"])
        angle_increment = float(scan["angle_increment"])
        if not math.isfinite(angle_increment) or angle_increment <= 0:
            raise DeskewError("Laser angle_increment must be finite and positive")
        period = angle_increment * count

        # Reference laser pose in odometry coordinates at scan end.
        offset_x, offset_y = _rotate(
            self.laser_x, self.laser_y, end_pose[2])
        reference_x = end_pose[0] + offset_x
        reference_y = end_pose[1] + offset_y
        reference_yaw = end_pose[2] + self.laser_yaw

        for index, raw_range in enumerate(ranges):
            if raw_range is None:
                continue
            distance = float(raw_range)
            if not math.isfinite(distance):
                continue
            self.input_points += 1

            rank = acquisition_rank(index, count)
            beam_stamp = start + (rank / float(count)) * scan_time
            pose_x, pose_y, pose_yaw = self.timeline.interpolate(beam_stamp)

            laser_offset_x, laser_offset_y = _rotate(
                self.laser_x, self.laser_y, pose_yaw)
            laser_world_x = pose_x + laser_offset_x
            laser_world_y = pose_y + laser_offset_y
            beam_angle = angle_min + index * angle_increment
            point_laser_x = distance * math.cos(beam_angle)
            point_laser_y = distance * math.sin(beam_angle)
            point_world_dx, point_world_dy = _rotate(
                point_laser_x, point_laser_y,
                pose_yaw + self.laser_yaw)
            point_world_x = laser_world_x + point_world_dx
            point_world_y = laser_world_y + point_world_dy

            reference_dx = point_world_x - reference_x
            reference_dy = point_world_y - reference_y
            corrected_x, corrected_y = _rotate(
                reference_dx, reference_dy, -reference_yaw)
            corrected_range = math.hypot(corrected_x, corrected_y)
            corrected_angle = math.atan2(corrected_y, corrected_x)
            relative = (corrected_angle - angle_min) % period
            corrected_index = int(round(relative / angle_increment)) % count
            correction = math.hypot(
                corrected_x - point_laser_x,
                corrected_y - point_laser_y)
            self.maximum_point_correction = max(
                self.maximum_point_correction, correction)

            previous = output_ranges[corrected_index]
            if previous is not None:
                self.rebin_collisions += 1
                if previous <= corrected_range:
                    continue
            output_ranges[corrected_index] = corrected_range
            if output_intensities:
                output_intensities[corrected_index] = (
                    input_intensities[index]
                    if index < len(input_intensities) else 0.0)

        result = dict(scan)
        result["ranges"] = output_ranges
        if output_intensities:
            result["intensities"] = output_intensities
        result["time_increment"] = 0.0
        self.scans_corrected += 1
        self.output_points += sum(
            1 for value in output_ranges if value is not None)
        return result

    def summary(self):
        return {
            "reference_pose": "scan_end",
            "acquisition_order": "clockwise_reversed_ros_bins",
            "laser_x": self.laser_x,
            "laser_y": self.laser_y,
            "laser_yaw": self.laser_yaw,
            "scans_observed": self.scans_observed,
            "scans_corrected": self.scans_corrected,
            "scans_without_coverage": self.scans_without_coverage,
            "input_points": self.input_points,
            "output_points": self.output_points,
            "rebin_collisions": self.rebin_collisions,
            "maximum_scan_translation_m": self.maximum_scan_translation,
            "maximum_scan_rotation_rad": self.maximum_scan_rotation,
            "maximum_scan_rotation_deg": math.degrees(
                self.maximum_scan_rotation),
            "maximum_point_correction_m": self.maximum_point_correction,
        }
