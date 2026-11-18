#!/usr/bin/env python3

"""Pure helpers for converting RPLIDAR measurements to ROS LaserScan bins."""

import math
import bisect


NUM_BINS = 360
RANGE_MIN_M = 0.15
RANGE_MAX_M = 12.0
CARTESIAN_MAX_GAP_DEG = 3.0
CARTESIAN_MAX_CHORD_M = 0.35


def ros_bin_for_hardware_angle(angle_deg, num_bins=NUM_BINS):
    """Map the RPLIDAR's clockwise-positive angle to ROS CCW-positive bins.

    Preserve the legacy bridge's integer-degree binning before reversing the
    direction.  This isolates the handedness repair from any resampling change.
    """

    hardware_bin = int(angle_deg) % num_bins
    return (-hardware_bin) % num_bins


def laser_scan_angles(num_bins=NUM_BINS):
    """Return scan metadata compatible with this legacy Karto build.

    Karto computes its expected count as (angle_max-angle_min)/increment,
    without adding the inclusive endpoint. It therefore requires 2*pi here
    for 360 one-degree values even though the final array index is 359. RF2O
    separately uses the actual array length when constructing its geometry.
    """

    increment = (2.0 * math.pi) / num_bins
    return 0.0, num_bins * increment, increment


def resample_cartesian_chords(raw, num_bins=NUM_BINS):
    """Resample fractional beams at bin centers using short measured chords.

    Only two same-scan, range-valid returns may support an output. Their
    angular gap must be at most three degrees and their Cartesian endpoints
    at most 0.35 m apart. The target ray must intersect the segment itself;
    there is no extrapolation, temporal state or alteration of raw input.
    """
    by_angle = {}
    for quality, angle_deg, distance_mm in raw:
        distance_m = distance_mm / 1000.0
        if RANGE_MIN_M <= distance_m <= RANGE_MAX_M:
            by_angle[float(angle_deg) % 360.0] = (
                distance_m, float(quality))

    angles = sorted(by_angle)
    ranges = [float('inf')] * num_bins
    intensities = [0.0] * num_bins
    if len(angles) < 2:
        return ranges, intensities

    bin_width_deg = 360.0 / num_bins
    for hardware_bin in range(num_bins):
        target = (hardware_bin + 0.5) * bin_width_deg
        right_index = bisect.bisect_left(angles, target)
        if right_index < len(angles) and angles[right_index] == target:
            distance, intensity = by_angle[target]
        else:
            left_index = (right_index - 1) % len(angles)
            wrapped_right = right_index % len(angles)
            left_original = angles[left_index]
            right_original = angles[wrapped_right]
            left_angle = left_original
            right_angle = right_original
            adjusted_target = target
            if right_index == 0:
                adjusted_target += 360.0
            if wrapped_right == 0:
                right_angle += 360.0
            gap = right_angle - left_angle
            if gap <= 0.0 or gap > CARTESIAN_MAX_GAP_DEG:
                continue

            left_range, left_quality = by_angle[left_original]
            right_range, right_quality = by_angle[right_original]
            left_rad = math.radians(left_angle)
            right_rad = math.radians(right_angle)
            target_rad = math.radians(adjusted_target)
            left_point = (left_range * math.cos(left_rad),
                          left_range * math.sin(left_rad))
            right_point = (right_range * math.cos(right_rad),
                           right_range * math.sin(right_rad))
            segment = (right_point[0] - left_point[0],
                       right_point[1] - left_point[1])
            if math.hypot(segment[0], segment[1]) > CARTESIAN_MAX_CHORD_M:
                continue

            direction = (math.cos(target_rad), math.sin(target_rad))
            denominator = (direction[0] * segment[1] -
                           direction[1] * segment[0])
            if abs(denominator) < 1e-12:
                continue
            distance = (left_point[0] * segment[1] -
                        left_point[1] * segment[0]) / denominator
            fraction = (left_point[0] * direction[1] -
                        left_point[1] * direction[0]) / denominator
            if (distance <= 0.0 or fraction < -1e-9 or
                    fraction > 1.0 + 1e-9):
                continue
            intensity = min(left_quality, right_quality)

        ros_bin = (-hardware_bin) % num_bins
        ranges[ros_bin] = distance
        intensities[ros_bin] = intensity
    return ranges, intensities
