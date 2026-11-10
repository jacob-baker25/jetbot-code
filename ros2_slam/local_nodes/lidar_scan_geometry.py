#!/usr/bin/env python3

"""Pure helpers for converting RPLIDAR measurements to ROS LaserScan bins."""

import math


NUM_BINS = 360


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
