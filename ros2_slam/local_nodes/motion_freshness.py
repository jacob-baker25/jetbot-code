"""Freshness check for supervised mapping motion, not a real-time safety layer."""
import math

MAX_MOTION_AGE_S = 0.35


def require_fresh_motion(odom, scan, ros_now_s, mono_now_s):
    for name, sample in (("odometry", odom), ("scan", scan)):
        if sample is None:
            raise RuntimeError("MOTION_DATA_STALE missing " + name)
        stamp_age = ros_now_s - sample["stamp_s"]
        receipt_age = mono_now_s - sample["receipt_mono"]
        if (not all(math.isfinite(v) for v in (stamp_age, receipt_age)) or
                not -.05 <= stamp_age <= MAX_MOTION_AGE_S or
                not 0 <= receipt_age <= MAX_MOTION_AGE_S):
            raise RuntimeError(
                "MOTION_DATA_STALE {} stamp_age={:.3f}s receipt_age={:.3f}s".format(
                    name, stamp_age, receipt_age))
