#!/usr/bin/env python3
"""Pure planar IMU integration and RF2O translation composition."""

import bisect
import math


class ImuTimingError(RuntimeError):
    pass


class StreamTimingError(RuntimeError):
    pass


def wrap_radians(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


class YawIntegrator:
    def __init__(self, max_gap_s=0.10, history_s=15.0):
        self.max_gap_s = max_gap_s
        self.history_s = history_s
        self.times = []
        self.rates = []
        self.angles = []

    def add(self, timestamp_s, yaw_rate_rad_s):
        if not math.isfinite(timestamp_s) or not math.isfinite(yaw_rate_rad_s):
            raise ImuTimingError("nonfinite IMU timestamp or yaw rate")
        if self.times:
            dt = timestamp_s - self.times[-1]
            if dt <= 0.0:
                raise ImuTimingError("IMU timestamp did not increase")
            if dt > self.max_gap_s:
                raise ImuTimingError(
                    "IMU gap {:.3f}s exceeds {:.3f}s".format(dt, self.max_gap_s)
                )
            angle = self.angles[-1] + 0.5 * (
                self.rates[-1] + yaw_rate_rad_s
            ) * dt
        else:
            angle = 0.0
        self.times.append(timestamp_s)
        self.rates.append(yaw_rate_rad_s)
        self.angles.append(angle)
        cutoff = timestamp_s - self.history_s
        remove = bisect.bisect_left(self.times, cutoff)
        if remove:
            del self.times[:remove]
            del self.rates[:remove]
            del self.angles[:remove]

    def interpolate(self, timestamp_s):
        if len(self.times) < 2:
            return None
        if timestamp_s < self.times[0] or timestamp_s > self.times[-1]:
            return None
        index = bisect.bisect_right(self.times, timestamp_s) - 1
        if index >= len(self.times) - 1:
            return self.angles[-1], self.rates[-1]
        ta, tb = self.times[index], self.times[index + 1]
        fraction = (timestamp_s - ta) / (tb - ta)
        angle = self.angles[index] + fraction * (
            self.angles[index + 1] - self.angles[index]
        )
        rate = self.rates[index] + fraction * (
            self.rates[index + 1] - self.rates[index]
        )
        return angle, rate


class PlanarYawFusion:
    def __init__(self):
        self.initialized = False
        self.raw_x = None
        self.raw_y = None
        self.raw_yaw = None
        self.fused_x = None
        self.fused_y = None
        self.fused_yaw = None
        self.yaw_offset = None

    def update(self, raw_x, raw_y, raw_yaw, imu_integrated_yaw):
        values = (raw_x, raw_y, raw_yaw, imu_integrated_yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("nonfinite odometry input")
        if not self.initialized:
            self.raw_x = raw_x
            self.raw_y = raw_y
            self.raw_yaw = raw_yaw
            self.fused_x = raw_x
            self.fused_y = raw_y
            self.yaw_offset = raw_yaw - imu_integrated_yaw
            self.fused_yaw = raw_yaw
            self.initialized = True
            return self.fused_x, self.fused_y, wrap_radians(self.fused_yaw)

        # Recover RF2O's relative translation in the previous robot frame,
        # then compose that same increment using the IMU-derived orientation.
        dx = raw_x - self.raw_x
        dy = raw_y - self.raw_y
        cosine_raw = math.cos(self.raw_yaw)
        sine_raw = math.sin(self.raw_yaw)
        body_x = cosine_raw * dx + sine_raw * dy
        body_y = -sine_raw * dx + cosine_raw * dy
        cosine_fused = math.cos(self.fused_yaw)
        sine_fused = math.sin(self.fused_yaw)
        self.fused_x += cosine_fused * body_x - sine_fused * body_y
        self.fused_y += sine_fused * body_x + cosine_fused * body_y
        self.fused_yaw = self.yaw_offset + imu_integrated_yaw
        self.raw_x = raw_x
        self.raw_y = raw_y
        self.raw_yaw = raw_yaw
        return self.fused_x, self.fused_y, wrap_radians(self.fused_yaw)


class StartupAwareWatchdog:
    """Bound startup separately from strict steady-state stream health."""

    def __init__(self, start_s, stable_samples=10, startup_timeout_s=20.0,
                 stale_timeout_s=1.0):
        self.start_s = start_s
        self.stable_samples = stable_samples
        self.startup_timeout_s = startup_timeout_s
        self.stale_timeout_s = stale_timeout_s
        self.samples = 0
        self.last_sample_s = None

    @property
    def ready(self):
        return self.samples >= self.stable_samples

    def observe(self, now_s):
        self.samples += 1
        self.last_sample_s = now_s

    def check(self, now_s):
        if self.ready:
            if now_s - self.last_sample_s > self.stale_timeout_s:
                raise StreamTimingError("raw odometry became stale")
        elif now_s - self.start_s > self.startup_timeout_s:
            raise StreamTimingError(
                "raw odometry did not stabilize: {}/{} fused samples".format(
                    self.samples, self.stable_samples
                )
            )
