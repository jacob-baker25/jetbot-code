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


class TimestampSynchronizer:
    """Hold slower messages until a faster timestamped stream brackets them."""

    def __init__(self, max_pending=100):
        self.max_pending = max_pending
        self.pending = []
        self.last_timestamp_s = None

    def add(self, timestamp_s, receipt_s, value):
        if not math.isfinite(timestamp_s) or not math.isfinite(receipt_s):
            raise StreamTimingError("nonfinite queued-message timestamp")
        if (self.last_timestamp_s is not None and
                timestamp_s <= self.last_timestamp_s):
            raise StreamTimingError("raw odometry timestamp did not increase")
        if len(self.pending) >= self.max_pending:
            raise StreamTimingError(
                "raw odometry synchronization queue overflow ({})".format(
                    self.max_pending
                )
            )
        self.pending.append((timestamp_s, receipt_s, value))
        self.last_timestamp_s = timestamp_s

    def drain(self, oldest_reference_s, newest_reference_s):
        """Return (ready, too_old), retaining messages newer than reference."""
        ready = []
        too_old = []
        while self.pending and self.pending[0][0] < oldest_reference_s:
            too_old.append(self.pending.pop(0))
        while self.pending and self.pending[0][0] <= newest_reference_s:
            ready.append(self.pending.pop(0))
        return ready, too_old

    def oldest_wait_s(self, now_s):
        if not self.pending:
            return None
        return now_s - self.pending[0][1]


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
    """Require a continuous startup window before arming a strict stale gate."""

    def __init__(self, start_s, stable_samples=10, startup_timeout_s=20.0,
                 stale_timeout_s=1.0, stable_duration_s=3.0):
        self.start_s = start_s
        self.stable_samples = stable_samples
        self.startup_timeout_s = startup_timeout_s
        self.stale_timeout_s = stale_timeout_s
        self.stable_duration_s = stable_duration_s
        self.total_samples = 0
        self.consecutive_samples = 0
        self.stable_start_s = None
        self.last_sample_s = None
        self.last_gap_s = None
        self._ready = False

    @property
    def ready(self):
        return self._ready

    def stable_elapsed_s(self, now_s=None):
        if self.stable_start_s is None:
            return 0.0
        end_s = self.last_sample_s if now_s is None else now_s
        return max(0.0, end_s - self.stable_start_s)

    def observe(self, now_s):
        reset = False
        if not self.ready:
            if (self.last_sample_s is None or
                    now_s - self.last_sample_s > self.stale_timeout_s):
                if self.last_sample_s is not None:
                    self.last_gap_s = now_s - self.last_sample_s
                    reset = True
                self.stable_start_s = now_s
                self.consecutive_samples = 0
            self.consecutive_samples += 1
        self.total_samples += 1
        self.last_sample_s = now_s
        if (not self.ready and
                self.consecutive_samples >= self.stable_samples and
                self.stable_elapsed_s() >= self.stable_duration_s):
            self._ready = True
        return reset

    def check(self, now_s):
        if self.ready:
            if now_s - self.last_sample_s > self.stale_timeout_s:
                raise StreamTimingError("raw odometry became stale")
        elif now_s - self.start_s > self.startup_timeout_s:
            raise StreamTimingError(
                (
                    "raw odometry did not stabilize: total={} consecutive={}/{} "
                    "continuous={:.3f}/{:.3f}s"
                ).format(
                    self.total_samples, self.consecutive_samples,
                    self.stable_samples, self.stable_elapsed_s(),
                    self.stable_duration_s
                )
            )
