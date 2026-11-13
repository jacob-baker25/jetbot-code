#!/usr/bin/env python3
"""Stationary gyro bias hold for the planar Phidget yaw rate (opt-in).

zeroGyro() removes most bias at startup, but a small residual (and warm-up
change) remains. Integrated over a session it becomes heading drift that
reaches SLAM through odom->base_link while the robot is parked.

When the yaw rate shows the robot is still, this publishes exactly zero yaw
rate (holding the integrated heading) and learns the residual bias from the
one-second mean. While moving, it subtracts the learned bias. Stillness is
judged from the gyro alone, because LiDAR motor vibration makes the
accelerometer an unreliable stillness signal on this robot.

Units are the raw sensor deg/s, before the mounting sign/rad conversion.
This is a zero-velocity model assumption, not a second sensor measurement;
it is only enabled by the imu_yaw_hold odometry mode so it can be compared
against plain imu_yaw.
"""

from collections import deque


STILL_WINDOW_S = 1.0        # still must hold for this long before holding
STILL_MEAN_DPS = 0.5        # |mean rate| over the window (turns are tens of deg/s)
STILL_SPREAD_DPS = 1.0      # every sample within this of the window mean
STILL_SAMPLE_DPS = 1.0      # the current sample itself (after bias) must be this small
BIAS_SMOOTHING = 0.02       # per still sample; about 1 s to settle at 50 Hz
MAX_BIAS_DPS = 1.0          # never learn more than this; larger means not still


class StationaryBiasHold:

    def __init__(self, window_s=STILL_WINDOW_S):
        self.window_s = window_s
        self.window = deque()   # (device time s, raw rate deg/s)
        self.bias_dps = 0.0
        self.still = False
        self.samples = 0
        self.still_samples = 0

    def _window_still(self, rate_dps):
        if abs(rate_dps - self.bias_dps) > STILL_SAMPLE_DPS:
            return False
        if len(self.window) < 3:
            return False
        span = self.window[-1][0] - self.window[0][0]
        if span < 0.9 * self.window_s:
            return False
        rates = [rate for _, rate in self.window]
        mean = sum(rates) / len(rates)
        if abs(mean) > STILL_MEAN_DPS or abs(mean) > MAX_BIAS_DPS:
            return False
        if max(abs(rate - mean) for rate in rates) > STILL_SPREAD_DPS:
            return False
        return True

    def update(self, timestamp_s, rate_dps):
        """Return the rate to publish, in raw sensor deg/s."""
        self.window.append((timestamp_s, rate_dps))
        while self.window and timestamp_s - self.window[0][0] > self.window_s:
            self.window.popleft()
        self.samples += 1
        self.still = self._window_still(rate_dps)
        if self.still:
            self.still_samples += 1
            mean = sum(rate for _, rate in self.window) / len(self.window)
            self.bias_dps += BIAS_SMOOTHING * (mean - self.bias_dps)
            self.bias_dps = max(-MAX_BIAS_DPS, min(MAX_BIAS_DPS, self.bias_dps))
            return 0.0
        return rate_dps - self.bias_dps

    def still_fraction(self):
        return self.still_samples / self.samples if self.samples else 0.0
