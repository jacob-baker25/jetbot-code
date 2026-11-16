"""Narrow adapter for the inspected Roboticia RPLidar driver.

Use PySerial's configured read timeout rather than polling inWaiting forever.
Protocol decoding and the vendor's overflow limit/recovery remain unchanged.
"""
from collections import deque
import math
import time

from rplidar import RPLidar as VendorRPLidar, RPLidarException


class BoundedRPLidar(VendorRPLidar):
    def __init__(self, *args, **kwargs):
        self.read_calls = 0
        self.read_timeouts = 0
        self.max_read_s = 0.0
        self.recovery_count = 0
        self.recoveries = deque(maxlen=32)
        self.capture_active = False
        self.pending_recovery = None
        super(BoundedRPLidar, self).__init__(*args, **kwargs)

    def _read_response(self, dsize):
        timeout = self._serial.timeout
        if timeout is None or not math.isfinite(timeout) or timeout <= 0:
            raise RPLidarException("Bounded reader requires a positive finite serial timeout")
        started = time.monotonic()
        self.read_calls += 1
        try:
            data = self._serial.read(dsize)
        finally:
            self.max_read_s = max(self.max_read_s, time.monotonic() - started)
        if len(data) != dsize:
            self.read_timeouts += 1
            raise RPLidarException(
                "Serial response incomplete: expected {} bytes, got {} within {}s".format(
                    dsize, len(data), timeout))
        return data

    def stop(self):
        if self.capture_active and self.scanning[0]:
            self.recovery_count += 1
            self.pending_recovery = (time.monotonic(), time.time())
        return super(BoundedRPLidar, self).stop()

    def start(self, scan_type='normal'):
        success = False
        try:
            result = super(BoundedRPLidar, self).start(scan_type)
            success = bool(self.scanning[0])
            return result
        finally:
            if self.pending_recovery is not None:
                mono, unix = self.pending_recovery
                self.recoveries.append({"start_mono_s": mono, "start_unix_s": unix,
                    "duration_s": time.monotonic() - mono, "success": success})
                self.pending_recovery = None

    def timing_summary(self):
        return {"reader": "blocking_serial_read", "read_calls": self.read_calls,
                "read_timeouts": self.read_timeouts, "max_read_s": self.max_read_s,
                "recovery_count": self.recovery_count, "recoveries": list(self.recoveries)}
