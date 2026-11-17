"""Narrow adapter for the inspected Roboticia RPLidar driver.

Use PySerial's configured read timeout rather than polling inWaiting forever.
Protocol decoding and the vendor's overflow limit/recovery remain unchanged.
"""
from collections import deque
import math
import time

from rplidar import RPLidar as VendorRPLidar, RPLidarException
import rplidar as vendor_driver


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


def check_batched_driver():
    """Check the inspected normal-packet contract without opening a serial port."""
    decode = getattr(vendor_driver, '_process_scan', None)
    normal = getattr(vendor_driver, '_SCAN_TYPE', {}).get('normal', {})
    if not callable(decode) or normal.get('size') != 5 or normal.get('byte') != b'\x20':
        raise RPLidarException('Unsupported vendor normal-scan interface')
    # Known wire packets: quality 15, 90 degrees, 1000 mm, start/not-start.
    for first, expected in ((61, True), (62, False)):
        if decode(bytes([first, 1, 45, 160, 15])) != (expected, 15, 90.0, 1000.0):
            raise RPLidarException('Vendor normal-scan decoder contract mismatch')
    for name in ('start_motor', 'start', 'stop'):
        if not callable(getattr(VendorRPLidar, name, None)):
            raise RPLidarException('Unsupported vendor method: '+name)


class BatchedRPLidar(BoundedRPLidar):
    """Opt-in normal-mode batching; retain vendor decoding and overflow recovery.

    At most 200 bytes are held outside the serial buffer. A read fills one batch
    (40 packets) rather than doing a syscall/timeout check for each five bytes.
    No unbounded queue, background reader, packet resynchronization or restamp.
    """
    BATCH_BYTES = 200
    MAX_BUFFER_BYTES = 3000

    def __init__(self, *args, **kwargs):
        check_batched_driver()
        self.batch_reads = 0
        self.decoded_packets = 0
        self.buffer_checks = 0
        self.max_pending_bytes = 0
        self.max_batch_read_s = 0.0
        self.max_batch_cycle_s = 0.0
        self.overflow_events = deque(maxlen=32)
        self.discarded_partial_scans = 0
        self.discarded_batch_bytes = 0
        self.recovery_generation = 0
        super(BatchedRPLidar, self).__init__(*args, **kwargs)

    def _recover_backlog(self, prefetched_bytes=0):
        queued = self._serial.inWaiting() + prefetched_bytes
        self.buffer_checks += 1
        self.max_pending_bytes = max(self.max_pending_bytes, queued)
        if queued <= self.MAX_BUFFER_BYTES:
            return False
        self.overflow_events.append(dict(pending_bytes=queued,
            mono_s=time.monotonic(), unix_s=time.time()))
        self.recovery_generation += 1
        self.stop()
        self.start('normal')
        return True

    def iter_measures(self, scan_type='normal', max_buf_meas=3000):
        if scan_type != 'normal' or max_buf_meas != self.MAX_BUFFER_BYTES:
            raise RPLidarException('Batched reader requires normal mode and 3000-byte limit')
        self.start_motor()
        if not self.scanning[0]:
            self.start('normal')
        previous_batch = None
        while True:
            if self.scanning != [True, 5, 'normal']:
                raise RPLidarException('Unexpected scan descriptor in batched reader')
            now = time.monotonic()
            if previous_batch is not None:
                self.max_batch_cycle_s = max(self.max_batch_cycle_s, now-previous_batch)
            previous_batch = now
            # The previous batch is exhausted here; there is no hidden backlog.
            self._recover_backlog()
            started = time.monotonic()
            data = self._read_response(self.BATCH_BYTES)
            self.max_batch_read_s = max(self.max_batch_read_s, time.monotonic()-started)
            self.batch_reads += 1
            for offset in range(0, len(data), 5):
                packet = vendor_driver._process_scan(data[offset:offset+5])
                self.decoded_packets += 1
                # A scan boundary can resume the ROS consumer. Recheck first so
                # data left in a batch cannot bypass overflow handling after a
                # consumer pause. Count software bytes as well as serial bytes.
                if packet[0] and self._recover_backlog(len(data)-offset):
                    self.discarded_batch_bytes += len(data)-offset
                    break
                yield packet

    def iter_scans(self, scan_type='normal', max_buf_meas=3000, min_len=5):
        scan = []
        started = False
        generation = self.recovery_generation
        for new_scan, quality, angle, distance in self.iter_measures(scan_type, max_buf_meas):
            if generation != self.recovery_generation:
                if scan:
                    self.discarded_partial_scans += 1
                scan = []
                started = False
                generation = self.recovery_generation
            if new_scan:
                if started and len(scan) > min_len:
                    yield scan
                scan = []
                started = True
            if started and distance > 0:
                scan.append((quality, angle, distance))

    def timing_summary(self):
        result = super(BatchedRPLidar, self).timing_summary()
        result.update(reader='batched_normal_v1', batch_bytes=self.BATCH_BYTES,
            max_buffer_bytes=self.MAX_BUFFER_BYTES, batch_reads=self.batch_reads,
            decoded_packets=self.decoded_packets, buffer_checks=self.buffer_checks,
            max_pending_bytes=self.max_pending_bytes, max_batch_read_s=self.max_batch_read_s,
            max_batch_cycle_s=self.max_batch_cycle_s,
            overflow_events=list(self.overflow_events),
            discarded_partial_scans=self.discarded_partial_scans,
            discarded_batch_bytes=self.discarded_batch_bytes)
        return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Read-only driver contract check; no serial port opened')
    parser.add_argument('--check-batched-driver', action='store_true', required=True)
    parser.parse_args()
    check_batched_driver()
    print('Batched lidar decoder/interface check: PASS (no hardware opened)')
