#!/usr/bin/env python3
"""Capture Phidget22 Spatial callbacks without ROS, fusion or motor control.

Preserves vendor units and device timestamps. Use after installing official
Phidget22 Python bindings AND a compatible native library. No automatic gyro
zeroing, firmware update, heater change, or magnetic calibration is performed.
"""
import argparse
import math
import queue
import time
from capture_sensors import BoundedWriter, plain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='New JSONL output path')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--interval-ms', type=int, default=20)
    parser.add_argument('--serial', type=int)
    parser.add_argument('--zero-gyro', action='store_true',
                        help='Keep sensor still: zero gyro before recording in this same session')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 1200:
        parser.error('--seconds must be in (0, 1200]')
    if args.interval_ms <= 0:
        parser.error('--interval-ms must be positive')
    from Phidget22.Devices.Spatial import Spatial
    spatial = Spatial()
    events = queue.Queue(maxsize=2000)
    state = {'dropped': 0, 'error': None}
    start = time.monotonic()
    writer = BoundedWriter(args.output, 64 * 1024**2, 150 * 1024**2)

    def on_data(device, acceleration, angular_rate, magnetic_field, timestamp):
        record = {'topic': 'phidget_raw', 'receipt_unix_s': time.time(),
                  'elapsed_s': time.monotonic() - start,
                  'gyro_zeroed_before_capture': args.zero_gyro,
                  'msg': {'device_timestamp_ms': timestamp,
                          'acceleration_g': list(acceleration),
                          'angular_rate_deg_s': list(angular_rate),
                          'magnetic_field_gauss': list(magnetic_field)}}
        try:
            events.put_nowait(plain_record(record))
        except queue.Full:
            state['dropped'] += 1

    def on_error(device, code, description):
        state['error'] = '{}: {}'.format(code, description)

    try:
        if args.serial is not None:
            spatial.setDeviceSerialNumber(args.serial)
        spatial.setOnErrorHandler(on_error)
        spatial.openWaitForAttachment(10000)
        spatial.setDataInterval(args.interval_ms)
        print('Attached serial {}. Interval {} ms. Raw units: g, deg/s, gauss.'.format(
            spatial.getDeviceSerialNumber(), spatial.getDataInterval()), flush=True)
        if args.zero_gyro:
            print('Keep completely stationary: zeroing gyro; waiting 3 seconds.', flush=True)
            zero_before_capture(spatial)
        start = time.monotonic()
        # Subscribe after zeroing so calibration samples cannot enter the capture.
        spatial.setOnSpatialDataHandler(on_data)
        print('Recording now. Keep still for a stationary test.', flush=True)
        last_sample = start
        next_report = start + 5
        while time.monotonic() - start < args.seconds:
            if state['error']:
                raise RuntimeError('Phidget error: ' + state['error'])
            try:
                record = events.get(timeout=0.2)
            except queue.Empty:
                if time.monotonic() - last_sample > 3:
                    raise RuntimeError('No Spatial samples for three seconds')
                continue
            last_sample = time.monotonic()
            if not writer.write(record):
                raise RuntimeError('Capture stopped: ' + writer.reason)
            if last_sample >= next_report:
                print('{} samples, {} queue drops'.format(
                    writer.counts['phidget_raw'], state['dropped']), flush=True)
                next_report = last_sample + 5
    except KeyboardInterrupt:
        pass
    finally:
        spatial.close()
        writer.close()
        print('Saved {} samples; {} queue drops. No ROS data or TF published.'.format(
            writer.counts['phidget_raw'], state['dropped']), flush=True)


def plain_record(record):
    # plain() operates on ROS messages/sequences, so dictionaries recurse here.
    return {key: plain_record(value) if isinstance(value, dict) else plain(value)
            for key, value in record.items()}


def zero_before_capture(spatial, wait=time.sleep):
    spatial.zeroGyro()
    wait(3.0)


if __name__ == '__main__':
    main()
