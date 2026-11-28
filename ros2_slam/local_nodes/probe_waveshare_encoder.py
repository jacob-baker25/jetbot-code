#!/usr/bin/env python3
"""Bounded, receive-only probe for Waveshare JetBot RP2040 telemetry."""
from __future__ import print_function

import argparse
import json
from pathlib import Path
import sys
import time

from waveshare_encoder_protocol import (
    FrameParser, TELEMETRY_FRAME_SIZE, TELEMETRY_FRAME_TYPE, decode_telemetry)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Read Waveshare encoder telemetry without writing commands")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--minimum-frames", type=int, default=10)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.duration > 30:
        parser.error("duration must be greater than zero and at most 30 seconds")
    if args.minimum_frames < 1:
        parser.error("minimum-frames must be positive")
    return args


def _update_range(ranges, key, value):
    current = ranges.setdefault(key, {"minimum": value, "maximum": value})
    current["minimum"] = min(current["minimum"], value)
    current["maximum"] = max(current["maximum"], value)


def run_probe(port, baud, duration, serial_factory=None):
    if serial_factory is None:
        import serial
        serial_factory = serial.Serial

    parser = FrameParser()
    decoded_count = 0
    decode_failures = 0
    first = None
    last = None
    ranges = {}
    started = time.monotonic()
    serial_port = serial_factory(port=port, baudrate=baud, timeout=0.2)
    try:
        while time.monotonic() - started < duration:
            data = serial_port.read(512)
            for frame in parser.feed(data):
                if (len(frame) != TELEMETRY_FRAME_SIZE or
                        frame[3] != TELEMETRY_FRAME_TYPE):
                    continue
                try:
                    decoded = decode_telemetry(frame)
                except ValueError:
                    decode_failures += 1
                    continue
                decoded_count += 1
                if first is None:
                    first = decoded
                last = decoded
                for name, value in decoded["pose"].items():
                    _update_range(ranges, "pose." + name, value)
                for name, value in decoded["motor"].items():
                    _update_range(ranges, "motor." + name, value)
    finally:
        serial_port.close()

    elapsed = time.monotonic() - started
    summary = parser.summary()
    summary.update({
        "version": 1,
        "mode": "receive_only",
        "port": port,
        "baud": baud,
        "requested_duration_s": duration,
        "elapsed_s": elapsed,
        "decoded_telemetry_frames": decoded_count,
        "decode_failures": decode_failures,
        "decoded_rate_hz": decoded_count / elapsed if elapsed else 0.0,
        "first": first,
        "last": last,
        "ranges": ranges,
    })
    return summary


def main(argv=None):
    args = parse_args(argv)
    print("WAVESHARE_ENCODER_PROBE_START mode=receive_only port={} baud={}".format(
        args.port, args.baud), flush=True)
    try:
        summary = run_probe(args.port, args.baud, args.duration)
    except Exception as error:
        print("WAVESHARE_ENCODER_PROBE_ERROR {}".format(error), file=sys.stderr)
        return 2

    encoded = json.dumps(summary, sort_keys=True)
    print("WAVESHARE_ENCODER_PROBE_SUMMARY " + encoded)
    if args.output:
        Path(args.output).write_text(encoded + "\n")
    if summary["decoded_telemetry_frames"] < args.minimum_frames:
        print("WAVESHARE_ENCODER_PROBE_FAIL expected at least {} frames".format(
            args.minimum_frames), file=sys.stderr)
        return 3
    print("WAVESHARE_ENCODER_PROBE_PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
