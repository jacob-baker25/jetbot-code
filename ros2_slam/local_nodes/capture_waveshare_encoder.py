#!/usr/bin/env python3
"""Capture every Waveshare RP2040 telemetry frame without serial writes."""
from __future__ import print_function

import argparse
import json
from pathlib import Path
import signal
import sys
import time

from waveshare_encoder_protocol import (
    FrameParser, TELEMETRY_FRAME_SIZE, TELEMETRY_FRAME_TYPE, decode_telemetry)


class CaptureAccumulator(object):
    def __init__(self):
        self.count = 0
        self.first = None
        self.last = None
        self.first_receipt_unix = None
        self.last_receipt_unix = None
        self.last_receipt_mono = None
        self.maximum_receipt_gap_s = 0.0

    def record(self, receipt_unix, receipt_mono, decoded):
        self.count += 1
        if self.first is None:
            self.first = decoded
            self.first_receipt_unix = receipt_unix
        if self.last_receipt_mono is not None:
            self.maximum_receipt_gap_s = max(
                self.maximum_receipt_gap_s,
                receipt_mono - self.last_receipt_mono)
        self.last = decoded
        self.last_receipt_unix = receipt_unix
        self.last_receipt_mono = receipt_mono
        row = {
            "version": 1,
            "sequence": self.count,
            "receipt_unix": receipt_unix,
            "receipt_mono": receipt_mono,
        }
        row.update(decoded)
        return row

    def summary(self):
        return {
            "decoded_telemetry_frames": self.count,
            "first_receipt_unix": self.first_receipt_unix,
            "last_receipt_unix": self.last_receipt_unix,
            "maximum_receipt_gap_s": self.maximum_receipt_gap_s,
            "first": self.first,
            "last": self.last,
        }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=900.0)
    parser.add_argument("--minimum-frames", type=int, default=10)
    args = parser.parse_args(argv)
    if args.maximum_seconds <= 0 or args.maximum_seconds > 1800:
        parser.error("maximum-seconds must be greater than zero and at most 1800")
    if args.minimum_frames < 1:
        parser.error("minimum-frames must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        import serial
        args.output.mkdir()
        frames_path = args.output / "frames.jsonl"
        summary_path = args.output / "summary.json"
        serial_port = serial.Serial(
            port=args.port, baudrate=args.baud, timeout=0.1)
    except Exception as error:
        print("WAVESHARE_ENCODER_CAPTURE_ERROR {}".format(error), file=sys.stderr)
        return 2

    stop = {"requested": False, "reason": None}

    def request_stop(signum, unused_frame):
        stop["requested"] = True
        stop["reason"] = "signal_{}".format(signum)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    parser = FrameParser()
    capture = CaptureAccumulator()
    decode_failures = 0
    started_mono = time.monotonic()
    started_unix = time.time()
    print(
        "WAVESHARE_ENCODER_CAPTURE_READY version=1 mode=receive_only "
        "port={} baud={}".format(args.port, args.baud), flush=True)

    try:
        with frames_path.open("x", buffering=1) as output:
            while not stop["requested"]:
                if time.monotonic() - started_mono >= args.maximum_seconds:
                    stop["reason"] = "maximum_seconds"
                    break
                first_byte = serial_port.read(1)
                if not first_byte:
                    continue
                waiting = min(getattr(serial_port, "in_waiting", 0), 4095)
                data = first_byte + (serial_port.read(waiting) if waiting else b"")
                receipt_mono = time.monotonic()
                receipt_unix = time.time()
                for frame in parser.feed(data):
                    if (len(frame) != TELEMETRY_FRAME_SIZE or
                            frame[3] != TELEMETRY_FRAME_TYPE):
                        continue
                    try:
                        decoded = decode_telemetry(frame)
                    except ValueError:
                        decode_failures += 1
                        continue
                    row = capture.record(receipt_unix, receipt_mono, decoded)
                    output.write(json.dumps(row, allow_nan=False) + "\n")
    finally:
        serial_port.close()

    elapsed = time.monotonic() - started_mono
    summary = {
        "version": 1,
        "mode": "receive_only",
        "port": args.port,
        "baud": args.baud,
        "start_unix": started_unix,
        "end_unix": time.time(),
        "duration_s": elapsed,
        "stop_reason": stop["reason"] or "loop_exit",
        "decode_failures": decode_failures,
        "decoded_rate_hz": capture.count / elapsed if elapsed else 0.0,
        "frames_path": str(frames_path),
        "frames_bytes": frames_path.stat().st_size,
        "parser": parser.summary(),
    }
    summary.update(capture.summary())
    passed = (
        capture.count >= args.minimum_frames and
        decode_failures == 0 and
        parser.checksum_failures == 0 and
        parser.buffer_overflows == 0)
    summary["passed"] = passed
    summary_path.write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print("WAVESHARE_ENCODER_CAPTURE_SUMMARY " + json.dumps(
        summary, allow_nan=False, sort_keys=True), flush=True)
    print("WAVESHARE_ENCODER_CAPTURE_{}".format(
        "PASS" if passed else "FAIL"), flush=True)
    return 0 if passed else 3


if __name__ == "__main__":
    sys.exit(main())
