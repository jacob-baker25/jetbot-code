"""Read-only decoder for the Waveshare JetBot ROS RP2040 telemetry frame.

The wire layout is taken from Waveshare's official ``jetbot_pro`` driver at
commit b76d483fbd5ceb41a66be4945ed2fc712b5d36a4.  This module deliberately
contains no serial-port writes or motor-control frame builders.

Reference:
https://github.com/waveshare/jetbot_pro/blob/main/src/jetbot.cpp
"""
from __future__ import print_function

import math
import struct


HEADER = b"\xaa\x55"
TELEMETRY_FRAME_SIZE = 45
TELEMETRY_FRAME_TYPE = 0x01
MINIMUM_FRAME_SIZE = 5
MAXIMUM_FRAME_SIZE = 128


def checksum(frame_without_checksum):
    return sum(bytearray(frame_without_checksum)) & 0xff


def frame_is_valid(frame):
    return (len(frame) >= MINIMUM_FRAME_SIZE and
            frame[:2] == HEADER and
            frame[2] == len(frame) and
            checksum(frame[:-1]) == frame[-1])


def _signed_i16(frame, offset):
    return struct.unpack(">h", frame[offset:offset + 2])[0]


def _vector(frame, offsets, scale):
    return [_signed_i16(frame, offset) * scale for offset in offsets]


def decode_telemetry(frame):
    """Decode one validated manufacturer telemetry frame into SI units."""
    if not frame_is_valid(frame):
        raise ValueError("invalid Waveshare telemetry frame")
    if len(frame) != TELEMETRY_FRAME_SIZE:
        raise ValueError("unexpected telemetry frame size: {}".format(len(frame)))
    if frame[3] != TELEMETRY_FRAME_TYPE:
        raise ValueError("unexpected telemetry frame type: {}".format(frame[3]))

    gyro_scale = 2000.0 * math.pi / (32768.0 * 180.0)
    acceleration_scale = 2.0 * 9.8 / 32768.0
    return {
        "frame_size": len(frame),
        "frame_type": frame[3],
        "gyro_rad_s": _vector(frame, (4, 6, 8), gyro_scale),
        "acceleration_m_s2": _vector(
            frame, (10, 12, 14), acceleration_scale),
        "attitude_deg": _vector(frame, (16, 18, 20), 0.1),
        "pose": {
            "x_m": _signed_i16(frame, 22) / 1000.0,
            "y_m": _signed_i16(frame, 24) / 1000.0,
            "yaw_rad": _signed_i16(frame, 26) / 1000.0,
        },
        "delta": {
            "x_m": _signed_i16(frame, 28) / 1000.0,
            "y_m": _signed_i16(frame, 30) / 1000.0,
            "yaw_rad": _signed_i16(frame, 32) / 1000.0,
        },
        "motor": {
            "left_velocity_ticks": _signed_i16(frame, 34),
            "right_velocity_ticks": _signed_i16(frame, 36),
            "left_setpoint_ticks": _signed_i16(frame, 38),
            "right_setpoint_ticks": _signed_i16(frame, 40),
        },
        # The official driver does not interpret bytes 42--43. Preserve them
        # for attribution without assigning unsupported semantics.
        "uninterpreted_tail_hex": frame[42:44].hex(),
    }


class FrameParser(object):
    """Incremental, bounded and resynchronizing parser for framed telemetry."""
    def __init__(self, maximum_buffer_bytes=4096):
        if maximum_buffer_bytes < MAXIMUM_FRAME_SIZE:
            raise ValueError("maximum buffer is too small")
        self.maximum_buffer_bytes = maximum_buffer_bytes
        self.buffer = bytearray()
        self.bytes_received = 0
        self.frames_received = 0
        self.checksum_failures = 0
        self.invalid_sizes = 0
        self.discarded_bytes = 0
        self.buffer_overflows = 0
        self.frame_type_counts = {}

    def _discard(self, count):
        if count:
            del self.buffer[:count]
            self.discarded_bytes += count

    def _consume(self, count):
        if count:
            del self.buffer[:count]

    def feed(self, data):
        if not data:
            return []
        self.bytes_received += len(data)
        self.buffer.extend(bytearray(data))
        if len(self.buffer) > self.maximum_buffer_bytes:
            overflow = len(self.buffer) - self.maximum_buffer_bytes
            self._discard(overflow)
            self.buffer_overflows += 1

        frames = []
        while True:
            header_offset = self.buffer.find(HEADER)
            if header_offset < 0:
                keep = 1 if self.buffer and self.buffer[-1] == HEADER[0] else 0
                self._discard(len(self.buffer) - keep)
                break
            self._discard(header_offset)
            if len(self.buffer) < 3:
                break

            frame_size = self.buffer[2]
            if not MINIMUM_FRAME_SIZE <= frame_size <= MAXIMUM_FRAME_SIZE:
                self.invalid_sizes += 1
                self._discard(1)
                continue
            if len(self.buffer) < frame_size:
                break

            candidate = bytes(self.buffer[:frame_size])
            if checksum(candidate[:-1]) != candidate[-1]:
                self.checksum_failures += 1
                self._discard(1)
                continue

            self._consume(frame_size)
            self.frames_received += 1
            frame_type = candidate[3]
            self.frame_type_counts[frame_type] = (
                self.frame_type_counts.get(frame_type, 0) + 1)
            frames.append(candidate)
        return frames

    def summary(self):
        return {
            "bytes_received": self.bytes_received,
            "frames_received": self.frames_received,
            "checksum_failures": self.checksum_failures,
            "invalid_sizes": self.invalid_sizes,
            "discarded_bytes": self.discarded_bytes,
            "buffer_overflows": self.buffer_overflows,
            "buffered_bytes": len(self.buffer),
            "frame_type_counts": {
                str(key): value for key, value in self.frame_type_counts.items()
            },
        }
