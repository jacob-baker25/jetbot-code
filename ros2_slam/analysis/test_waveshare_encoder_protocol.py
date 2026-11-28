import importlib.util
from pathlib import Path
import struct
import sys
import types
import unittest


ROOT = Path(__file__).resolve().parents[1]
LOCAL_NODES = ROOT / "local_nodes"
sys.path.insert(0, str(LOCAL_NODES))

from waveshare_encoder_protocol import (  # noqa: E402
    FrameParser, checksum, decode_telemetry)


def load_capture_module():
    spec = importlib.util.spec_from_file_location(
        "capture_waveshare_encoder",
        str(LOCAL_NODES / "capture_waveshare_encoder.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def put_i16(frame, offset, value):
    frame[offset:offset + 2] = struct.pack(">h", value)


def telemetry_frame():
    frame = bytearray(45)
    frame[:4] = b"\xaa\x55\x2d\x01"
    values = {
        4: -100, 6: 200, 8: -300,
        10: 1000, 12: -2000, 14: 3000,
        16: 123, 18: -456, 20: 900,
        22: 1234, 24: -567, 26: 1571,
        28: 12, 30: -3, 32: 4,
        34: 101, 36: -102, 38: 103, 40: -104,
    }
    for offset, value in values.items():
        put_i16(frame, offset, value)
    frame[42:44] = b"\x12\x34"
    frame[-1] = checksum(frame[:-1])
    return bytes(frame)


class FrameParserTest(unittest.TestCase):
    def test_decodes_documented_fields_and_signed_values(self):
        decoded = decode_telemetry(telemetry_frame())
        self.assertAlmostEqual(decoded["pose"]["x_m"], 1.234)
        self.assertAlmostEqual(decoded["pose"]["y_m"], -0.567)
        self.assertAlmostEqual(decoded["pose"]["yaw_rad"], 1.571)
        self.assertEqual(decoded["motor"]["left_velocity_ticks"], 101)
        self.assertEqual(decoded["motor"]["right_velocity_ticks"], -102)
        self.assertEqual(decoded["uninterpreted_tail_hex"], "1234")

    def test_accepts_split_frame(self):
        parser = FrameParser()
        frame = telemetry_frame()
        self.assertEqual(parser.feed(frame[:7]), [])
        self.assertEqual(parser.feed(frame[7:]), [frame])
        self.assertEqual(parser.summary()["frames_received"], 1)

    def test_resynchronizes_after_garbage(self):
        parser = FrameParser()
        frame = telemetry_frame()
        self.assertEqual(parser.feed(b"garbage\xaa" + frame), [frame])
        self.assertGreaterEqual(parser.summary()["discarded_bytes"], 7)

    def test_rejects_bad_checksum_then_finds_next_frame(self):
        parser = FrameParser()
        bad = bytearray(telemetry_frame())
        bad[22] ^= 1
        good = telemetry_frame()
        self.assertEqual(parser.feed(bytes(bad) + good), [good])
        self.assertEqual(parser.summary()["checksum_failures"], 1)

    def test_rejects_invalid_size(self):
        parser = FrameParser()
        good = telemetry_frame()
        self.assertEqual(parser.feed(b"\xaa\x55\x01" + good), [good])
        self.assertEqual(parser.summary()["invalid_sizes"], 1)

    def test_probe_source_contains_no_serial_write(self):
        for name in ("probe_waveshare_encoder.py", "capture_waveshare_encoder.py"):
            source = (LOCAL_NODES / name).read_text()
            self.assertNotIn("serial_port.write(", source)
            self.assertIn("receive_only", source)

    def test_capture_accumulator_preserves_receipts_and_gaps(self):
        module = load_capture_module()
        accumulator = module.CaptureAccumulator()
        first = decode_telemetry(telemetry_frame())
        second = dict(first)
        row1 = accumulator.record(100.0, 20.0, first)
        row2 = accumulator.record(100.02, 20.02, second)
        self.assertEqual(row1["sequence"], 1)
        self.assertEqual(row2["sequence"], 2)
        summary = accumulator.summary()
        self.assertEqual(summary["decoded_telemetry_frames"], 2)
        self.assertAlmostEqual(summary["maximum_receipt_gap_s"], 0.02)
        self.assertEqual(summary["first_receipt_unix"], 100.0)
        self.assertEqual(summary["last_receipt_unix"], 100.02)


if __name__ == "__main__":
    unittest.main()
