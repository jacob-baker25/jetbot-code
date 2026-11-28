import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "analyze_wheel_checkpoints",
    str(ROOT / "analysis" / "analyze_wheel_checkpoints.py"))
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def checkpoint(index, stamp, x):
    pose = {"x": x, "y": 0.0, "yaw_deg": 0.0}
    return {
        "index": index,
        "label": "checkpoint_{}".format(index),
        "time_unix": stamp,
        "rf2o": dict(pose),
        "raw_rf2o": dict(pose),
    }


def frame(sequence, stamp, x):
    return {
        "version": 1,
        "sequence": sequence,
        "receipt_unix": stamp,
        "pose": {"x_m": x, "y_m": 0.0, "yaw_rad": 0.0},
        "motor": {
            "left_velocity_ticks": 0,
            "right_velocity_ticks": 0,
            "left_setpoint_ticks": 0,
            "right_setpoint_ticks": 0,
        },
    }


class WheelCheckpointAnalysisTest(unittest.TestCase):
    def test_aligns_nearest_frames_and_computes_relative_segments(self):
        frames = [frame(1, 9.99, 5.0), frame(2, 11.01, 6.0), frame(3, 12.0, 7.0)]
        results = {"checkpoints": [checkpoint(0, 10.0, 0.0),
                                    checkpoint(1, 11.0, 0.9),
                                    checkpoint(2, 12.01, 1.8)]}
        report = analysis.analyze(results, frames)
        self.assertTrue(report["alignment_passed"])
        self.assertAlmostEqual(report["checkpoints"][1]["wheel"]["distance_m"], 1.0)
        self.assertAlmostEqual(report["segments"][1]["wheel_distance_m"], 1.0)
        self.assertAlmostEqual(
            report["checkpoints"][2]["fused_rf2o"]["distance_m"], 1.8)

    def test_alignment_age_gate_is_explicit(self):
        frames = [frame(1, 9.0, 0.0), frame(2, 10.0, 1.0)]
        results = {"checkpoints": [checkpoint(0, 9.0, 0.0),
                                    checkpoint(1, 10.2, 1.0)]}
        report = analysis.analyze(results, frames, maximum_age_s=0.1)
        self.assertFalse(report["alignment_passed"])
        self.assertAlmostEqual(report["maximum_checkpoint_alignment_age_s"], 0.2)

    def test_loader_rejects_nonmonotonic_or_empty_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.jsonl"
            path.write_text("")
            with self.assertRaises(ValueError):
                analysis.load_frames(path)
            path.write_text(
                '{"version":1,"receipt_unix":2}\n'
                '{"version":1,"receipt_unix":1}\n')
            with self.assertRaises(ValueError):
                analysis.load_frames(path)


if __name__ == "__main__":
    unittest.main()
