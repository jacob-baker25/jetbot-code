import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "run_replay_trial", str(ROOT / "run_replay_trial.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ReplayTrialTest(unittest.TestCase):
    def setUp(self):
        self.runner = load_runner()

    def test_name_and_conflict_validation(self):
        self.assertEqual(self.runner.validate_name("moving_baseline-01"),
                         "moving_baseline-01")
        with self.assertRaises(Exception):
            self.runner.validate_name("../unsafe")
        self.assertEqual(
            self.runner.validate_commit("a" * 40), "a" * 40)
        with self.assertRaises(Exception):
            self.runner.validate_commit("ea237fe")
        listing = (
            "  10 harmless_process\n"
            "  20 /tmp/sync_slam_toolbox_node --ros-args\n"
            "  30 python3 replay_player.py --database capture.sqlite3\n")
        self.assertEqual(
            [entry["pid"] for entry in self.runner.find_conflicts(listing, 30)],
            [20])

    def test_player_summary_requires_exactly_one_summary(self):
        summary = {"late_messages": 0, "published_counts": {"/scan_slam": 2}}
        text = "ready\nREPLAY_PLAYER_SUMMARY {}\n".format(json.dumps(summary))
        self.assertEqual(self.runner.parse_player_summary(text), summary)
        with self.assertRaises(self.runner.ReplayTrialError):
            self.runner.parse_player_summary("no summary\n")

    def test_hash_json_and_archive_helpers(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"; run.mkdir()
            (run / "value.txt").write_bytes(b"value\n")
            self.runner.write_json(run / "result.json", {"passed": True})
            archive = root / "run.tar.gz"
            self.runner.archive_directory(run, archive)
            self.assertTrue(archive.is_file())
            self.assertEqual(
                self.runner.sha256(run / "value.txt"),
                "1e1f2c881ae0608ec77ebf88a75c66d3099113a7343238f2f7a0ebb91a4ed335")

    def test_rotation_filter_argument_is_opt_in_and_bounded(self):
        parser = self.runner.parser()
        common = [
            "run", "--source-git-head", "a" * 40,
            "--database", "capture.sqlite3", "--overlay-root", "overlay",
        ]
        self.assertIsNone(parser.parse_args(common).maximum_scan_rotation_degrees)
        self.assertEqual(
            parser.parse_args(common + [
                "--maximum-scan-rotation-degrees", "1.0"
            ]).maximum_scan_rotation_degrees,
            1.0)
        self.assertIsNone(
            parser.parse_args(common).maximum_scan_translation_meters)
        self.assertEqual(
            parser.parse_args(common + [
                "--maximum-scan-translation-meters", "0.003"
            ]).maximum_scan_translation_meters,
            0.003)
        self.assertFalse(parser.parse_args(common).deskew_scans)
        self.assertTrue(parser.parse_args(common + ["--deskew-scans"]).deskew_scans)
        self.assertFalse(parser.parse_args(common).stationary_median_scans)
        self.assertTrue(parser.parse_args(common + [
            "--stationary-median-scans"
        ]).stationary_median_scans)


if __name__ == "__main__":
    unittest.main()
