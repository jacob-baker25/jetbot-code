import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "map_navigation", str(ROOT / "analysis" / "analyze_map_navigation.py"))
MAP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MAP)


class MapNavigationTest(unittest.TestCase):
    def test_inflation_blocks_narrow_opening_and_preserves_wide_goal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            # 9x7 map: black border, a vertical wall with one-cell and
            # three-cell openings. P2 keeps the fixture readable.
            rows = [
                [0] * 9,
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0, 254, 254, 254, 254, 254, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0] * 9,
            ]
            pgm = root / "map.pgm"
            pgm.write_text("P2\n9 7\n255\n" + "\n".join(
                " ".join(str(value) for value in row) for row in rows))
            yaml = root / "map.yaml"
            yaml.write_text(
                "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            grid = MAP.OccupancyMap(yaml)
            self.assertEqual((grid.width, grid.height), (9, 7))
            blocked, radius = MAP.inflated_blocked(grid, .09)
            self.assertEqual(radius, 1)
            start = (2, 3)
            reachable = MAP.reachable_cells(grid, blocked, start)
            self.assertNotIn((6, 3), reachable)
            report = MAP.analyze(yaml, .18, 0, start_world=(.25, .35),
                                 goals_world=[(.65, .35)])
            self.assertFalse(report["goals"][0]["reachable"])
            json.dumps(report, allow_nan=False)

    def test_world_conversion_respects_pgm_vertical_axis(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "map.pgm").write_text("P2\n2 2\n255\n254 254\n254 254\n")
            yaml = root / "map.yaml"
            yaml.write_text(
                "image: map.pgm\nresolution: 1\norigin: [0, 0, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            grid = MAP.OccupancyMap(yaml)
            self.assertEqual(grid.world_to_cell(.1, .1), (0, 1))
            self.assertEqual(grid.world_to_cell(.1, 1.1), (0, 0))

    def test_binary_pgm_is_read_without_losing_first_pixel(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "binary.pgm"
            path.write_bytes(b"P5\n# comment\n2 2\n255\n" + bytes([0, 1, 254, 255]))
            width, height, pixels, maximum = MAP.read_pgm(path)
            self.assertEqual((width, height, maximum), (2, 2, 255))
            self.assertEqual(list(pixels), [0, 1, 254, 255])


if __name__ == "__main__":
    unittest.main()
