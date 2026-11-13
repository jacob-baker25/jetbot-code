import ast
import importlib.util
from pathlib import Path
import random
import unittest


ROOT = Path(__file__).resolve().parents[1]
BIAS = ROOT / "local_nodes" / "imu_bias.py"
spec = importlib.util.spec_from_file_location("imu_bias", str(BIAS))
imu_bias = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imu_bias)

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ


def run(profile, noise_dps=0.0, seed=1):
    """profile: list of (seconds, true_rate_dps). Sensor adds 0.1 deg/s bias."""
    rng = random.Random(seed)
    hold = imu_bias.StationaryBiasHold()
    t = 0.0
    raw_total = 0.0
    out_total = 0.0
    still_flags = []
    for seconds, rate in profile:
        for _ in range(int(round(seconds * RATE_HZ))):
            measured = rate + 0.1 + rng.gauss(0.0, noise_dps)
            out = hold.update(t, measured)
            raw_total += measured * DT
            out_total += out * DT
            still_flags.append(hold.still)
            t += DT
    return hold, raw_total, out_total, still_flags


class StationaryBiasHoldTest(unittest.TestCase):

    def test_parked_robot_holds_heading_and_learns_bias(self):
        hold, raw, out, _ = run([(10.0, 0.0)], noise_dps=0.03)
        self.assertAlmostEqual(raw, 1.0, delta=0.05)      # 0.1 deg/s for 10 s
        self.assertLess(abs(out), 0.15)                    # held after the first window
        self.assertAlmostEqual(hold.bias_dps, 0.1, delta=0.02)

    def test_turns_pass_through_minus_bias(self):
        profile = [(5.0, 0.0), (2.0, 45.0), (5.0, 0.0), (2.0, -45.0), (5.0, 0.0)]
        _, raw, out, _ = run(profile, noise_dps=0.03)
        self.assertAlmostEqual(raw, 1.9, delta=0.1)       # drift only, turns cancel
        self.assertLess(abs(out), 0.3)

    def test_single_turn_angle_is_preserved(self):
        hold, _, out, _ = run([(5.0, 0.0), (2.0, 45.0), (5.0, 0.0)], noise_dps=0.03)
        self.assertAlmostEqual(out, 90.0, delta=0.3)
        self.assertGreater(hold.still_fraction(), 0.6)

    def test_turn_is_not_held_from_first_sample(self):
        hold = imu_bias.StationaryBiasHold()
        t = 0.0
        for _ in range(100):
            hold.update(t, 0.1)
            t += DT
        self.assertTrue(hold.still)
        out = hold.update(t, 30.0)
        self.assertFalse(hold.still)
        self.assertAlmostEqual(out, 30.0 - hold.bias_dps)

    def test_slow_steady_rotation_is_not_learned_as_bias(self):
        hold, _, out, flags = run([(10.0, 2.0)])
        self.assertFalse(any(flags))
        self.assertAlmostEqual(hold.bias_dps, 0.0)
        self.assertAlmostEqual(out, 21.0, delta=0.1)

    def test_python36_syntax(self):
        for path in (BIAS, ROOT / "local_nodes" / "phidget_imu_publisher.py"):
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 6))


if __name__ == "__main__":
    unittest.main()
