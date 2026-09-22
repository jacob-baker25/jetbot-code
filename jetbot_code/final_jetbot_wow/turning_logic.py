"""
hallway_turn.py — Drive straight, turn one corner (left or right), stop.

Turn uses FFT cross-correlation scan matching (the same principle behind
Olson's Correlative Scan Matching and Hector SLAM's scanmatcher) to
measure cumulative rotation.  The robot pivots until it has swept exactly
~90°, regardless of hallway width, entry angle, or corner geometry.

Usage:
    python3 hallway_turn.py left
    python3 hallway_turn.py right
"""

from rplidar import RPLidar, RPLidarException
from jetbot import Robot
import sys, time
import numpy as np

# ── Motor setup ──────────────────────────────────────────────────────
robot = Robot()

def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

MOTOR_DIR_L, MOTOR_DIR_R = -1.0, -1.0   # flip if your motors are wired backward
BIAS_L, BIAS_R           = 1.00, 1.00   # per-motor trim

FORWARD_SPEED = 0.50
CREEP_SPEED   = 0.40
PIVOT_SPEED   = 0.30   # outer-wheel speed during pivot
PIVOT_RATIO   = 0.0    # 0.0 = point-turn, 0.3 = gentle arc

def _set(l, r):
    robot.set_motors(clamp(l * MOTOR_DIR_L * BIAS_L),
                     clamp(r * MOTOR_DIR_R * BIAS_R))

def stop():          _set(0, 0)
def drive(s):        _set(s, s)

# *** POLARITY FIX ***
# Right turn = RIGHT wheel forward, LEFT wheel slow/stopped  (swapped from before)
# Left  turn = LEFT wheel forward, RIGHT wheel slow/stopped
def pivot_right(speed=PIVOT_SPEED):  _set(speed * PIVOT_RATIO, speed)
def pivot_left(speed=PIVOT_SPEED):   _set(speed, speed * PIVOT_RATIO)

# ── LIDAR setup ──────────────────────────────────────────────────────
PORT      = '/dev/ttyUSB0'
BAUDRATE  = 256000
DMAX      = 12000        # mm
SCAN_SIZE = 360

# Sector definitions (center°, half-width°)
FRONT_SECTOR  = (0.0,   20.0)
RIGHT_SECTOR  = (60.0,  30.0)
LEFT_SECTOR   = (300.0, 30.0)

# ── Thresholds ───────────────────────────────────────────────────────
# Corner detection
CORNER_OPEN_THRESH_M  = 2.0    # side range meaning "no wall = opening"
CORNER_OPEN_FRACTION  = 0.65   # fraction of side-sector points that must be far
CORNER_OPEN_DEBOUNCE  = 3      # consecutive open scans before we trust it

# Intersection validation — a real corner must satisfy ALL of these:
#   1) turn-side is open  (the opening itself)
#   2) opposite side has a wall at normal distance  (we're still in a hallway)
#   3) front is reasonably clear  (we're not jammed against a wall at an angle)
OPP_WALL_MIN_M        = 0.15   # opposite wall must be at least this far
OPP_WALL_MAX_M        = 3.5    # opposite wall must be closer than this
FRONT_CLEAR_MIN_M     = 0.8    # front must be at least this clear

# Turn parameters
TARGET_ROTATION_DEG   = 85.0   # aim under 90 — ramp-down + coast closes the gap
ROTATION_TOLERANCE    = 4.0    # accept within ±this of target
ROTATION_DONE_DEBOUNCE = 2     # consecutive scans at target before stopping
TURN_GRACE_S          = 0.8    # ignore rotation checks while motors spin up
TURN_TIMEOUT_S        = 10.0   # safety abort

# Deceleration ramp — robot slows as it approaches the target so
# there is almost zero momentum left when the stop command fires.
RAMP_START_DEG        = 25.0   # begin slowing this many degrees before target
PIVOT_SPEED_MIN       = 0.15   # minimum pivot speed at end of ramp

# Creep into the corner mouth before pivoting
CREEP_INTO_CORNER_S   = 4.0

# ── Scan-matching rotation tracker ───────────────────────────────────
#
# Principle (from Correlative Scan Matching / Olson 2009):
#   Two LIDAR scans of the same environment taken at different headings
#   are circular shifts of each other (translation is negligible during
#   a point-turn).  The shift that maximises their cross-correlation
#   equals the rotation in degrees (for a 1°-resolution scan).
#
# We use FFT-based circular cross-correlation — O(n log n), runs in
# under 1 ms on the Jetson Nano.  The range profile is normalised to
# zero-mean unit-variance so only shape matters, not absolute distance.
#

def normalise_scan(scan_mm):
    """Convert raw mm scan to a zero-mean, unit-variance float array."""
    s = np.array(scan_mm, dtype=np.float64)
    s = np.clip(s, 0, DMAX)
    mu = s.mean()
    sigma = s.std()
    if sigma < 1e-6:
        return s - mu
    return (s - mu) / sigma

def scan_rotation_deg(ref_norm, cur_norm, search_limit=120):
    """
    Return the rotation (degrees) from ref to cur using FFT
    circular cross-correlation.

    Positive = clockwise rotation (robot turned RIGHT).
    search_limit restricts the answer to ±search_limit degrees
    to reject spurious 180° aliases in symmetric hallways.
    """
    n = len(ref_norm)
    # Circular cross-correlation via FFT
    corr = np.real(np.fft.ifft(np.fft.fft(ref_norm) * np.conj(np.fft.fft(cur_norm))))

    # Build candidate indices: [0..search_limit] ∪ [n-search_limit..n-1]
    pos = list(range(0, min(search_limit + 1, n)))
    neg = list(range(max(0, n - search_limit), n))
    candidates = pos + neg

    best_idx = max(candidates, key=lambda i: corr[i])
    shift = best_idx if best_idx <= n // 2 else best_idx - n

    return float(shift)

# ── LIDAR helpers ────────────────────────────────────────────────────
def wrap(d):
    return d % 360.0

def in_sector(angle, center, half):
    diff = (wrap(angle) - wrap(center) + 540.0) % 360.0 - 180.0
    return abs(diff) <= half

def sector_ranges(scan_mm, center, half):
    out = []
    for deg, r in enumerate(scan_mm):
        if not in_sector(deg, center, half):
            continue
        if r <= 0:
            continue
        out.append(min(r, DMAX) / 1000.0)
    return out

def side_is_open(scan_mm, sector):
    """True if most of the sector sees far-away (no wall)."""
    r = sector_ranges(scan_mm, sector[0], sector[1])
    if len(r) < 6:
        return False
    far = sum(1 for v in r if v >= CORNER_OPEN_THRESH_M)
    return (far / len(r)) >= CORNER_OPEN_FRACTION

def is_real_intersection(scan_mm, turn_sector, opp_sector):
    """
    Confirm that the opening on the turn-side is a genuine intersection,
    not a false positive from the robot being angled against a wall.

    A real intersection looks like:
      • turn-side   → mostly open (long ranges)
      • opposite    → wall at normal hallway distance (0.25 – 3.5 m)
      • front       → reasonably clear (> 0.8 m)

    When angled against a wall the opposite side reads very short (< 0.25 m)
    or the front is blocked, failing the check.
    """
    if not side_is_open(scan_mm, turn_sector):
        return False, "turn-side not open"

    # Opposite wall sanity check
    opp = sector_ranges(scan_mm, opp_sector[0], opp_sector[1])
    if len(opp) < 5:
        return False, "opposite-side no data"
    opp_p10 = float(np.percentile(opp, 10))
    if opp_p10 < OPP_WALL_MIN_M:
        return False, f"opposite too close ({opp_p10:.2f}m) — angled into wall?"
    if opp_p10 > OPP_WALL_MAX_M:
        return False, f"opposite too far ({opp_p10:.2f}m) — no wall?"

    # Front clearance check
    fwd = sector_ranges(scan_mm, FRONT_SECTOR[0], FRONT_SECTOR[1])
    if len(fwd) < 5:
        return False, "front no data"
    fwd_p10 = float(np.percentile(fwd, 10))
    if fwd_p10 < FRONT_CLEAR_MIN_M:
        return False, f"front blocked ({fwd_p10:.2f}m) — angled into wall?"

    return True, f"VALID (opp={opp_p10:.2f}m, fwd={fwd_p10:.2f}m)"

# ── Main FSM ─────────────────────────────────────────────────────────
def run(direction):
    direction = direction.upper()
    assert direction in ("LEFT", "RIGHT"), "Argument must be 'left' or 'right'"

    turn_sector = RIGHT_SECTOR if direction == "RIGHT" else LEFT_SECTOR
    opp_sector  = LEFT_SECTOR  if direction == "RIGHT" else RIGHT_SECTOR
    do_pivot = pivot_right if direction == "RIGHT" else pivot_left

    # Scan-matching sign convention: right turn shifts scan indices positively
    rotation_sign = 1.0 if direction == "RIGHT" else -1.0

    lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=2)
    lidar.start_motor()
    time.sleep(1.0)
    try:
        iterator = lidar.iter_scans(max_buf_meas=4000, min_len=5)
    except Exception:
        iterator = lidar.iter_scans()

    state       = "STRAIGHT"
    open_hits   = 0
    done_hits   = 0
    creep_start = None
    turn_start  = None
    ref_norm    = None      # normalised reference scan captured at turn start
    frame       = 0

    print(f"=== Hallway Turn: {direction} ===")
    print(f"Target rotation: {TARGET_ROTATION_DEG}° ± {ROTATION_TOLERANCE}°")
    print("Driving straight — waiting for corner opening on the "
          f"{'right' if direction == 'RIGHT' else 'left'} side...\n")

    try:
        while True:
            # ── Read one LIDAR scan ──
            frame += 1
            scan = [DMAX] * SCAN_SIZE
            for _, theta, r in next(iterator):
                scan[int(theta) % SCAN_SIZE] = int(r)
            now = time.time()

            # ─────────────────────────────────────────────────────────
            if state == "STRAIGHT":
                drive(FORWARD_SPEED)

                valid, reason = is_real_intersection(scan, turn_sector, opp_sector)
                if valid:
                    open_hits += 1
                else:
                    open_hits = 0

                if open_hits >= CORNER_OPEN_DEBOUNCE:
                    state = "CREEP"
                    creep_start = now
                    print(f"[NAV] Intersection confirmed ({reason}) — creeping to mouth...")

                if frame % 10 == 0:
                    sr = sector_ranges(scan, *turn_sector)
                    p10 = f"{float(np.percentile(sr, 10)):.2f}m" if len(sr) >= 5 else "?"
                    print(f"[STRAIGHT] turn-side p10: {p10}  open_hits: {open_hits}"
                          f"  check: {reason}")

            # ─────────────────────────────────────────────────────────
            elif state == "CREEP":
                drive(CREEP_SPEED)
                if now - creep_start >= CREEP_INTO_CORNER_S:
                    # Capture reference scan RIGHT NOW, before any rotation
                    ref_norm = normalise_scan(scan)
                    state = "TURNING"
                    turn_start = now
                    done_hits = 0
                    print(f"[NAV] Reference scan captured. Pivoting {direction}...")

            # ─────────────────────────────────────────────────────────
            elif state == "TURNING":
                elapsed = now - turn_start

                # ── Measure cumulative rotation via scan matching ──
                cur_norm = normalise_scan(scan)
                raw_rotation = scan_rotation_deg(ref_norm, cur_norm)
                signed_rotation = raw_rotation * rotation_sign

                # ── Compute ramped pivot speed ──
                remaining = max(0.0, TARGET_ROTATION_DEG - signed_rotation)
                if remaining < RAMP_START_DEG and signed_rotation > 10:
                    # Linear ramp from full speed → min speed
                    t = 1.0 - (remaining / RAMP_START_DEG)   # 0→1 as we approach target
                    speed = PIVOT_SPEED - t * (PIVOT_SPEED - PIVOT_SPEED_MIN)
                else:
                    speed = PIVOT_SPEED
                do_pivot(speed)

                # Grace period: motors need time to actually start moving
                if elapsed < TURN_GRACE_S:
                    if frame % 5 == 0:
                        print(f"[TURNING] {elapsed:.1f}s  (grace)  "
                              f"rotation: {signed_rotation:.1f}°")
                    continue

                # Check if we've reached target
                error = abs(signed_rotation - TARGET_ROTATION_DEG)
                at_target = error <= ROTATION_TOLERANCE
                overshot  = signed_rotation > (TARGET_ROTATION_DEG + ROTATION_TOLERANCE)

                if at_target or overshot:
                    done_hits += 1
                else:
                    done_hits = 0

                if done_hits >= ROTATION_DONE_DEBOUNCE:
                    stop()
                    print(f"\n[NAV] ✓ Turn complete in {elapsed:.1f}s  "
                          f"(measured rotation: {signed_rotation:.1f}°)")
                    state = "DONE"

                elif elapsed > TURN_TIMEOUT_S:
                    stop()
                    print(f"\n[NAV] ✗ Turn timed out after {TURN_TIMEOUT_S}s  "
                          f"(reached {signed_rotation:.1f}°) — stopping.")
                    state = "DONE"

                elif frame % 3 == 0:
                    print(f"[TURNING] {elapsed:.1f}s  rotation: {signed_rotation:.1f}° / "
                          f"{TARGET_ROTATION_DEG}°  speed: {speed:.2f}  done_hits: {done_hits}")

            # ─────────────────────────────────────────────────────────
            elif state == "DONE":
                stop()
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stop()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

# ── Entry point ──────────────────────────────────────────────────────
if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1].lower() not in ("left", "right"):
        print("Usage: python3 hallway_turn.py <left|right>")
        sys.exit(1)
    run(sys.argv[1])