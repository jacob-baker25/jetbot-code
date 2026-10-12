"""
demonstration.py — Cruise a hallway and execute a LEFT → LEFT → RIGHT turn sequence.

Logic is pulled directly from:
  • cruise_control.py  — PD wall-following (centering between walls / single-wall follow)
  • turning_logic.py   — Corner detection, intersection validation, FFT scan-matched 90° pivot

Route:
  1) Cruise → detect left opening  → turn LEFT  → cruise cooldown
  2) Cruise → detect left opening  → turn LEFT  → cruise cooldown
  3) Cruise → detect right opening → turn RIGHT → stop

After each completed turn the robot cruises straight for CRUISE_COOLDOWN_S
seconds before it starts looking for the next corner.  This prevents
false re-triggers from residual intersection geometry.

Usage:
    python3 demonstration.py
"""

from rplidar import RPLidar
from jetbot import Robot
import time
import numpy as np

# ╔══════════════════════════════════════════════════════════════════════╗
# ║              ROUTE DEFINITION                                       ║
# ╚══════════════════════════════════════════════════════════════════════╝
# Each entry is a direction to turn at the next detected corner.
ROUTE = ["LEFT", "LEFT", "RIGHT"]

# After completing a turn + settle, cruise blindly for this long before
# looking for the next corner.  Prevents immediate re-trigger.
CRUISE_COOLDOWN_S = 5.0

# ╔══════════════════════════════════════════════════════════════════════╗
# ║              HARDWARE CONFIG  (from cruise_control.py)              ║
# ╚══════════════════════════════════════════════════════════════════════╝
robot = Robot()

PORT      = '/dev/ttyUSB0'
BAUDRATE  = 256000
DMAX      = 12000          # mm — max meaningful LiDAR range
SCAN_SIZE = 360

# Motor calibration (shared by both source files)
MOTOR_DIR_L, MOTOR_DIR_R = -1.0, -1.0
BIAS_L, BIAS_R           = 1.00, 1.00

def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def _set(l, r):
    robot.set_motors(clamp(l * MOTOR_DIR_L * BIAS_L),
                     clamp(r * MOTOR_DIR_R * BIAS_R))

def stop():          _set(0, 0)
def drive(s):        _set(s, s)
def drive_raw(l, r): _set(l, r)

# ╔══════════════════════════════════════════════════════════════════════╗
# ║              SPEED CONSTANTS                                        ║
# ╚══════════════════════════════════════════════════════════════════════╝
# Cruise (from cruise_control.py)
CRUISE_SPEED = 1.00

# Turning (from turning_logic.py)
CREEP_SPEED     = 0.40
PIVOT_SPEED     = 0.30
PIVOT_SPEED_MIN = 0.15
PIVOT_RATIO     = 0.0    # 0.0 = point-turn
SETTLE_SPEED    = 0.85

# ╔══════════════════════════════════════════════════════════════════════╗
# ║              LIDAR SECTORS  (from cruise_control.py + turning_logic)║
# ╚══════════════════════════════════════════════════════════════════════╝

# PD Cruise centering sectors (cruise_control.py)
CENTERING_RIGHT = (90.0,  40.0)
CENTERING_LEFT  = (270.0, 40.0)

# Corner detection sectors (turning_logic.py)
FRONT_SECTOR = (0.0,   20.0)
RIGHT_SECTOR = (60.0,  30.0)
LEFT_SECTOR  = (300.0, 30.0)

# ╔══════════════════════════════════════════════════════════════════════╗
# ║              CRUISE PD THRESHOLDS  (from cruise_control.py)         ║
# ╚══════════════════════════════════════════════════════════════════════╝
GAP_THRESHOLD_M      = 1.8   # Ignore readings beyond this (likely a door/gap)
SINGLE_WALL_TARGET_M = 1.25  # Desired distance from a single wall

CRUISE_KP = 0.12   # Sensitivity to error
CRUISE_KD = 0.08   # "Braking" force to stop oscillation

# ╔══════════════════════════════════════════════════════════════════════╗
# ║              CORNER / TURN THRESHOLDS  (from turning_logic.py)      ║
# ╚══════════════════════════════════════════════════════════════════════╝
CORNER_OPEN_THRESH_M   = 2.0   # side range meaning "no wall = opening"
CORNER_OPEN_FRACTION   = 0.65  # fraction of side-sector points that must be far
CORNER_OPEN_DEBOUNCE   = 3     # consecutive open scans before we trust it

OPP_WALL_MIN_M         = 0.15  # opposite wall must be at least this far
OPP_WALL_MAX_M         = 3.5   # opposite wall must be closer than this
FRONT_CLEAR_MIN_M      = 0.8   # front must be at least this clear

TARGET_ROTATION_DEG    = 85.0  # aim under 90 — ramp-down + coast closes the gap
ROTATION_TOLERANCE     = 4.0   # accept within ±this of target
ROTATION_DONE_DEBOUNCE = 2     # consecutive scans at target before stopping
RAMP_START_DEG         = 25.0  # begin slowing this many degrees before target
TURN_GRACE_S           = 0.8   # ignore rotation checks while motors spin up
TURN_TIMEOUT_S         = 10.0  # safety abort
CREEP_INTO_CORNER_S    = 4.0   # drive into corner mouth before pivoting
SETTLE_S               = 1.5   # blind cruise to clear intersection after turn

# ╔══════════════════════════════════════════════════════════════════════╗
# ║        CRUISE LIDAR HELPER  (from cruise_control.py)                ║
# ╚══════════════════════════════════════════════════════════════════════╝

def get_sector_dist(scan_data, center, half_width):
    """10th-percentile wall distance estimate (cruise_control.py logic)."""
    ranges = []
    for deg in range(int(center - half_width), int(center + half_width)):
        r_mm = scan_data[deg % SCAN_SIZE]
        if 150 < r_mm < DMAX:          # Filter out near-field noise and maxed values
            ranges.append(r_mm / 1000.0)
    if len(ranges) < 5:
        return None
    # Use 10th percentile for a stable wall estimate
    return float(np.percentile(ranges, 10))

# ╔══════════════════════════════════════════════════════════════════════╗
# ║        TURNING LIDAR HELPERS  (from turning_logic.py)               ║
# ╚══════════════════════════════════════════════════════════════════════╝

def wrap(d):
    return d % 360.0

def in_sector(angle, center, half):
    diff = (wrap(angle) - wrap(center) + 540.0) % 360.0 - 180.0
    return abs(diff) <= half

def sector_ranges(scan_mm, center, half):
    """Return all valid readings (m) within a sector."""
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

    A real intersection has:
      • turn-side   → mostly open (long ranges)
      • opposite    → wall at normal hallway distance (0.15 – 3.5 m)
      • front       → reasonably clear (> 0.8 m)
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

# ╔══════════════════════════════════════════════════════════════════════╗
# ║        FFT SCAN-MATCHING  (from turning_logic.py)                   ║
# ╚══════════════════════════════════════════════════════════════════════╝

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

# ╔══════════════════════════════════════════════════════════════════════╗
# ║        PIVOT FUNCTIONS  (from turning_logic.py)                     ║
# ╚══════════════════════════════════════════════════════════════════════╝

def pivot_left(speed=PIVOT_SPEED):
    """Left turn = LEFT wheel forward, RIGHT wheel slow/stopped."""
    _set(speed, speed * PIVOT_RATIO)

def pivot_right(speed=PIVOT_SPEED):
    """Right turn = RIGHT wheel forward, LEFT wheel slow/stopped."""
    _set(speed * PIVOT_RATIO, speed)

# ╔══════════════════════════════════════════════════════════════════════╗
# ║        CRUISE PD STEP  (extracted from cruise_control.py loop)      ║
# ╚══════════════════════════════════════════════════════════════════════╝

def cruise_step(scan, prev_error, dt, speed):
    """
    One iteration of PD wall-following (cruise_control.py logic).
    Returns the new error value and diagnostic info.
    """
    l_dist = get_sector_dist(scan, *CENTERING_LEFT)
    r_dist = get_sector_dist(scan, *CENTERING_RIGHT)

    valid_l = l_dist is not None and l_dist < GAP_THRESHOLD_M
    valid_r = r_dist is not None and r_dist < GAP_THRESHOLD_M

    # Error calculation (cruise_control.py lines 78-91)
    if valid_l and valid_r:
        error = l_dist - r_dist           # Both walls: stay centered
    elif valid_l:
        error = l_dist - SINGLE_WALL_TARGET_M  # Gap on right: follow left wall
    elif valid_r:
        error = SINGLE_WALL_TARGET_M - r_dist  # Gap on left: follow right wall
    else:
        error = 0.0                        # Both gaps: cruise straight

    # PD calculation (cruise_control.py lines 94-98)
    derivative = (error - prev_error) / dt
    steer = clamp((CRUISE_KP * error) + (CRUISE_KD * derivative), -0.03, 0.03)

    # Apply steering (cruise_control.py lines 101-103)
    l_speed = speed * (1.0 - max(0, -steer))
    r_speed = speed * (1.0 - max(0,  steer))
    drive_raw(l_speed, r_speed)

    return error, l_dist, r_dist, valid_l, valid_r, steer

# ╔══════════════════════════════════════════════════════════════════════╗
# ║        DIRECTION HELPERS                                            ║
# ╚══════════════════════════════════════════════════════════════════════╝

def get_turn_config(direction):
    """Return (turn_sector, opp_sector, pivot_fn, rotation_sign) for a direction."""
    if direction == "RIGHT":
        return RIGHT_SECTOR, LEFT_SECTOR, pivot_right, 1.0
    else:  # LEFT
        return LEFT_SECTOR, RIGHT_SECTOR, pivot_left, -1.0

# ╔══════════════════════════════════════════════════════════════════════╗
# ║        MAIN FSM — CRUISE → TURN → COOLDOWN → repeat for each leg   ║
# ╚══════════════════════════════════════════════════════════════════════╝

def run():
    print("\n=== DEMONSTRATION: LEFT → LEFT → RIGHT ===")
    for i, d in enumerate(ROUTE):
        print(f"  Maneuver {i+1}: {d}")
    print(f"\n  Cruise speed:        {CRUISE_SPEED}")
    print(f"  Target rotation:     {TARGET_ROTATION_DEG}° ± {ROTATION_TOLERANCE}°")
    print(f"  Creep into corner:   {CREEP_INTO_CORNER_S}s")
    print(f"  Post-turn settle:    {SETTLE_S}s")
    print(f"  Post-turn cooldown:  {CRUISE_COOLDOWN_S}s\n")

    # ── LiDAR startup ────────────────────────────────────────────────
    lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=2)
    lidar.start_motor()
    time.sleep(1.0)
    try:
        iterator = lidar.iter_scans(max_buf_meas=4000, min_len=5)
    except Exception:
        iterator = lidar.iter_scans()

    # ── FSM variables ─────────────────────────────────────────────────
    route_idx   = 0
    direction   = ROUTE[route_idx]
    turn_sector, opp_sector, do_pivot, rotation_sign = get_turn_config(direction)

    state        = "CRUISE"
    open_hits    = 0
    done_hits    = 0
    creep_start  = None
    turn_start   = None
    settle_start = None
    cooldown_start = None
    ref_norm     = None       # normalised reference scan captured at turn start
    prev_error   = 0.0
    last_time    = time.time()
    frame        = 0

    print(f"[DEMO] Leg 1/{len(ROUTE)} — cruising, watching for {direction} opening…\n")

    try:
        while True:
            # ── Read one LIDAR scan (blocking) ────────────────────────
            frame += 1
            scan = [DMAX] * SCAN_SIZE
            for _, theta, r in next(iterator):
                scan[int(theta) % SCAN_SIZE] = int(r)
            now = time.time()
            dt = max(now - last_time, 0.01)
            last_time = now

            # ─────────────────────────────────────────────────────────
            #  CRUISE — PD wall-following, watching for corner opening
            #          (cruise_control.py + turning_logic.py detection)
            # ─────────────────────────────────────────────────────────
            if state == "CRUISE":
                error, l_dist, r_dist, vl, vr, steer = cruise_step(
                    scan, prev_error, dt, CRUISE_SPEED)
                prev_error = error

                # Check for turn opening (turning_logic.py)
                valid, reason = is_real_intersection(scan, turn_sector, opp_sector)
                if valid:
                    open_hits += 1
                else:
                    open_hits = 0

                if open_hits >= CORNER_OPEN_DEBOUNCE:
                    print(f"[DEMO] Corner detected ({reason}) — slowing to approach…")
                    state = "APPROACH"
                    open_hits = 0

                # Periodic logging
                if frame % 10 == 0:
                    l_str = f"{l_dist:.2f}m" if vl else "GAP  "
                    r_str = f"{r_dist:.2f}m" if vr else "GAP  "
                    print(f"[CRUISE {route_idx+1}/{len(ROUTE)}] "
                          f"L: {l_str} | R: {r_str} | Steer: {steer:+.3f} | "
                          f"Next: {direction}")

            # ─────────────────────────────────────────────────────────
            #  APPROACH — creep speed PD following, re-validating corner
            # ─────────────────────────────────────────────────────────
            elif state == "APPROACH":
                error, l_dist, r_dist, vl, vr, steer = cruise_step(
                    scan, prev_error, dt, CREEP_SPEED)
                prev_error = error

                valid, reason = is_real_intersection(scan, turn_sector, opp_sector)
                if valid:
                    open_hits += 1
                else:
                    open_hits = 0

                if open_hits >= CORNER_OPEN_DEBOUNCE:
                    print(f"[DEMO] Intersection confirmed ({reason}) — creeping into mouth…")
                    state = "CREEP"
                    creep_start = now

                if frame % 10 == 0:
                    print(f"[APPROACH] hits: {open_hits} | {reason}")

            # ─────────────────────────────────────────────────────────
            #  CREEP — drive into the corner mouth before pivoting
            #          (turning_logic.py CREEP state)
            # ─────────────────────────────────────────────────────────
            elif state == "CREEP":
                drive(CREEP_SPEED)
                if now - creep_start >= CREEP_INTO_CORNER_S:
                    # Capture reference scan RIGHT NOW, before any rotation
                    ref_norm = normalise_scan(scan)
                    state = "TURNING"
                    turn_start = now
                    done_hits = 0
                    print(f"[DEMO] Reference scan captured. Pivoting {direction}…")

            # ─────────────────────────────────────────────────────────
            #  TURNING — FFT scan-matched pivot
            #            (turning_logic.py TURNING state — verbatim)
            # ─────────────────────────────────────────────────────────
            elif state == "TURNING":
                elapsed = now - turn_start

                # Measure cumulative rotation via scan matching
                cur_norm = normalise_scan(scan)
                raw_rotation = scan_rotation_deg(ref_norm, cur_norm)
                signed_rotation = raw_rotation * rotation_sign

                # Compute ramped pivot speed (deceleration near target)
                remaining = max(0.0, TARGET_ROTATION_DEG - signed_rotation)
                if remaining < RAMP_START_DEG and signed_rotation > 10:
                    t = 1.0 - (remaining / RAMP_START_DEG)
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
                err = abs(signed_rotation - TARGET_ROTATION_DEG)
                at_target = err <= ROTATION_TOLERANCE
                overshot  = signed_rotation > (TARGET_ROTATION_DEG + ROTATION_TOLERANCE)

                if at_target or overshot:
                    done_hits += 1
                else:
                    done_hits = 0

                if done_hits >= ROTATION_DONE_DEBOUNCE:
                    stop()
                    print(f"\n[DEMO] ✓ Turn {route_idx+1}/{len(ROUTE)} ({direction}) "
                          f"complete in {elapsed:.1f}s  "
                          f"(measured rotation: {signed_rotation:.1f}°)")
                    state = "SETTLE"
                    settle_start = now

                elif elapsed > TURN_TIMEOUT_S:
                    stop()
                    print(f"\n[DEMO] ✗ Turn {route_idx+1}/{len(ROUTE)} timed out "
                          f"after {TURN_TIMEOUT_S}s  "
                          f"(reached {signed_rotation:.1f}°) — settling anyway.")
                    state = "SETTLE"
                    settle_start = now

                elif frame % 3 == 0:
                    print(f"[TURNING] {elapsed:.1f}s  rotation: {signed_rotation:.1f}° / "
                          f"{TARGET_ROTATION_DEG}°  speed: {speed:.2f}  done_hits: {done_hits}")

            # ─────────────────────────────────────────────────────────
            #  SETTLE — blind cruise to clear the intersection
            # ─────────────────────────────────────────────────────────
            elif state == "SETTLE":
                drive(SETTLE_SPEED)
                if now - settle_start >= SETTLE_S:
                    route_idx += 1

                    # All maneuvers done?
                    if route_idx >= len(ROUTE):
                        print("\n[DEMO] All maneuvers complete! Entering final cruise.")
                        state = "FINAL_CRUISE"
                        prev_error = 0.0
                    else:
                        # Transition to cooldown cruise before looking for next corner
                        print(f"[DEMO] Settle done. Cruising for {CRUISE_COOLDOWN_S}s "
                              f"before watching for maneuver {route_idx+1}…\n")
                        state = "COOLDOWN"
                        cooldown_start = now
                        prev_error = 0.0

            # ─────────────────────────────────────────────────────────
            #  COOLDOWN — PD wall-following but NOT looking for corners
            #             Prevents immediate re-trigger after a turn
            # ─────────────────────────────────────────────────────────
            elif state == "COOLDOWN":
                error, l_dist, r_dist, vl, vr, steer = cruise_step(
                    scan, prev_error, dt, CRUISE_SPEED)
                prev_error = error

                elapsed = now - cooldown_start
                if elapsed >= CRUISE_COOLDOWN_S:
                    # Load next maneuver config
                    direction = ROUTE[route_idx]
                    turn_sector, opp_sector, do_pivot, rotation_sign = get_turn_config(direction)
                    open_hits = 0
                    print(f"[DEMO] Cooldown complete. Leg {route_idx+1}/{len(ROUTE)} — "
                          f"watching for {direction} opening…\n")
                    state = "CRUISE"

                if frame % 15 == 0:
                    l_str = f"{l_dist:.2f}m" if vl else "GAP  "
                    r_str = f"{r_dist:.2f}m" if vr else "GAP  "
                    print(f"[COOLDOWN] {elapsed:.1f}/{CRUISE_COOLDOWN_S}s | "
                          f"L: {l_str} | R: {r_str}")

            # ─────────────────────────────────────────────────────────
            #  FINAL_CRUISE — all turns done, PD cruise indefinitely
            #                 (Ctrl+C to stop)
            # ─────────────────────────────────────────────────────────
            elif state == "FINAL_CRUISE":
                error, l_dist, r_dist, vl, vr, steer = cruise_step(
                    scan, prev_error, dt, CRUISE_SPEED)
                prev_error = error

                if frame % 15 == 0:
                    l_str = f"{l_dist:.2f}m" if vl else "GAP  "
                    r_str = f"{r_dist:.2f}m" if vr else "GAP  "
                    print(f"[FINAL] L: {l_str} | R: {r_str} | Steer: {steer:+.3f}")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"\n[DEMO] Error: {e}")
        raise
    finally:
        stop()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("[DEMO] Motors off. LiDAR disconnected.")


if __name__ == "__main__":
    run()
