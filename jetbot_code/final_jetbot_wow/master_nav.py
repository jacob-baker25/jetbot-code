"""
master_nav.py  –  Master Autonomous Navigation for JetBot (FFT Scan-Matching)
=============================================================================
Combines BLE beacon detection, LiDAR-based PD cruise control, and
FFT-based scan matching for precise 90-degree turns.

BLE Strategy — "One-Shot Burst"
-------------------------------
Instead of a continuous BLE thread (which starves the LiDAR serial buffer
via GIL contention), BLE runs in SHORT bursts (~2 s) with cooldown gaps
in between.  Once a beacon is detected, BLE stops entirely for the rest
of that leg.  The robot never stops moving — it cruises on LiDAR PD the
whole time.

    SCAN_CRUISE  →  beacon found?  YES → CRUISE (corner detection, no BLE)
                                   NO  → cruise BLE-free for cooldown, retry

Usage:
    python3 master_nav.py
"""

import time
import math
import statistics
import numpy as np
from collections import deque
from threading import Thread, Lock, Event

from rplidar import RPLidar, RPLidarException
from jetbot import Robot

# ╔══════════════════════════════════════════════════════════════════╗
# ║              ★  MISSION CONFIGURATION  ★                        ║
# ╚══════════════════════════════════════════════════════════════════╝

# Physical BLE beacons (Major/Minor)
BEACONS = {
    "corner_1": {"major": 1, "minor": 4949},
    "corner_2": {"major": 4, "minor": 4949},
    "corner_3": {"major": 3, "minor": 4949},
}

# Route Definition
# action: "LEFT", "RIGHT", "STRAIGHT", "STOP"
# trigger_dist_m: distance to beacon to start looking for corner
ROUTE = [
    {"beacon": "corner_1", "action": "LEFT",  "trigger_dist_m": 20.0},
    {"beacon": "corner_2", "action": "LEFT",  "trigger_dist_m": 20.0},
    {"beacon": "corner_3", "action": "RIGHT", "trigger_dist_m": 20.0},
]

# ╔══════════════════════════════════════════════════════════════════╗
# ║              HARDWARE & SPEED CONFIG                             ║
# ╚══════════════════════════════════════════════════════════════════╝

PORT      = '/dev/ttyUSB0'
BAUDRATE  = 256000
DMAX      = 12000
SCAN_SIZE = 360

MOTOR_DIR_L, MOTOR_DIR_R = -1.0, -1.0
BIAS_L, BIAS_R           = 1.00, 1.00

# Speeds
CRUISE_SPEED    = 1.0
CREEP_SPEED     = 0.85
PIVOT_SPEED     = 0.30   # matches working hallway_turn.py
PIVOT_SPEED_MIN = 0.15
PIVOT_RATIO     = 0.0    # 0.0 = point-turn
SETTLE_SPEED    = 0.85

# ╔══════════════════════════════════════════════════════════════════╗
# ║              LIDAR SECTORS & THRESHOLDS                          ║
# ╚══════════════════════════════════════════════════════════════════╝

# PD Cruise Sectors (matching cruise_control.py — 90° = right side, 270° = left side)
CENTERING_RIGHT = (90.0,  40.0)
CENTERING_LEFT  = (270.0, 40.0)

# Corner Detection Sectors (matching turning_logic.py exactly)
FRONT_SECTOR  = (0.0,   20.0)
RIGHT_SECTOR  = (60.0,  30.0)
LEFT_SECTOR   = (300.0, 30.0)

# Cruise PD thresholds
GAP_THRESHOLD_M      = 1.8
SINGLE_WALL_TARGET_M = 1.25
CRUISE_KP            = 0.12
CRUISE_KD            = 0.08

# Intersection Validation (matching turning_logic.py exactly)
CORNER_OPEN_THRESH_M = 2.0
CORNER_OPEN_FRACTION = 0.65
CORNER_OPEN_DEBOUNCE = 3
OPP_WALL_MIN_M       = 0.15
OPP_WALL_MAX_M       = 3.5
FRONT_CLEAR_MIN_M    = 0.8

# Turning Params (matching turning_logic.py exactly)
TARGET_ROTATION_DEG    = 85.0
ROTATION_TOLERANCE     = 4.0
ROTATION_DONE_DEBOUNCE = 2
RAMP_START_DEG         = 25.0
TURN_GRACE_S           = 0.8
TURN_TIMEOUT_S         = 10.0
CREEP_INTO_CORNER_S    = 4.0
SETTLE_S               = 1.5

# ╔══════════════════════════════════════════════════════════════════╗
# ║              BLE CONFIG                                          ║
# ╚══════════════════════════════════════════════════════════════════╝

PATH_LOSS_N        = 2.0
BLE_SCAN_DURATION  = 2.0    # seconds per burst
BLE_COOLDOWN_S     = 5.0    # cruise BLE-free between failed bursts

# ══════════════════════════════════════════════════════════════════════════════
#  ROBOT & DRIVE  (identical to hallway_turn.py)
# ══════════════════════════════════════════════════════════════════════════════
robot = Robot()

def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def _set(l, r):
    robot.set_motors(clamp(l * MOTOR_DIR_L * BIAS_L),
                     clamp(r * MOTOR_DIR_R * BIAS_R))

def stop():          _set(0, 0)
def drive(s):        _set(s, s)
def drive_raw(l, r): _set(l, r)

# Pivot functions — identical to hallway_turn.py
def pivot_right(speed=PIVOT_SPEED):  _set(speed * PIVOT_RATIO, speed)
def pivot_left(speed=PIVOT_SPEED):   _set(speed, speed * PIVOT_RATIO)

# ══════════════════════════════════════════════════════════════════════════════
#  LIDAR HELPERS  (identical to hallway_turn.py)
# ══════════════════════════════════════════════════════════════════════════════

def wrap(d):
    return d % 360.0

def in_sector(angle, center, half):
    diff = (wrap(angle) - wrap(center) + 540.0) % 360.0 - 180.0
    return abs(diff) <= half

def sector_ranges(scan_mm, center, half):
    """Identical to hallway_turn.py — includes all readings 0 < r <= DMAX."""
    out = []
    for deg, r in enumerate(scan_mm):
        if not in_sector(deg, center, half):
            continue
        if r <= 0:
            continue
        out.append(min(r, DMAX) / 1000.0)
    return out

def get_sector_dist(scan_mm, center, half):
    """10th-percentile wall estimate (matching cruise_control.py with 150mm near-field filter)."""
    ranges = []
    for deg in range(int(center - half), int(center + half)):
        r_mm = scan_mm[deg % SCAN_SIZE]
        if 150 < r_mm < DMAX:
            ranges.append(r_mm / 1000.0)
    if len(ranges) < 5:
        return None
    return float(np.percentile(ranges, 10))

def side_is_open(scan_mm, sector):
    """True if most of the sector sees far-away (no wall). Identical to hallway_turn.py."""
    r = sector_ranges(scan_mm, sector[0], sector[1])
    if len(r) < 6:
        return False
    far = sum(1 for v in r if v >= CORNER_OPEN_THRESH_M)
    return (far / len(r)) >= CORNER_OPEN_FRACTION

def is_real_intersection(scan_mm, turn_sector, opp_sector):
    """Identical to hallway_turn.py — takes explicit sector tuples."""
    if not side_is_open(scan_mm, turn_sector):
        return False, "turn-side not open"

    opp = sector_ranges(scan_mm, opp_sector[0], opp_sector[1])
    if len(opp) < 5:
        return False, "opposite-side no data"
    opp_p10 = float(np.percentile(opp, 10))
    if opp_p10 < OPP_WALL_MIN_M:
        return False, f"opposite too close ({opp_p10:.2f}m)"
    if opp_p10 > OPP_WALL_MAX_M:
        return False, f"opposite too far ({opp_p10:.2f}m)"

    fwd = sector_ranges(scan_mm, FRONT_SECTOR[0], FRONT_SECTOR[1])
    if len(fwd) < 5:
        return False, "front no data"
    fwd_p10 = float(np.percentile(fwd, 10))
    if fwd_p10 < FRONT_CLEAR_MIN_M:
        return False, f"front blocked ({fwd_p10:.2f}m)"

    return True, f"VALID (opp={opp_p10:.2f}m, fwd={fwd_p10:.2f}m)"

# ══════════════════════════════════════════════════════════════════════════════
#  FFT SCAN MATCHING  (identical to hallway_turn.py)
# ══════════════════════════════════════════════════════════════════════════════

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
    corr = np.real(np.fft.ifft(np.fft.fft(ref_norm) * np.conj(np.fft.fft(cur_norm))))

    pos = list(range(0, min(search_limit + 1, n)))
    neg = list(range(max(0, n - search_limit), n))
    candidates = pos + neg

    best_idx = max(candidates, key=lambda i: corr[i])
    shift = best_idx if best_idx <= n // 2 else best_idx - n

    return float(shift)

# ══════════════════════════════════════════════════════════════════════════════
#  BLE ONE-SHOT BURST SCANNER
# ══════════════════════════════════════════════════════════════════════════════
#
#  Fires a single ~2 s BLE scan in a background thread.
#  The thread auto-terminates after the scan window.
#  Call .start(target_name) to kick off a burst.
#  Poll .found / .busy from the main loop — no locks needed for booleans.
#  Once .found is True, the beacon was seen and BLE is already dead.
#

def parse_ibeacon(mfr_bytes):
    if len(mfr_bytes) < 23 or mfr_bytes[0] != 0x02 or mfr_bytes[1] != 0x15:
        return None
    major = int.from_bytes(mfr_bytes[18:20], "big")
    minor = int.from_bytes(mfr_bytes[20:22], "big")
    txp   = int.from_bytes(mfr_bytes[22:23], "big", signed=True)
    return major, minor, txp


class BLEBurst:
    """
    One-shot BLE scanner.  Runs for BLE_SCAN_DURATION seconds in a
    daemon thread, then stops.  Checks for a single target beacon.
    """

    def __init__(self, known_beacons):
        self.known_beacons = known_beacons
        self.found = False          # True once target seen
        self.distance = None        # estimated distance (m) when found
        self.busy = False           # True while scan is running
        self._target_major = None
        self._target_minor = None
        self._found_event = Event()

    def start(self, target_name):
        """Kick off a background scan burst for the named beacon."""
        cfg = self.known_beacons[target_name]
        self._target_major = cfg["major"]
        self._target_minor = cfg["minor"]
        self.found = False
        self.distance = None
        self.busy = True
        self._found_event.clear()
        Thread(target=self._run, daemon=True).start()

    def _run(self):
        from bleak import BleakScanner
        import asyncio

        def callback(device, adv_data):
            if self.found:
                return
            mfr = adv_data.manufacturer_data or {}
            if 0x004C not in mfr:
                return
            parsed = parse_ibeacon(mfr[0x004C])
            if not parsed:
                return
            major, minor, txp = parsed
            if major == self._target_major and minor == self._target_minor:
                raw_d = 0.1 if device.rssi >= 0 else \
                    10 ** ((txp - device.rssi) / (10.0 * PATH_LOSS_N))
                self.distance = raw_d
                self.found = True
                self._found_event.set()

        async def scan():
            scanner = BleakScanner(detection_callback=callback)
            await scanner.start()
            # Wait up to BLE_SCAN_DURATION, but bail early if found
            self._found_event.wait(timeout=BLE_SCAN_DURATION)
            await scanner.stop()

        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(scan())
        except Exception as e:
            print(f"[BLE] Burst error: {e}")
        finally:
            self.busy = False

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN FSM
# ══════════════════════════════════════════════════════════════════════════════
#
#  States:
#    SCAN_CRUISE   — PD cruise + BLE burst running.  Waiting for beacon.
#    SCAN_COOLDOWN — PD cruise, NO BLE.  Brief gap before next burst.
#    CRUISE        — PD cruise + corner detection.  Beacon already confirmed.
#    CREEP         — Slow into corner mouth.
#    TURNING       — FFT pivot.
#    SETTLING      — Blind cruise to clear intersection.
#

def run():
    print("\n=== JetBot Master Navigation (One-Shot BLE) ===")
    for i, wp in enumerate(ROUTE):
        b = BEACONS[wp["beacon"]]
        print(f"  Waypoint {i+1}: '{wp['beacon']}' (Major={b['major']}) → {wp['action']}")
    print()

    # ── LiDAR startup (direct, blocking — NOT threaded) ───────────────
    lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=2)
    lidar.start_motor()
    time.sleep(1.0)
    try:
        iterator = lidar.iter_scans(max_buf_meas=4000, min_len=5)
    except Exception:
        try:
            iterator = lidar.iter_scans(max_buf_meas=4000)
        except Exception:
            iterator = lidar.iter_scans()

    # ── BLE burst scanner ─────────────────────────────────────────────
    ble = BLEBurst(BEACONS)

    # ── FSM variables ─────────────────────────────────────────────────
    state       = "SCAN_CRUISE"
    route_idx   = 0
    open_hits   = 0
    done_hits   = 0
    creep_start = None
    turn_start  = None
    settle_start = None
    cooldown_start = None
    ref_norm    = None
    prev_error  = 0.0
    prev_time   = time.time()
    frame       = 0

    # Kick off first BLE burst
    wp = ROUTE[route_idx]
    ble.start(wp["beacon"])
    print(f"[NAV] Scanning for beacon '{wp['beacon']}' (burst mode)…")

    try:
        while route_idx < len(ROUTE):
            # ── Read one LiDAR scan (BLOCKING) ──
            frame += 1
            scan = [DMAX] * SCAN_SIZE
            for _, theta, r in next(iterator):
                scan[int(theta) % SCAN_SIZE] = int(r)
            now = time.time()
            dt = max(now - prev_time, 0.01)
            prev_time = now

            wp = ROUTE[route_idx]

            # Precompute turn config
            if wp["action"] == "RIGHT":
                turn_sector, opp_sector = RIGHT_SECTOR, LEFT_SECTOR
                do_pivot, rotation_sign = pivot_right, 1.0
            else:
                turn_sector, opp_sector = LEFT_SECTOR, RIGHT_SECTOR
                do_pivot, rotation_sign = pivot_left, -1.0

            # ── PD cruise helper (inline) ─────────────────────────────
            def do_cruise(speed):
                nonlocal prev_error
                l_dist = get_sector_dist(scan, *CENTERING_LEFT)
                r_dist = get_sector_dist(scan, *CENTERING_RIGHT)
                vl = l_dist is not None and l_dist < GAP_THRESHOLD_M
                vr = r_dist is not None and r_dist < GAP_THRESHOLD_M
                if vl and vr:       error = l_dist - r_dist
                elif vl:            error = l_dist - SINGLE_WALL_TARGET_M
                elif vr:            error = SINGLE_WALL_TARGET_M - r_dist
                else:               error = 0.0
                steer = clamp(CRUISE_KP * error + CRUISE_KD * (error - prev_error) / dt,
                              -0.03, 0.03)
                prev_error = error
                drive_raw(speed * (1.0 - max(0, -steer)),
                          speed * (1.0 - max(0,  steer)))
                return error, l_dist, r_dist, vl, vr, steer

            # ─────────────────────────────────────────────────────────
            #  SCAN_CRUISE — PD cruise while BLE burst is active
            #  Robot keeps moving.  Once beacon found → CRUISE.
            #  If burst finishes without finding → SCAN_COOLDOWN.
            # ─────────────────────────────────────────────────────────
            if state == "SCAN_CRUISE":
                do_cruise(CRUISE_SPEED)

                if ble.found:
                    print(f"[NAV] ✓ Beacon '{wp['beacon']}' detected "
                          f"({ble.distance:.1f}m). Corner search active.")
                    state = "CRUISE"
                    open_hits = 0
                elif not ble.busy:
                    # Burst ended, beacon not found — cooldown then retry
                    cooldown_start = now
                    state = "SCAN_COOLDOWN"
                    if frame % 15 == 0:
                        print(f"[NAV] Burst miss. Cruising BLE-free for "
                              f"{BLE_COOLDOWN_S}s…")

                if frame % 20 == 0:
                    print(f"[SCAN_CRUISE] Scanning for '{wp['beacon']}'… "
                          f"(BLE {'active' if ble.busy else 'done'})")

            # ─────────────────────────────────────────────────────────
            #  SCAN_COOLDOWN — PD cruise, NO BLE running at all
            #  Pure LiDAR, zero GIL contention.  After cooldown → retry.
            # ─────────────────────────────────────────────────────────
            elif state == "SCAN_COOLDOWN":
                do_cruise(CRUISE_SPEED)

                if now - cooldown_start >= BLE_COOLDOWN_S:
                    ble.start(wp["beacon"])
                    state = "SCAN_CRUISE"
                    print(f"[NAV] Retrying BLE burst for '{wp['beacon']}'…")

                if frame % 20 == 0:
                    elapsed = now - cooldown_start
                    print(f"[COOLDOWN] {elapsed:.1f}/{BLE_COOLDOWN_S}s — "
                          f"no BLE, pure LiDAR cruise")

            # ─────────────────────────────────────────────────────────
            #  CRUISE — beacon confirmed, PD cruise + corner detection
            #           NO BLE running.
            # ─────────────────────────────────────────────────────────
            elif state == "CRUISE":
                error, l_d, r_d, vl, vr, steer = do_cruise(CRUISE_SPEED)

                if wp["action"] == "STOP":
                    print("[NAV] Destination reached. Stopping.")
                    break

                if wp["action"] == "STRAIGHT":
                    v = side_is_open(scan, turn_sector)
                    open_hits = open_hits + 1 if v else 0
                else:
                    v, reason = is_real_intersection(scan, turn_sector, opp_sector)
                    open_hits = open_hits + 1 if v else 0

                if open_hits >= CORNER_OPEN_DEBOUNCE:
                    print(f"[NAV] Corner confirmed! Creeping into mouth…")
                    state = "CREEP"
                    creep_start = now

                if frame % 15 == 0:
                    print(f"[CRUISE] hits: {open_hits} | error: {error:+.2f}")

            # ─────────────────────────────────────────────────────────
            #  CREEP — drive into corner mouth (identical to hallway_turn.py)
            # ─────────────────────────────────────────────────────────
            elif state == "CREEP":
                drive(CREEP_SPEED)
                if now - creep_start >= CREEP_INTO_CORNER_S:
                    if wp["action"] == "STRAIGHT":
                        print("[NAV] Passed through straight. Next segment.")
                        route_idx += 1
                        prev_error = 0.0
                        if route_idx < len(ROUTE):
                            ble.start(ROUTE[route_idx]["beacon"])
                            state = "SCAN_CRUISE"
                            print(f"[NAV] Scanning for next beacon "
                                  f"'{ROUTE[route_idx]['beacon']}'…")
                        else:
                            break
                    else:
                        ref_norm = normalise_scan(scan)
                        state = "TURNING"
                        turn_start = now
                        done_hits = 0
                        print(f"[NAV] Reference scan captured. "
                              f"Pivoting {wp['action']}…")

            # ─────────────────────────────────────────────────────────
            #  TURNING — FFT scan-matched pivot (identical to hallway_turn.py)
            # ─────────────────────────────────────────────────────────
            elif state == "TURNING":
                elapsed = now - turn_start

                cur_norm = normalise_scan(scan)
                raw_rotation = scan_rotation_deg(ref_norm, cur_norm)
                signed_rotation = raw_rotation * rotation_sign

                remaining = max(0.0, TARGET_ROTATION_DEG - signed_rotation)
                if remaining < RAMP_START_DEG and signed_rotation > 10:
                    t = 1.0 - (remaining / RAMP_START_DEG)
                    speed = PIVOT_SPEED - t * (PIVOT_SPEED - PIVOT_SPEED_MIN)
                else:
                    speed = PIVOT_SPEED
                do_pivot(speed)

                if elapsed < TURN_GRACE_S:
                    if frame % 5 == 0:
                        print(f"[TURNING] {elapsed:.1f}s  (grace)  "
                              f"rotation: {signed_rotation:.1f}°")
                    continue

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
                    state = "SETTLING"
                    settle_start = now

                elif elapsed > TURN_TIMEOUT_S:
                    stop()
                    print(f"\n[NAV] ✗ Turn timed out after {TURN_TIMEOUT_S}s  "
                          f"(reached {signed_rotation:.1f}°) — settling anyway.")
                    state = "SETTLING"
                    settle_start = now

                elif frame % 3 == 0:
                    print(f"[TURNING] {elapsed:.1f}s  rotation: "
                          f"{signed_rotation:.1f}° / {TARGET_ROTATION_DEG}°  "
                          f"speed: {speed:.2f}  done_hits: {done_hits}")

            # ─────────────────────────────────────────────────────────
            #  SETTLING — blind cruise to clear intersection
            # ─────────────────────────────────────────────────────────
            elif state == "SETTLING":
                drive(SETTLE_SPEED)
                if now - settle_start >= SETTLE_S:
                    route_idx += 1
                    if route_idx >= len(ROUTE):
                        print("[NAV] Final waypoint cleared. Mission complete!")
                        break
                    # Start scanning for next beacon
                    next_wp = ROUTE[route_idx]
                    ble.start(next_wp["beacon"])
                    prev_error = 0.0
                    open_hits = 0
                    state = "SCAN_CRUISE"
                    print(f"\n[NAV] Segment done. Scanning for "
                          f"'{next_wp['beacon']}' → {next_wp['action']}\n")

        print("\n=== MISSION COMPLETE ===")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"\n[NAV] Error: {e}")
        raise
    finally:
        stop()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("[NAV] Motors off. LiDAR disconnected.")


if __name__ == "__main__":
    run()
