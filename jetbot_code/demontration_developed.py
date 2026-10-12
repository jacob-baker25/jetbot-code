"""
hallway_nav.py — Cruise a hallway (PD wall-centering), execute a sequence of
                  turns (left, left, right), then stop after the final turn.

BLE integration via separate process (ble_scanner.py):
  - This script writes scan requests to /tmp/jetbot_ble_request.json
  - ble_scanner.py reads requests, scans BLE, writes results to
    /tmp/jetbot_beacons.json
  - Corner detection is gated on the beacon being seen at least once
  - Zero GIL contention — BLE runs in a completely separate process

FSM states
----------
  CRUISE   – PD-centered forward drive; watching for the next turn's opening
             (ignores openings until BOTH cruise_delay elapsed AND beacon seen)
  CREEP    – Slow forward into the corner mouth, centering still active
  TURNING  – FFT scan-matching pivot; ramps down as target approaches
  DONE     – All turns complete, robot stopped

Usage:
    Terminal 1:  python3 ble_scanner.py
    Terminal 2:  python3 hallway_nav.py
"""

from rplidar import RPLidar, RPLidarException
from jetbot import Robot
import time
import json
import os
import numpy as np

# ── Hardware config (from cruise_test_1.py) ───────────────────────────────────
robot = Robot()
PORT_NAME  = '/dev/ttyUSB0'
BAUDRATE   = 256000
DMAX       = 12000
SCAN_SIZE  = 360

MOTOR_DIR_L, MOTOR_DIR_R = -1.0, -1.0
BIAS_L, BIAS_R           = 1.0, 1.0
BASE_SPEED               = 1.00

def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def drive(left, right):
    robot.set_motors(
        clamp(left  * MOTOR_DIR_L * BIAS_L),
        clamp(right * MOTOR_DIR_R * BIAS_R)
    )

# ── Turn motor helpers (from turn_test.py) ────────────────────────────────────
CREEP_SPEED = 0.40
PIVOT_SPEED = 0.30
PIVOT_RATIO = 0.0    # 0.0 = point-turn

def stop():
    drive(0, 0)

# Left turn  = LEFT wheel forward, RIGHT wheel slow/stopped
# Right turn = RIGHT wheel forward, LEFT wheel slow/stopped
def pivot_left(speed=PIVOT_SPEED):
    drive(speed, speed * PIVOT_RATIO)

def pivot_right(speed=PIVOT_SPEED):
    drive(speed * PIVOT_RATIO, speed)

# ── LIDAR centering (from cruise_test_1.py) ───────────────────────────────────
CENTERING_RIGHT      = (90.0,  40.0)
CENTERING_LEFT       = (270.0, 40.0)
GAP_THRESHOLD_M      = 1.8
SINGLE_WALL_TARGET_M = 1.25

KP = 0.12
KD = 0.08

def get_sector_dist(scan_data, center, half_width):
    ranges = []
    for deg in range(int(center - half_width), int(center + half_width)):
        r_mm = scan_data[deg % 360]
        if 150 < r_mm < DMAX:
            ranges.append(r_mm / 1000.0)
    if len(ranges) < 5:
        return None
    return float(np.percentile(ranges, 10))

# ── Corner detection (from turn_test.py) ──────────────────────────────────────
FRONT_SECTOR = (0.0,   20.0)
RIGHT_SECTOR = (60.0,  30.0)
LEFT_SECTOR  = (300.0, 30.0)

CORNER_OPEN_THRESH_M  = 2.0
CORNER_OPEN_FRACTION  = 0.65
CORNER_OPEN_DEBOUNCE  = 3

OPP_WALL_MIN_M    = 0.15
OPP_WALL_MAX_M    = 3.5
FRONT_CLEAR_MIN_M = 0.8

# ── BLE IPC shared file paths (must match ble_scanner.py) ─────────────────────
BLE_STATUS_FILE  = "/tmp/jetbot_beacons.json"
BLE_REQUEST_FILE = "/tmp/jetbot_ble_request.json"

CREEP_INTO_CORNER_S = 4.0

def sector_ranges(scan_mm, center, half):
    out = []
    for deg, r in enumerate(scan_mm):
        diff = (deg - center + 540.0) % 360.0 - 180.0
        if abs(diff) > half:
            continue
        if r <= 0:
            continue
        out.append(min(r, DMAX) / 1000.0)
    return out

def side_is_open(scan_mm, sector):
    r = sector_ranges(scan_mm, sector[0], sector[1])
    if len(r) < 6:
        return False
    far = sum(1 for v in r if v >= CORNER_OPEN_THRESH_M)
    return (far / len(r)) >= CORNER_OPEN_FRACTION

def is_real_intersection(scan_mm, turn_sector, opp_sector):
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

# ── Scan-matching (from turn_test.py) ─────────────────────────────────────────
TARGET_ROTATION_DEG    = 85.0
ROTATION_TOLERANCE     = 4.0
ROTATION_DONE_DEBOUNCE = 2
TURN_GRACE_S           = 0.8
TURN_TIMEOUT_S         = 10.0

RAMP_START_DEG  = 25.0
PIVOT_SPEED_MIN = 0.15

def normalise_scan(scan_mm):
    s = np.array(scan_mm, dtype=np.float64)
    s = np.clip(s, 0, DMAX)
    mu    = s.mean()
    sigma = s.std()
    if sigma < 1e-6:
        return s - mu
    return (s - mu) / sigma

def scan_rotation_deg(ref_norm, cur_norm, search_limit=120):
    n = len(ref_norm)
    corr = np.real(np.fft.ifft(
        np.fft.fft(ref_norm) * np.conj(np.fft.fft(cur_norm))
    ))
    pos  = list(range(0, min(search_limit + 1, n)))
    neg  = list(range(max(0, n - search_limit), n))
    best = max(pos + neg, key=lambda i: corr[i])
    shift = best if best <= n // 2 else best - n
    return float(shift)

# ── Turn plan ─────────────────────────────────────────────────────────────────
# Each entry: (label, turn_sector, opp_sector, rotation_sign, pivot_fn,
#              cruise_delay_s, beacon_name)
#   cruise_delay_s: minimum seconds to cruise before looking for this turn
#   beacon_name:    beacon that must be seen (via ble_scanner.py) before
#                   corner detection activates — None to skip BLE gate
TURN_PLAN = [
    ("LEFT  #1", LEFT_SECTOR,  RIGHT_SECTOR, -1.0, pivot_left,   5.0, "corner_1"),
    ("LEFT  #2", LEFT_SECTOR,  RIGHT_SECTOR, -1.0, pivot_left,   5.0, "corner_2"),
    ("RIGHT #3", RIGHT_SECTOR, LEFT_SECTOR,   1.0, pivot_right, 10.0, "corner_3"),
]

# ── BLE IPC helpers ───────────────────────────────────────────────────────────

def request_ble_scan(beacon_name):
    """Tell ble_scanner.py to start scanning for this beacon."""
    if beacon_name is None:
        return
    tmp = BLE_REQUEST_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"target": beacon_name, "scan": True}, f)
    os.replace(tmp, BLE_REQUEST_FILE)

def stop_ble_scan():
    """Tell ble_scanner.py to stop scanning."""
    tmp = BLE_REQUEST_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"scan": False}, f)
    os.replace(tmp, BLE_REQUEST_FILE)

def beacon_seen(beacon_name):
    """Check if ble_scanner.py has detected this beacon at least once."""
    if beacon_name is None:
        return True  # no beacon required for this turn
    try:
        with open(BLE_STATUS_FILE, "r") as f:
            status = json.load(f)
        return status.get(beacon_name, {}).get("seen", False)
    except (FileNotFoundError, json.JSONDecodeError):
        return False

# ── Main ──────────────────────────────────────────────────────────────────────
def run():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=2)
    lidar.start_motor()
    time.sleep(1.5)

    try:
        iterator = lidar.iter_scans(max_buf_meas=4000, min_len=5)
    except Exception:
        iterator = lidar.iter_scans()

    state       = "CRUISE"
    prev_error  = 0.0
    last_time   = time.time()
    frame       = 0

    turn_index  = 0
    open_hits   = 0
    creep_start = None
    ref_norm    = None
    turn_start  = None
    done_hits   = 0
    cruise_start = time.time()   # when the current CRUISE segment began

    label, turn_sector, opp_sector, rotation_sign, do_pivot, cruise_delay, \
        beacon_name = TURN_PLAN[turn_index]

    print("=== Hallway Nav: LEFT → LEFT → RIGHT then stop ===")
    print(f"Turn plan: {[t[0].strip() for t in TURN_PLAN]}")
    print(f"Target rotation per turn: {TARGET_ROTATION_DEG}° ± {ROTATION_TOLERANCE}°")
    print(f"NOTE: Requires ble_scanner.py running in a separate terminal.\n")

    # Request first beacon scan
    request_ble_scan(beacon_name)
    print(f"[NAV] Cruising — waiting for beacon '{beacon_name}' + "
          f"{cruise_delay:.0f}s blind period, then watching"
          f" for turn 1 ({label.strip()})...")

    try:
        while True:
            frame += 1

            scan = [DMAX] * SCAN_SIZE
            for _, theta, r in next(iterator):
                scan[int(theta) % SCAN_SIZE] = int(r)

            now = time.time()
            dt  = max(now - last_time, 0.01)
            last_time = now

            # ── CRUISE ───────────────────────────────────────────────
            if state == "CRUISE":
                l_dist = get_sector_dist(scan, *CENTERING_LEFT)
                r_dist = get_sector_dist(scan, *CENTERING_RIGHT)

                valid_l = l_dist is not None and l_dist < GAP_THRESHOLD_M
                valid_r = r_dist is not None and r_dist < GAP_THRESHOLD_M

                if valid_l and valid_r:
                    error = l_dist - r_dist
                elif valid_l:
                    error = l_dist - SINGLE_WALL_TARGET_M
                elif valid_r:
                    error = SINGLE_WALL_TARGET_M - r_dist
                else:
                    error = 0.0

                derivative = (error - prev_error) / dt
                prev_error = error
                steer = clamp(KP * error + KD * derivative, -0.03, 0.03)

                l_speed = BASE_SPEED * (1.0 - max(0,  -steer))
                r_speed = BASE_SPEED * (1.0 - max(0,   steer))
                drive(l_speed, r_speed)

                cruise_elapsed = now - cruise_start
                timer_ok = cruise_elapsed >= cruise_delay
                ble_ok   = beacon_seen(beacon_name)
                looking  = timer_ok and ble_ok

                if int(now * 10) % 2 == 0:
                    l_str = f"{l_dist:.2f}m" if valid_l else "GAP  "
                    r_str = f"{r_dist:.2f}m" if valid_r else "GAP  "
                    reason_str = ""
                    if not timer_ok:
                        reason_str = f"  [blind {cruise_elapsed:.1f}/{cruise_delay:.0f}s]"
                    elif not ble_ok:
                        reason_str = f"  [waiting for beacon '{beacon_name}']"
                    print(f"[CRUISE] L: {l_str} | R: {r_str} | Steer: {steer:+.3f}{reason_str}")

                if looking:
                    # Beacon confirmed — stop requesting BLE scans
                    stop_ble_scan()

                    valid, reason = is_real_intersection(scan, turn_sector, opp_sector)
                    open_hits = (open_hits + 1) if valid else 0

                    if open_hits >= CORNER_OPEN_DEBOUNCE:
                        state = "CREEP"
                        creep_start = now
                        open_hits = 0
                        print(f"\n[NAV] Intersection confirmed ({reason})"
                              f" — creeping to mouth for turn {turn_index + 1}"
                              f" ({label.strip()})...")
                else:
                    open_hits = 0

            # ── CREEP ─────────────────────────────────────────────────
            elif state == "CREEP":
                l_dist = get_sector_dist(scan, *CENTERING_LEFT)
                r_dist = get_sector_dist(scan, *CENTERING_RIGHT)

                valid_l = l_dist is not None and l_dist < GAP_THRESHOLD_M
                valid_r = r_dist is not None and r_dist < GAP_THRESHOLD_M

                if valid_l and valid_r:
                    error = l_dist - r_dist
                elif valid_l:
                    error = l_dist - SINGLE_WALL_TARGET_M
                elif valid_r:
                    error = SINGLE_WALL_TARGET_M - r_dist
                else:
                    error = 0.0

                derivative = (error - prev_error) / dt
                prev_error = error
                steer = clamp(KP * error + KD * derivative, -0.03, 0.03)

                l_speed = CREEP_SPEED * (1.0 - max(0,  -steer))
                r_speed = CREEP_SPEED * (1.0 - max(0,   steer))
                drive(l_speed, r_speed)

                elapsed = now - creep_start
                if frame % 5 == 0:
                    print(f"[CREEP] {elapsed:.1f}s / {CREEP_INTO_CORNER_S:.1f}s"
                          f" | Steer: {steer:+.3f}")

                if elapsed >= CREEP_INTO_CORNER_S:
                    ref_norm   = normalise_scan(scan)
                    state      = "TURNING"
                    turn_start = now
                    done_hits  = 0
                    print(f"\n[NAV] Reference scan captured."
                          f" Pivoting {label.strip()}...")

            # ── TURNING ───────────────────────────────────────────────
            elif state == "TURNING":
                elapsed = now - turn_start

                cur_norm        = normalise_scan(scan)
                raw_rotation    = scan_rotation_deg(ref_norm, cur_norm)
                signed_rotation = raw_rotation * rotation_sign

                remaining = max(0.0, TARGET_ROTATION_DEG - signed_rotation)
                if remaining < RAMP_START_DEG and signed_rotation > 10:
                    t     = 1.0 - (remaining / RAMP_START_DEG)
                    speed = PIVOT_SPEED - t * (PIVOT_SPEED - PIVOT_SPEED_MIN)
                else:
                    speed = PIVOT_SPEED

                do_pivot(speed)

                if elapsed < TURN_GRACE_S:
                    if frame % 5 == 0:
                        print(f"[TURNING] {elapsed:.1f}s  (grace)"
                              f"  rotation: {signed_rotation:.1f}°")
                    continue

                at_target = abs(signed_rotation - TARGET_ROTATION_DEG) <= ROTATION_TOLERANCE
                overshot  = signed_rotation > TARGET_ROTATION_DEG + ROTATION_TOLERANCE

                if at_target or overshot:
                    done_hits += 1
                else:
                    done_hits = 0

                if done_hits >= ROTATION_DONE_DEBOUNCE or elapsed > TURN_TIMEOUT_S:
                    stop()
                    if elapsed > TURN_TIMEOUT_S and done_hits < ROTATION_DONE_DEBOUNCE:
                        print(f"\n[NAV] ✗ Turn {turn_index + 1} timed out"
                              f" (reached {signed_rotation:.1f}°)")
                    else:
                        print(f"\n[NAV] ✓ Turn {turn_index + 1} ({label.strip()})"
                              f" complete in {elapsed:.1f}s"
                              f" (measured: {signed_rotation:.1f}°)")

                    turn_index += 1

                    if turn_index >= len(TURN_PLAN):
                        state = "DONE"
                        stop_ble_scan()
                        print("\n[NAV] All turns complete. Stopping.")
                    else:
                        label, turn_sector, opp_sector, rotation_sign, \
                            do_pivot, cruise_delay, beacon_name = \
                            TURN_PLAN[turn_index]
                        open_hits    = 0
                        prev_error   = 0.0
                        cruise_start = now   # reset blind timer for new segment
                        state        = "CRUISE"
                        # Request BLE scan for next beacon
                        request_ble_scan(beacon_name)
                        print(f"[NAV] Cruising — waiting for beacon "
                              f"'{beacon_name}' + {cruise_delay:.0f}s blind, "
                              f"then watching for turn {turn_index + 1} "
                              f"({label.strip()})...")

                elif frame % 3 == 0:
                    print(f"[TURNING] {elapsed:.1f}s"
                          f"  rotation: {signed_rotation:.1f}° / {TARGET_ROTATION_DEG}°"
                          f"  speed: {speed:.2f}  done_hits: {done_hits}")

            # ── DONE ──────────────────────────────────────────────────
            elif state == "DONE":
                stop()
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stop()
        stop_ble_scan()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == "__main__":
    run()