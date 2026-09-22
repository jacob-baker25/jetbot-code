from rplidar import RPLidar
from jetbot import Robot
import time
import numpy as np

# ------------------- HARDWARE CONFIG -------------------
robot = Robot()
PORT_NAME = '/dev/ttyUSB0' #
BAUDRATE = 256000          #
DMAX = 12000               #
SCAN_SIZE = 360            #

# Motor calibration
MOTOR_DIR_L, MOTOR_DIR_R = -1.0, -1.0
BIAS_L, BIAS_R = 1.0, 1.0
BASE_SPEED = 1.00

def clamp(v, lo=-1.0, hi=1.0): return max(lo, min(hi, v))

def drive(left, right):
    robot.set_motors(
        clamp(left * MOTOR_DIR_L * BIAS_L),
        clamp(right * MOTOR_DIR_R * BIAS_R)
    )

# ------------------- LIDAR HELPERS -------------------
# Looking at 40-degree windows to the sides
CENTERING_RIGHT = (90.0, 40.0)
CENTERING_LEFT  = (270.0, 40.0)
GAP_THRESHOLD_M = 1.8  # Ignore readings beyond this (likely a door/gap)
SINGLE_WALL_TARGET_M = 1.25  # Desired distance from a single wall

def get_sector_dist(scan_data, center, half_width):
    ranges = []
    for deg in range(int(center - half_width), int(center + half_width)):
        r_mm = scan_data[deg % 360]
        if 150 < r_mm < DMAX: # Filter out near-field noise and maxed values
            ranges.append(r_mm / 1000.0)

    if len(ranges) < 5: return None
    # Use 10th percentile for a stable wall estimate
    return float(np.percentile(ranges, 10))

# ------------------- CRUISE TEST -------------------
def run_cruise_test():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=2)
    lidar.start_motor()
    time.sleep(1.5)

    # PD Controller Constants
    KP = 0.12   # Sensitivity to error
    KD = 0.08   # "Braking" force to stop oscillation

    prev_error = 0.0
    last_time = time.time()

    print("Starting Cruise Test... Press Ctrl+C to stop.")

    try:
        for scan in lidar.iter_scans():
            scan_data = [DMAX] * SCAN_SIZE
            for _, theta, r in scan:
                scan_data[int(theta) % SCAN_SIZE] = int(r)

            now = time.time()
            dt = max(now - last_time, 0.01)
            last_time = now

            # 1. Acquire Wall Distances
            l_dist = get_sector_dist(scan_data, *CENTERING_LEFT)
            r_dist = get_sector_dist(scan_data, *CENTERING_RIGHT)

            # 2. Check for Gaps
            valid_l = l_dist is not None and l_dist < GAP_THRESHOLD_M
            valid_r = r_dist is not None and r_dist < GAP_THRESHOLD_M

            # 3. Calculate Error (Input for steering)
            if valid_l and valid_r:
                # Both walls seen: stay in the exact middle
                error = l_dist - r_dist
            elif valid_l:
                # Gap on right: follow the left wall at 1.25m
                # Positive error → steer left (toward wall), negative → steer right (away)
                error = l_dist - SINGLE_WALL_TARGET_M
            elif valid_r:
                # Gap on left: follow the right wall at 1.25m
                # Negative error → steer right (toward wall), positive → steer left (away)
                error = SINGLE_WALL_TARGET_M - r_dist
            else:
                # Both gaps: cruise straight until a wall reappears
                error = 0.0

            # 4. PD Calculation
            derivative = (error - prev_error) / dt
            prev_error = error

            steer = (KP * error) + (KD * derivative)
            steer = clamp(steer, -0.03, 0.03)

            # 5. Apply Steering — FIXED: positive steer turns left, negative turns right
            l_speed = BASE_SPEED * (1.0 - max(0,  -steer))
            r_speed = BASE_SPEED * (1.0 - max(0, steer))
            drive(l_speed, r_speed)

            # Logging for debugging
            if int(now * 10) % 2 == 0:
                l_str = f"{l_dist:.2f}m" if valid_l else "GAP  "
                r_str = f"{r_dist:.2f}m" if valid_r else "GAP  "
                print(f"L: {l_str} | R: {r_str} | Steer: {steer:+.2f}")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        drive(0, 0)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == "__main__":
    run_cruise_test()