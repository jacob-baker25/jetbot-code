from rplidar import RPLidar, RPLidarException
from jetbot import Robot
import time
import math
import statistics
import numpy as np
from collections import deque
from threading import Thread, Lock


BEACONS = {
    "corner_1":    {"major": 1, "minor": 4949},
    "corner_2":    {"major": 2, "minor": 4949},
    "corner_3":    {"major": 4, "minor": 4949},
    "destination": {"major": 3, "minor": 4949},
}

ROUTE = [
    {"beacon": "corner_3",    "action": "LEFT",  "turn_duration_s": 4.00, "trigger_dist_m": 7.5},
    {"beacon": "corner_1",    "action": "RIGHT", "turn_duration_s": 4.00, "trigger_dist_m": 7.5},
    {"beacon": "corner_2",    "action": "RIGHT", "turn_duration_s": 4.00, "trigger_dist_m": 17.0},
    {"beacon": "destination", "action": "STOP",  "turn_duration_s": 0,    "trigger_dist_m": 12.0},
]

TRIGGER_DEBOUNCE_SEC = 0.60
PATH_LOSS_N   = 2.0
BLE_EMA_ALPHA = 0.25

# ------------------- BLE SCANNING & FILTERING -------------------
def parse_ibeacon(mfr_bytes):
    if len(mfr_bytes) < 23 or mfr_bytes[0] != 0x02 or mfr_bytes[1] != 0x15:
        return None
    major = int.from_bytes(mfr_bytes[18:20], "big")
    minor = int.from_bytes(mfr_bytes[20:22], "big")
    txp   = int.from_bytes(mfr_bytes[22:23], "big", signed=True)
    return major, minor, txp

class BLETracker:
    def __init__(self, known_beacons):
        self.known_beacons = known_beacons
        self.lock = Lock()
        self.raw_history = {name: deque(maxlen=5) for name in known_beacons}
        self.dist_ema  = {}
        self.last_seen = {}
        self._stop = False

    def start(self):
        self._stop = False
        Thread(target=self._run, daemon=True).start()

    def get_distance(self, name):
        with self.lock:
            if name not in self.dist_ema or (time.time() - self.last_seen.get(name, 0.0)) > 3.0:
                return None
            return self.dist_ema[name]

    def _run(self):
        from bleak import BleakScanner
        import asyncio, statistics

        def callback(device, adv_data):
            mfr = adv_data.manufacturer_data or {}
            if 0x004C not in mfr: return
            parsed = parse_ibeacon(mfr[0x004C])
            if not parsed: return
            major, minor, txp = parsed
            rssi = device.rssi
            matched = next((n for n, c in self.known_beacons.items()
                            if major == c["major"] and minor == c["minor"]), None)
            if not matched: return
            raw_d = 0.1 if rssi >= 0 else 10 ** ((txp - rssi) / (10.0 * PATH_LOSS_N))
            with self.lock:
                self.raw_history[matched].append(raw_d)
                filtered = statistics.median(self.raw_history[matched])
                prev = self.dist_ema.get(matched)
                self.dist_ema[matched] = filtered if prev is None else \
                    (1 - BLE_EMA_ALPHA) * prev + BLE_EMA_ALPHA * filtered
                self.last_seen[matched] = time.time()

        async def scan_loop():
            while not self._stop:
                scanner = BleakScanner(detection_callback=callback)
                await scanner.start()
                for _ in range(15):
                    if self._stop: break
                    await asyncio.sleep(0.1)
                await scanner.stop()

        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(scan_loop())
        except Exception as e:
            print(f"BLE Error: {e}")



# ------------------- NAV FSM -------------------
def run():
    ble = BLETracker(BEACONS)
    ble.start()

    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=2)
    lidar.start_motor()
    time.sleep(1.0)

    try:    iterator = lidar.iter_scans(max_buf_meas=2000, min_len=5)
    except: iterator = lidar.iter_scans()

    state      = "WAIT_FOR_BEACON"
    route_idx  = 0
    within_since, turn_start, post_turn_start, open_hits = None, None, None, 0
    frame      = 0

    # PD / wall-follow state
    prev_center_error = 0.0
    prev_frame_time   = time.time()
    preferred_wall    = None   # "LEFT" | "RIGHT" | None  — chosen dynamically

    active_sector = RIGHT_SECTOR if ROUTE[route_idx]["action"] == "RIGHT" else LEFT_SECTOR

    print("=== Four-Beacon Route Mode ===")
    for i, r in enumerate(ROUTE):
        b = BEACONS[r["beacon"]]
        print(f"  Waypoint {i+1}: Major={b['major']}, Minor={b['minor']} "
              f"→ {r['action']} at {r['trigger_dist_m']}m")
    print("Waiting to acquire first beacon signal before moving…")

    try:
            d = ble.get_distance(current_route["beacon"])

            # ============================================================
            if state == "WAIT_FOR_BEACON":
                stop()
                if d is not None:
                    print(f"\n[NAV] *** Signal acquired for '{current_route['beacon']}'! "
                          f"Distance: {d:.3f}m ***")
                    print("[NAV] Engaging motors → CRUISE.\n")
                    state = "CRUISE"



    except KeyboardInterrupt: print("Stopped by user.")
    except Exception as e:    print(f"Error: {e}")
    finally:
        stop(); ble._stop = True; lidar.stop(); lidar.stop_motor(); lidar.disconnect()

if __name__ == "__main__":
    run()