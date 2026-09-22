"""
ble_scanner.py — Standalone BLE beacon scanner for JetBot navigation.

Normal mode (nav-driven):
    Runs as a separate process. Reads scan requests written by hallway_nav.py
    to /tmp/jetbot_ble_request.json and writes results to /tmp/jetbot_beacons.json.

    Usage:
        python3 ble_scanner.py

Manual mode (standalone testing):
    Prompts you to pick a beacon, scan all at once, reset, or print the JSON.
    No nav script needed.

    Usage:
        python3 ble_scanner.py --manual
"""

import sys
import time
import json
import os
import asyncio

# ── Beacon definitions (must match nav script) ───────────────────────────────
BEACONS = {
    "corner_1": {"major": 1, "minor": 4949},
    "corner_2": {"major": 5, "minor": 4949},
    "corner_3": {"major": 3, "minor": 4949},
}

# ── Shared file paths ────────────────────────────────────────────────────────
STATUS_FILE  = "/tmp/jetbot_beacons.json"     # this script WRITES
REQUEST_FILE = "/tmp/jetbot_ble_request.json"  # nav script WRITES, this script READS

# ── BLE config ────────────────────────────────────────────────────────────────
PATH_LOSS_N      = 2.0
SCAN_WINDOW_S    = 2.0    # how long each scan burst lasts
POLL_INTERVAL_S  = 0.5    # how often to check for new requests when idle

# ── iBeacon parser ────────────────────────────────────────────────────────────
def parse_ibeacon(mfr_bytes):
    if len(mfr_bytes) < 23 or mfr_bytes[0] != 0x02 or mfr_bytes[1] != 0x15:
        return None
    major = int.from_bytes(mfr_bytes[18:20], "big")
    minor = int.from_bytes(mfr_bytes[20:22], "big")
    txp   = int.from_bytes(mfr_bytes[22:23], "big", signed=True)
    return major, minor, txp

# ── File I/O helpers ──────────────────────────────────────────────────────────
def write_status(status):
    """Atomic write — write to temp file then rename to avoid partial reads."""
    tmp = STATUS_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(status, f, indent=2)
    os.replace(tmp, STATUS_FILE)

def read_request():
    """Read the nav script's current request. Returns dict or None."""
    try:
        with open(REQUEST_FILE, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None

def print_status(status):
    """Pretty-print the current beacon status table."""
    print(f"\n{'Beacon':<12} {'Seen':<6} {'Distance':>10}  {'Age (s)':>10}")
    print("─" * 44)
    now = time.time()
    for name, entry in status.items():
        seen_str = "YES" if entry["seen"] else "no"
        dist_str = f"{entry['distance']:.2f}m" if entry["distance"] is not None else "—"
        if entry["timestamp"] is not None:
            age_str = f"{now - entry['timestamp']:.1f}s"
        else:
            age_str = "—"
        print(f"  {name:<10} {seen_str:<6} {dist_str:>10}  {age_str:>10}")
    print()

# ── BLE scan burst ────────────────────────────────────────────────────────────
def scan_for_beacon(target_name, scan_duration=SCAN_WINDOW_S):
    """
    Run a single BLE scan burst for scan_duration seconds.
    Returns (found: bool, distance: float or None).
    """
    from bleak import BleakScanner

    cfg = BEACONS[target_name]
    target_major = cfg["major"]
    target_minor = cfg["minor"]

    result = {"found": False, "distance": None}

    def callback(device, adv_data):
        if result["found"]:
            return
        mfr = adv_data.manufacturer_data or {}
        if 0x004C not in mfr:
            return
        parsed = parse_ibeacon(mfr[0x004C])
        if not parsed:
            return
        major, minor, txp = parsed
        if major == target_major and minor == target_minor:
            raw_d = 0.1 if device.rssi >= 0 else \
                10 ** ((txp - device.rssi) / (10.0 * PATH_LOSS_N))
            result["found"] = True
            result["distance"] = round(raw_d, 2)

    async def do_scan():
        scanner = BleakScanner(detection_callback=callback)
        await scanner.start()
        deadline = time.time() + scan_duration
        while not result["found"] and time.time() < deadline:
            await asyncio.sleep(0.1)
        await scanner.stop()

    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(do_scan())
        loop.close()
    except Exception as e:
        print(f"[BLE] Scan error: {e}")

    return result["found"], result["distance"]

# ── Manual mode ───────────────────────────────────────────────────────────────
def run_manual():
    beacon_names = list(BEACONS.keys())

    # Load existing status file if present, otherwise start fresh
    try:
        with open(STATUS_FILE, "r") as f:
            status = json.load(f)
        # Ensure all known beacons are present
        for name in beacon_names:
            if name not in status:
                status[name] = {"seen": False, "distance": None, "timestamp": None}
        print(f"[MANUAL] Loaded existing status from {STATUS_FILE}")
    except (FileNotFoundError, json.JSONDecodeError):
        status = {name: {"seen": False, "distance": None, "timestamp": None}
                  for name in beacon_names}
        write_status(status)
        print(f"[MANUAL] Initialized fresh status at {STATUS_FILE}")

    print("\n=== BLE Scanner — MANUAL MODE ===")
    print(f"  Known beacons: {beacon_names}")
    print(f"  Status file:   {STATUS_FILE}")
    print(f"  Scan window:   {SCAN_WINDOW_S}s per burst\n")

    menu = (
        "Commands:\n"
        "  1..{n}  — scan a specific beacon\n"
        "  all     — scan all beacons one by one\n"
        "  status  — print current JSON status\n"
        "  reset   — mark all beacons unseen\n"
        "  quit    — exit\n"
    ).format(n=len(beacon_names))

    try:
        while True:
            print(menu)
            for i, name in enumerate(beacon_names, 1):
                tag = "✓" if status[name]["seen"] else " "
                print(f"  [{tag}] {i}. {name}  (major={BEACONS[name]['major']})")
            print()

            try:
                raw = input(">> ").strip().lower()
            except EOFError:
                break

            if raw in ("quit", "q", "exit"):
                break

            elif raw == "status":
                print_status(status)
                try:
                    with open(STATUS_FILE, "r") as f:
                        raw_json = f.read()
                    print(f"Raw JSON ({STATUS_FILE}):\n{raw_json}")
                except FileNotFoundError:
                    print("(status file not found)")

            elif raw == "reset":
                status = {name: {"seen": False, "distance": None, "timestamp": None}
                          for name in beacon_names}
                write_status(status)
                print("[MANUAL] All beacons reset to unseen.\n")

            elif raw == "all":
                for name in beacon_names:
                    print(f"[MANUAL] Scanning for '{name}'...", end=" ", flush=True)
                    found, distance = scan_for_beacon(name)
                    if found:
                        status[name]["seen"] = True
                        status[name]["distance"] = distance
                        status[name]["timestamp"] = time.time()
                        write_status(status)
                        print(f"FOUND at {distance:.2f}m")
                    else:
                        print("not found")
                print_status(status)

            elif raw.isdigit() and 1 <= int(raw) <= len(beacon_names):
                name = beacon_names[int(raw) - 1]
                print(f"[MANUAL] Scanning for '{name}'...", end=" ", flush=True)
                found, distance = scan_for_beacon(name)
                if found:
                    status[name]["seen"] = True
                    status[name]["distance"] = distance
                    status[name]["timestamp"] = time.time()
                    write_status(status)
                    print(f"FOUND at {distance:.2f}m")
                else:
                    print("not found")
                print_status(status)

            else:
                print(f"  Unknown command: '{raw}'\n")

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[MANUAL] Exiting. Status file left intact at", STATUS_FILE)

# ── Nav-driven mode ───────────────────────────────────────────────────────────
def run():
    status = {name: {"seen": False, "distance": None, "timestamp": None}
              for name in BEACONS}
    write_status(status)

    print("=== BLE Scanner Started ===")
    print(f"  Status file:  {STATUS_FILE}")
    print(f"  Request file: {REQUEST_FILE}")
    print(f"  Known beacons: {list(BEACONS.keys())}")
    print(f"  Scan window: {SCAN_WINDOW_S}s per burst")
    print("\n[BLE] Waiting for scan requests from nav script…\n")

    try:
        while True:
            req = read_request()

            if req is None or not req.get("scan", False):
                time.sleep(POLL_INTERVAL_S)
                continue

            target = req.get("target")
            if target not in BEACONS:
                print(f"[BLE] Unknown beacon '{target}' in request. Ignoring.")
                time.sleep(POLL_INTERVAL_S)
                continue

            if status[target]["seen"]:
                print(f"[BLE] '{target}' already confirmed. Skipping.")
                time.sleep(POLL_INTERVAL_S)
                continue

            print(f"[BLE] Scanning for '{target}'…", end=" ", flush=True)
            found, distance = scan_for_beacon(target)

            if found:
                status[target]["seen"] = True
                status[target]["distance"] = distance
                status[target]["timestamp"] = time.time()
                write_status(status)
                print(f"✓ FOUND at {distance:.1f}m")
            else:
                print(f"✗ not found (retrying next cycle)")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[BLE] Scanner stopped.")
    finally:
        for f in [STATUS_FILE, REQUEST_FILE]:
            try:
                os.remove(f)
            except FileNotFoundError:
                pass
        print("[BLE] Cleaned up. Exiting.")


if __name__ == "__main__":
    if "--manual" in sys.argv:
        run_manual()
    else:
        run()
