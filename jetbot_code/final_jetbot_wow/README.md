# AutoChair — JetBot Autonomous Navigation Codebase

**AutoChair** is a capstone project from the University of Delaware ECE Department (Mahesh, Macry, Tomlin, Turner) that develops an autonomous navigation system for powered wheelchairs, with the ultimate goal of serving residents at the Mary Campbell Center who lack the physical ability to navigate independently.

Development follows a **two-track strategy**: a JetBot (NVIDIA Jetson Nano) serves as the software testbed in Evans Hall, and a restored full-size wheelchair is the final hardware target. This repository contains all JetBot navigation software.

---

## What the Robot Does

The JetBot navigates hallways autonomously using two sensor inputs:

1. **2D LiDAR** (RPLidar, `/dev/ttyUSB0`) — keeps the robot centered between walls (PD control) and detects intersections. At each turn, an FFT-based scan-matching algorithm measures how many degrees the robot has rotated to achieve a precise 90° pivot without any encoders or gyroscope.

2. **BLE iBeacons** — **Blue Charm BC021** beacons are physically mounted at hallway intersections in Evans Hall. The robot scans for a specific beacon before it is willing to act on a detected corner opening. This prevents the robot from turning at the wrong intersection (e.g., a doorway that looks geometrically like a corner). The beacons are configured and monitored using the **KBeacon Pro** app (iOS/Android).

A **Telegram bot** (`corner_mapping.py`) acts as the user interface: a user sends `/route 1 6` from a phone and the bot computes the shortest path through a graph of Evans Hall, determines Left/Right at each turn node, and writes a route file that the nav script reads at startup.

---

## Evans Hall Map

```
Node 1 (Top-Left)  ──230──  Node 2 (Mid-Left)  ──234──  Node 3 (Bot-Left)
                                     │
                                    575
                                     │
Node 4 (Top-Right) ──230──  Node 5 (Mid-Right) ──234──  Node 6 (Bot-Right)
```

Edge weights are in arbitrary pixel units derived from a coordinate map of Evans. Shortest paths use Dijkstra's algorithm.

**Beacon assignments** (what's physically in Evans Hall):

| Node | Beacon Name | iBeacon Major | iBeacon Minor |
|------|-------------|---------------|---------------|
| 1    | `corner_1`  | 1             | 4949          |
| 5    | `corner_2`  | 5             | 4949          |
| 3    | `corner_3`  | 3             | 4949          |

> Note: `master_nav.py` and `beacon_reference.py` have slightly different major numbers — they represent earlier beacon assignments. The values in `ble_scanner.py` and `demontration_developed.py` are the most recent.

---

## File Map

| File | Role | Run directly? |
|------|------|---------------|
| `cruise_control.py` | PD wall-centering cruise only — no turns, no BLE. Useful for tuning `KP`/`KD` and validating LiDAR mount angle. | Yes |
| `turning_logic.py` | Single-turn test: drive straight → detect one corner → execute one 90° pivot → stop. Takes `left` or `right` as a CLI argument. | Yes |
| `ble_scanner.py` | Standalone BLE beacon scanner. Runs as a **separate process**. Communicates with nav scripts through two JSON files in `/tmp/`. Also has a `--manual` mode for bench-testing beacons without the robot moving. | Yes (two modes) |
| `demonstration.py` | Multi-turn LiDAR-only run (LEFT → LEFT → RIGHT). No BLE required. Good smoke-test for the full turn sequence in an environment where beacons aren't deployed. | Yes |
| `hallway_nav.py` | **Primary nav script.** Reads a route from `/tmp/jetbot_route.json` (written by `corner_mapping.py`) and executes it with BLE-gated corners. Requires `ble_scanner.py` in a second terminal. Falls back to a cruise-only mode if no route file is present. | Yes (needs companion processes) |
| `demontration_developed.py` | Earlier version of `hallway_nav.py` with the turn plan hardcoded (LEFT #1 / LEFT #2 / RIGHT #3 with beacons) instead of read from a file. Useful as a reference for the hardcoded-route pattern. | Yes |
| `corner_mapping.py` | Telegram bot + graph router. Receives `/route <start> <goal>` commands, computes a path, and writes `/tmp/jetbot_route.json` for `hallway_nav.py` to consume. Also runs Dijkstra's and `turn_direction()` logic. **Requires a Telegram bot token (see setup below).** | Yes (needs token) |
| `master_nav.py` | Alternative all-in-one nav: integrates BLE scanning inline (short 2 s burst threads with cooldown gaps to avoid GIL contention with LiDAR). Does **not** require `ble_scanner.py`. Uses its own hardcoded route. | Yes |
| `beacon_reference.py` | Archived early prototype combining threaded BLE tracking (with EMA filtering) + LiDAR nav. The FSM is incomplete — only the `WAIT_FOR_BEACON` state is implemented. **Do not run in production.** | No |

---

## What Has NOT Been Integrated Yet

The three main components — `corner_mapping.py`, `ble_scanner.py`, and `hallway_nav.py` — work together but are **not yet merged** into a single script or launcher. Specifically:

- **`corner_mapping.py` is not embedded into `hallway_nav.py`.** They communicate only through `/tmp/jetbot_route.json`. This means the Telegram bot must be started separately, a route command must be sent *before* `hallway_nav.py` starts (or the nav falls back to cruise-only), and if the route file is missing or stale the robot doesn't know its destination.

- **No single-command launcher exists.** Currently you need three separate terminals running three separate scripts simultaneously (see "Running the Full System" below).

- **`beacon_reference.py` FSM is incomplete.** The `WAIT_FOR_BEACON` → `CRUISE` transition exists but all subsequent states (`TURN_APPROACH`, `TURNING`, etc.) are absent from the `try` block. It was superseded by `master_nav.py`.

- **Route is not reconfigurable at runtime.** Once `hallway_nav.py` starts and loads the route file, sending a new `/route` command via Telegram does not update the running nav. The nav would need to be restarted.

- **No obstacle detection is implemented.** All navigation logic was developed and tested under the assumption of clear hallways. The LiDAR is used exclusively for wall-centering and turn detection — there is no logic to recognize or react to a person, object, or other obstruction in the robot's path. Adding obstacle detection (e.g., checking the forward sector for unexpected close readings and halting or rerouting) is a critical next step before deploying in a populated environment like the Mary Campbell Center.

---

## Hardware Requirements

- NVIDIA Jetson Nano with JetBot library (`from jetbot import Robot`)
- RPLidar connected via USB (`/dev/ttyUSB0`, 256000 baud)
- Bluetooth adapter accessible to `bleak` (built-in or USB dongle)
- Apple-format iBeacon transmitters (e.g., iOS devices running a beacon app, or Estimote / Kontakt hardware)
- Python 3.8+

### Python Dependencies

```bash
pip install rplidar-roboticia bleak numpy pyTelegramBotAPI
```

> The `jetbot` package is pre-installed on official JetBot SD card images. If you're on a bare Jetson, install it from the [official JetBot repo](https://github.com/NVIDIA-AI-IOT/jetbot).

---

## Connecting to the JetBot (SSH)

The JetBot runs headless — you interact with it entirely over SSH from your laptop.

1. **Power on** the JetBot. After it boots, the small OLED display on the robot will show its current IP address.
2. **SSH in** from your laptop (replace `<IP>` with what the display shows):
   ```bash
   ssh jetbot@<IP>
   ```
   Password: `jetbot`
3. **The IP address changes approximately every 30 minutes** (DHCP lease renewal). If your SSH session drops or a new connection refuses, power-cycle or wait for the display to update, then SSH to the new address. You do not need to reboot the JetBot — just reconnect to the new IP.

For the full navigation run you need two simultaneous SSH sessions (one for `ble_scanner.py`, one for `hallway_nav.py`). Open two terminal tabs, SSH into the JetBot in each, and run the scripts independently.

---

## Setup

### 1. BLE Beacons

The project uses **Blue Charm BC021** BLE beacons. To view and configure them, install the **KBeacon Pro** app on your phone (available on iOS and Android). Open the app, scan for nearby beacons, select a beacon, and set its Major/Minor values under the iBeacon configuration panel.

Physically place the beacons at hallway intersections corresponding to the node map above and configure each one with:

- Minor: `4949` (same for all three)
- Major: `1` at Node 1, `5` at Node 5, `3` at Node 3

Only Major and Minor are parsed by the code — UUID is ignored. Update `BEACONS` in `ble_scanner.py` if you reconfigure the hardware values.

### 2. Telegram Bot Token

`corner_mapping.py` contains a hardcoded `API_TOKEN`. **Replace this with your own bot token** before running:

1. Open Telegram and message `@BotFather`
2. Send `/newbot` and follow the prompts
3. Copy the token into `corner_mapping.py` line 134, or better, move it to an environment variable:

```python
import os
API_TOKEN = os.environ["TELEGRAM_BOT_TOKEN"]
```

### 3. LiDAR Mount Orientation

The code assumes the LiDAR is mounted with:
- `0°` pointing **forward**
- `90°` pointing **right**
- `270°` pointing **left**

If your mount is rotated, adjust `CENTERING_RIGHT`, `CENTERING_LEFT`, `RIGHT_SECTOR`, and `LEFT_SECTOR` in whichever script you're running.

---

## Reproducing Our Tests (Step by Step)

Tests are listed in increasing complexity. Start from the top and confirm each works before moving to the next.

### Test 1 — Cruise Only (LiDAR sanity check)

Confirms LiDAR is reading correctly and PD steering keeps the robot centered.

```bash
python3 cruise_control.py
```

Place the JetBot in a hallway. It should drive straight and self-correct toward center. Watch the console output: `L: 0.85m | R: 0.83m | Steer: +0.002` means it's seeing both walls and barely correcting. Press `Ctrl+C` to stop.

**What to tune:** If the robot drifts consistently to one side, adjust `BIAS_L` / `BIAS_R`. If it oscillates, lower `KD`. If it's sluggish to correct, raise `KP`.

---

### Test 2 — Single Turn (LiDAR turn detection + FFT rotation)

Confirms corner detection and FFT scan-matching both work for one turn.

```bash
python3 turning_logic.py left
# or
python3 turning_logic.py right
```

Place the robot in a hallway approaching a T-intersection or L-corner on the chosen side. It will:
1. Drive straight (watching for the opening)
2. Creep slowly into the corner mouth (4 s)
3. Capture a reference scan
4. Pivot until FFT says it has rotated 85°
5. Stop

Console should show `[NAV] ✓ Turn complete in X.Xs (measured rotation: 87.3°)`. If it times out (`✗ Turn timed out`), the reference scan is likely capturing too much open space — try starting further from the corner so the robot has more wall geometry in the scan.

---

### Test 3 — BLE Beacon Scan (standalone)

Confirms your BLE beacons are broadcasting and the JetBot can see them.

```bash
python3 ble_scanner.py --manual
```

At the menu, press `1`, `2`, or `3` to scan for a specific beacon, or `all` to scan all three. You should see `FOUND at X.XXm` within 2 seconds if the beacon is within ~10 m. Use `status` to view the current table, `reset` to clear seen flags, `quit` to exit.

**Troubleshooting:** If no beacons are found, verify `BEACONS` major/minor values match your physical hardware. RSSI-based distance is noisy — a beacon 3 m away may report 2 m or 5 m, which is normal.

---

### Test 4 — Multi-Turn Demo (LiDAR only, no BLE)

Confirms the full LEFT → LEFT → RIGHT sequence without needing beacons deployed.

```bash
python3 demonstration.py
```

Place the robot at the start of a route with three corners matching the sequence (left, left, right). No BLE required — corner detection fires on LiDAR geometry alone. After all three turns it enters a final cruise (Ctrl+C to stop).

**What to tune:** `CRUISE_COOLDOWN_S` (default 5 s) is the blind period after each turn before the robot starts looking for the next corner. If the robot turns and then immediately re-triggers on the same intersection, increase this value.

---

### Test 5 — Full System: Telegram Routing + BLE Gating + Multi-Turn Nav

The complete pipeline. Requires three terminals on the JetBot (or SSH sessions).

Both `ble_scanner.py` and `hallway_nav.py` must run **simultaneously on the JetBot**. Open two separate terminal windows (or SSH sessions) into the JetBot and keep both running for the duration of the test.

**Terminal 1 — BLE scanner (keep open the entire time):**
```bash
python3 ble_scanner.py
```
Leave running. It waits for scan requests from `hallway_nav.py` and writes results to `/tmp/jetbot_beacons.json`.

**Terminal 2 — Telegram routing bot:**
```bash
python3 corner_mapping.py
```
The bot must be running *before* you send a route command. It writes the route file and then stays alive.

**From your phone — send a route:**
Open Telegram, find your bot, and send:
```
/route 1 6
```
(Replace `1` and `6` with your actual start and destination nodes.) The bot confirms the path and writes `/tmp/jetbot_route.json`.

**Terminal 2 (or a third terminal) — start nav:**
```bash
python3 hallway_nav.py
```
The nav reads the route file, requests BLE scans for the first beacon via `ble_scanner.py`, and begins cruising. Each corner is gated: the robot will not turn until both (a) the cruise timer has elapsed and (b) `ble_scanner.py` confirms the beacon at that node has been seen.

**Startup order matters.** Always start `ble_scanner.py` and send the `/route` command *before* starting `hallway_nav.py`.

---

### Test 6 — Alternative: All-in-One Nav (`master_nav.py`)

If you want to avoid managing multiple terminal sessions, `master_nav.py` integrates BLE directly using short burst threads. Edit the `ROUTE` list at the top of the file to match your path, then:

```bash
python3 master_nav.py
```

No separate `ble_scanner.py` process needed. The tradeoff is that BLE scanning happens in 2 s bursts with 5 s cooldowns to avoid blocking the LiDAR serial buffer. This means beacon detection can lag by up to ~7 s compared to the separate-process approach.

---

## Architecture Overview

```
Phone (Telegram)
      │
      │  /route 1 6
      ▼
corner_mapping.py ──────────────────► /tmp/jetbot_route.json
  (Dijkstra + turn logic)                        │
                                                 │ (read at startup)
                                                 ▼
ble_scanner.py ◄──────────────────── hallway_nav.py  (main FSM)
  (BLE process)   /tmp/jetbot_ble_request.json     │
       │          /tmp/jetbot_beacons.json          │
       └──────────────────────────────────────────►│
                                              RPLidar
                                           (wall-follow + turn)
```

The IPC between `ble_scanner.py` and `hallway_nav.py` uses atomic file writes (write to `.tmp`, then `os.replace()`) so neither process reads a half-written file.

---

## Key Parameters Reference

| Parameter | File | Default | Effect |
|-----------|------|---------|--------|
| `KP` | `cruise_control.py`, `hallway_nav.py` | 0.12 | Steering sensitivity to wall-centering error |
| `KD` | same | 0.08 | Damping to prevent oscillation |
| `BASE_SPEED` | `hallway_nav.py` | 1.00 | Forward cruise speed (0–1 scale) |
| `CREEP_SPEED` | `hallway_nav.py` | 0.40 | Speed while inching into corner mouth |
| `PIVOT_SPEED` | `hallway_nav.py` | 0.30 | Outer-wheel speed during point turn |
| `CREEP_INTO_CORNER_S` | `hallway_nav.py` | 4.0 s | How long to creep before pivoting |
| `TARGET_ROTATION_DEG` | `hallway_nav.py` | 85.0° | FFT pivot target (coast closes the last ~5°) |
| `CORNER_OPEN_THRESH_M` | `hallway_nav.py` | 2.0 m | LiDAR range considered "open" (no wall) |
| `CRUISE_DELAY_S` | `hallway_nav.py` | 5.0 s | Blind period after each turn before looking for next |
| `GAP_THRESHOLD_M` | `cruise_control.py` | 1.8 m | LiDAR readings beyond this are treated as gaps (doors) |
| `SCAN_WINDOW_S` | `ble_scanner.py` | 2.0 s | Duration of each BLE scan burst |
