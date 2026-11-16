# Lidar delivery repair: evidence and next trial

2026-11-15. Objective: continuous, correctly timed lidar/odometry for supervised
hallway mapping, followed by map geometry and localization validation.

## What the latest evidence establishes

In hallway_ready_01, the fixed IMU publisher delivered 6,972 samples with
continuous 20 ms timestamps. Two active-window scan receipt gaps of 1.285 and
1.362 seconds preceded corresponding raw/fused odometry gaps. The observer
remained responsive. A separate earlier raw-only gap also exists; not every
pause is explained by scan delivery. The stationary map/checkpoints do not
validate motion accuracy or autonomous navigation.

The installed Python 3.6/PySerial 3.5 driver inspection confirms:

- The warning about >3,000 buffered bytes invokes stop() then start(). This
  discards buffered input and restarts scanning. Four warnings were recorded.
- Each response packet is read only after an inWaiting() polling loop, with a
  1 ms sleep when insufficient bytes are present. That loop has no deadline;
  the configured serial timeout does not apply until read() is called.
- The scan generator and ROS publication share one thread. A slow consumer or
  process scheduling pause can therefore contribute to input backlog.

The buffer restarts are a plausible contributor to scan gaps. Previous logs
do not timestamp them sufficiently to establish their individual duration or
prove why overflow occurred. Polling overhead, CPU scheduling, and publication
delays remain competing explanations; USB/device delays are not excluded.

## Implemented candidate

`lidar_serial_reader.py` subclasses the installed vendor driver and replaces
only its response read with serial.read(size). PySerial waits for the requested
bytes up to its existing two-second timeout; a short response raises an error
instead of decoding an incomplete packet. Positive finite timeouts are required.
This removes the extra polling/sleep loop. It is a focused overhead reduction
and bounded-wait repair, not a demonstrated cure for overflow.

Packet decoding, baud rate, buffer limit, overflow recovery, angle conversion,
matcher settings, fusion watchdogs and motion freshness thresholds are unchanged.
The installed system package is not edited. The existing hallway runner takes
the candidate from a repository source snapshot in RAM.

`LIDAR_TIMING_SUMMARY` in lidar.log records serial read count/timeouts/maximum
duration, active stop/start count and last 32 recovery events, scan delivery gaps,
and conversion/publication duration. Slow-event histories are bounded to 32
entries and printed after shutdown, avoiding per-packet disk logging. Times
include monotonic and wall-clock values for comparison with observer events.
Fatal scan exceptions now yield a nonzero process exit; cleanup is idempotent.
The prior observer process-group shutdown fix is also included in this candidate.

Limitations: this does not decouple serial acquisition from ROS publication,
repair acquisition timestamps, or implement scan deskew. Existing scan header
times mark publication, not independently measured acquisition. A timing PASS
therefore qualifies delivery in the observed interval only. A forcibly killed
process may not write its final timing summary.

Reference: [PySerial read and timeout semantics](https://pyserial.readthedocs.io/en/latest/pyserial_api.html).
Installed driver evidence is preserved locally in
`experiments/rplidar_driver_inspection_01.txt` on the analysis laptop.

## Run once, stationary

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
git log -1 --oneline
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
python3 ros2_slam/run_hallway_trial.py hallway_reader_01 --phase stationary
```

Stop if pull fails; preserve local edits. No deployment, C++ build, separate
observer or publisher is needed. Keep the robot stationary through startup and
60 seconds after MAPPING SESSION READY, then press Q. Wait for Saved bundle.
Use a new trial name if repeating; archives are not overwritten.

Success or failure, copy the automatically produced archive in laptop PowerShell:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/hallway_reader_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\hallway_reader_01_bundle.tar.gz"
```

If no archive is printed, preserve the displayed RAM output path and paste the
error. Do not reboot before recovering RAM-only evidence.

## Decision after this run

Require complete status, fresh recorded checkpoints and the existing worst-gap
transport check, not just average rate/p95. Inspect startup separately from the
active interval and correlate recovery/publication events with scan/odom gaps.

- Healthy delivery: proceed to measured turn pair, straight segment and original
  elbow route in HALLWAY_TO_NAVIGATION_PLAN.md. Do not keep repeating stationary
  experiments without a new question.
- Scan gaps coincide with slow publication: evaluate separating acquisition from
  publication with a bounded queue and explicit stale/partial scan handling.
- Restarts persist without slow publication: investigate serial acquisition and
  scheduling/resource evidence before choosing a driver replacement.
- Scans continuous but raw odometry stalls: focus on RF2O. Continuous raw but
  stalled fused output instead focuses the investigation on fusion.

Do not increase freshness limits or buffer size merely to obtain PASS. The aim
is usable hallway sensing, not keeping an unhealthy process alive.

Validation: 71 local tests run, 70 passed; existing C++ test skipped because g++
is unavailable. Tests cover exact packet preservation, short-read failure,
finite-timeout enforcement, serial exception propagation, bounded recovery
history, failed restart reporting, nonzero bridge exit and once-only cleanup,
deployment dependencies, and Python 3.6 syntax. These are software checks;
Jetson timing and physical mapping remain to be tested.
