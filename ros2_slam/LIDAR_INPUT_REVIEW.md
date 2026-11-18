# Stationary raw-angle capture and offline findings

2026-11-17. Do not deploy the offline gap-interpolation candidate as a mapping
fix. It failed slow-turn qualification. RF2O C++, IMU fusion, scan geometry and
published ranges are unchanged by this branch. No colcon build is needed.

## Why this capture is the next step

Actual RF2O sequence replay of two previously copied stationary captures:

| Clip | Duration | Mean bins changing valid/missing per scan | Original endpoint / max excursion | Interpolated endpoint / max excursion |
| --- | ---: | ---: | ---: | ---: |
| still.jsonl | 20.30 s | 134.63 / 360 | 8.44 / 19.76 cm | 0.87 / 1.66 cm |
| move.jsonl (actually stationary) | 23.40 s | 133.46 / 360 | 14.57 / 17.49 cm | 2.00 / 2.17 cm |

Both modes accepted every replayed scan. The candidate filled only isolated
single-bin holes using a same-scan chord between neighbors within 0.3 m range
difference; no temporal averaging or deadband. Original finite returns stayed
unchanged. Raw heading still drifted about 5.1 and 9.8 degrees in those clips.

The same recorded missing-bin masks on an ideal simulated rectangular room
produced false motion even without range noise. Interpolation improved stationary
translation and preserved approximately 10 cm of signed slow translation, but
estimated roughly +/-19 degrees for actual +/-5.73-degree slow turns. Only 4/10
candidate synthetic cases met the predeclared combined position/heading screen.
This is a rejected candidate, not a successful deployment. Do not relax the
screen to call it successful. Synthetic masked rooms are not measured hallway
motion or proof of a unique physical root cause.

Other offline investigations: excluding derivatives near missing points lost too
much support and could expire the reference; zeroing the previous-motion prior
reduced translation but did not solve heading; atan2 angle extraction did not
resolve the slow-turn discrepancy. None was integrated into the live estimator.

The existing captures contain integer-angle bins, not original fractional beam
angles. Current conversion floors the hardware angle and lets later returns
overwrite earlier returns in the same bin. We need original data to test angular
resampling and collision behavior without inventing missing information.

Replay limitations: feeds every captured scan with a fresh zero pose/prior, rather
than reproducing the live node's scheduling/subsampling. No IMU or SLAM replay;
these are raw RF2O coordinates. Do not equate these endpoints to the earlier
nine-minute fused-odometry result. Input hashes and numeric results are retained
in analysis/results/stationary_sequence_review.json.

## What the new recorder does

`--lidar-input-scans 600` records original quality, fractional clockwise beam
angle and distance, along with the exact published 360-bin ranges and stamp.
It stores a bounded RAM list, performs no live disk writes and writes JSONL after
the LiDAR is stopped. Hard bounds: 1200 scans, 2000 input points per scan, 250000
total raw points. Any limit/oversize loss is counted explicitly. The helper does
not change measurements or issue motor commands. Recording overhead is still
possible; existing timing diagnostics remain active. Output is archived by the
existing runner. A forced kill can lose this RAM-only capture.

## On the Jetson: isolated checkout

Coordinate exclusive use of the robot first. No mapping process should be running.
This keeps the existing deployment checkout and teammate branch untouched.

```bash
(
set -e
cd ~/jetbot-code-f03243c
git fetch origin
test ! -e "$HOME/jetbot-input-review"
git worktree add --detach "$HOME/jetbot-input-review" origin/analysis/stationary-sequence-replay
cd ~/jetbot-input-review
git log -1 --oneline
python3 -m unittest discover -s ros2_slam/analysis -p test_lidar_input_capture.py
)
```

If the folder already exists, stop and inspect it; do not delete or reset it.
No deployment script, apt install or C++ build is required for this capture.

## One stationary run (also suitable while remote)

```bash
(
set -e
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/jetbot-input-review
python3 ros2_slam/run_hallway_trial.py lidar_raw_angles_01 \
  --phase stationary --odom imu_yaw --rf2o-log-level warn \
  --rf2o-deadband off --lidar-reader batched --lidar-input-scans 600
)
```

Keep the robot physically still. Stationary mode disables driving. At MAPPING
SESSION READY, wait 60 seconds, press L, wait 5 seconds, then Q. The 600-scan input
capture stops growing automatically; this does not stop mapping. Finish before
the observer's ten-minute cap. Do not run another recorder or an IMU probe.

If it fails, let normal shutdown finish and copy the same bundle; no blind rerun.
If no archive is produced, retain the printed /dev/shm directory and send terminal
output. Do not reboot before recovering RAM data.

PowerShell on the laptop:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/lidar_raw_angles_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\lidar_raw_angles_01_bundle.tar.gz"
```

## Decision after capture

Reconstruct published bins exactly from raw angles and verify parity. Measure
within-bin angle variation, collisions and missing-bin phase. Compare physically
bounded resampling offline, including static scans and signed slow-motion tests.
Only then choose an estimator/input change for a stationary A/B and measured
turn/straight tests. Do not resume elbow/autonomy trials on the basis of a better
stationary number alone. Preserve the current original algorithm as the control.

Reproduce existing replay on Linux with Eigen headers installed:

```bash
python3 ros2_slam/analysis/replay_scan_sequence.py /path/to/still.jsonl /path/to/move.jsonl --output /new/output/directory
```

Exit status 1 is expected for the retained rejected gap candidate. Inspect
report.json; it must not be interpreted as a robot failure. The tool never
deploys code or accesses hardware.
