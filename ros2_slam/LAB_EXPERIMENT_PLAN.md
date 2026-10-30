# Scan-gate diagnostic experiment

This experiment compares the original Eloquent Slam Toolbox scan-distance gate
with the corrected Euclidean distance calculation. Both modes use the same
compiled binary; only the launch parameter changes.

## 1. Pull and deploy on the Jetson

```bash
cd ~/jetbot-code
git fetch origin
git switch experiment/scan-gate-diagnostics
git pull --ff-only

bash ros2_slam/deploy_scan_gate_diagnostics.sh
```

The deployment script copies only the six diagnostic files and saves the old
Jetson versions under `~/autochair_ros2_ws/backups/`.

## 2. Build

```bash
cd ~/autochair_ros2_ws
source /opt/ros/eloquent/setup.bash
colcon build --packages-select slam_toolbox --symlink-install
source install/setup.bash
```

Do not proceed if the build fails.

## 3. Stationary smoke test

With the JetBot safely supported or placed where it cannot drive unexpectedly,
run the mapping session and allow its synchronization and stationary checks to
complete. Stop before commanding motion if anything fails.

```bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py stationary_gate_smoke legacy
```

Confirm `slam.log` contains:

```text
Scan distance gate mode: legacy
```

Finish the session normally so its files are flushed.

## 4. Experiment A: instrumented legacy baseline

```bash
python3 mapping_session.py elbow_instrumented_legacy_01 legacy
```

Use the same marked start pose, turn locations, speed, pauses, and route used in
the previous elbow experiments. Capture named checkpoints at the start, each
meter, immediately before and after each turn, the far end, the return corner,
and the final pose.

## 5. Experiment B: corrected Euclidean gate

Restart the complete stack through a new mapping session, return the robot to
the same marked start pose, and run:

```bash
python3 mapping_session.py elbow_euclidean_gate_01 euclidean
```

Change no other source, parameter, driving speed, or route detail.

## 6. Required artifacts

Each experiment directory should contain the existing maps and logs plus:

- `trajectory.csv`: 10 Hz RF2O, SLAM, and `map -> odom` trajectory
- `slam.log` lines beginning with `SCAN_GATE_ACCEPT`
- `slam.log` lines beginning with `KARTO_SEQ_MATCH`
- `slam.log` lines beginning with `KARTO_SCAN_FINAL`
- any accepted closure as `KARTO_LOOP_ACCEPT`
- `results.json` with `scan_distance_gate_mode`

Do not change the odometry penalty or loop-closure setting until these two runs
have been compared.
