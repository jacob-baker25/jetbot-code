# Experimental IMU-yaw-aided odometry

## Architecture

This mode is opt-in. Existing runs without a fourth mode argument continue to
use direct RF2O odometry and TF.

In `imu_yaw` mode:

1. The qualified Phidget publisher zeros while the robot is stationary and
   publishes `/imu/data_raw` in `base_link`.
2. RF2O publishes `/odom_rf2o_raw` with `publish_tf=false`.
3. `imu_yaw_odometry.py` buffers device-stamped yaw rates, interpolates the
   integrated yaw at each RF2O timestamp, and composes RF2O's relative body
   translation using that yaw.
4. The node publishes `/odom_rf2o` and the only `odom -> base_link` TF.
5. Slam Toolbox continues to publish `map -> odom`.

The mode exits on nonfinite/reversed IMU data, an IMU gap over 100 ms, an IMU
stream stale over 500 ms, unexpected frames, or non-increasing raw odometry
stamps. Raw odometry gets at most 20 seconds to produce 10 successfully fused
samples; only then is the strict one-second runtime stale guard armed.
Mapping-session process monitoring stops
the robot and session if a required process exits.

No acceleration, magnetic heading, absolute orientation, wheel model, fixed
RF2O multiplier or new ROS package is used.

## Deploy

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
bash ros2_slam/deploy_sensor_diagnostics.sh
```

No colcon build is required.

## Stationary smoke test

Keep the robot completely stationary from command launch through readiness;
the mapping session starts and zeros the IMU automatically. Do not run a second
Phidget publisher.

```bash
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py imu_yaw_smoke_01 euclidean_heading narrow imu_yaw
```

Required output includes `Odometry mode: imu_yaw`, an odometry-source PASS,
20/20 scan-TF, and stationary-preflight PASS. Press `Q` without moving. Inspect:

```bash
EXP=$(ls -dt ~/autochair_ros2_ws/experiments/imu_yaw_smoke_01_* | head -1)
grep -E 'IMU_YAW_ODOM_(READY|FATAL|STATUS)' "$EXP/imu_yaw_odom.log"
grep -E 'queue drops|invalid|No TF/odom' "$EXP/imu.log" | tail
```

Any FATAL marker blocks motion testing.

## Manual turn-pair qualification

Terminal A starts lean passive recording before the stack:

```bash
source /opt/ros/eloquent/setup.bash
python3 ~/autochair_ros2_ws/local_nodes/capture_sensors.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/imu_yaw_turn_pair_01.jsonl \
  --seconds 90 --imu-topic /imu/data_raw --lean
```

Terminal B:

```bash
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py imu_yaw_turn_pair_01 euclidean_heading narrow imu_yaw
```

After all preflights pass, manually turn roughly 90 degrees CCW, settle and
press `L`; return clockwise to the marked start heading, settle, press `L`, then
`Q`. Stop Terminal A after mapping shuts down.

This is a software-qualification run, not proof of hallway improvement. Compare
raw RF2O, fused odometry, IMU integration, Slam Toolbox corrections, process
logs and physical floor residual. Do not proceed if TF has multiple authorities,
the fusion process reports a fatal condition, or stationary preflight fails.

## Hallway decision test

Only after the smoke and turn-pair gates pass, repeat the same taped hallway
route previously used for the narrow-profile baseline. Keep environment,
starting pose, speeds and checkpoint locations unchanged. Run at least one
`rf2o` and one `imu_yaw` session; do not compare unmatched routes. Record
physical final longitudinal, lateral and heading offsets. Primary outcomes are
wall consistency, Karto correction magnitude, physical checkpoint error and
process health—not estimator closure alone.
