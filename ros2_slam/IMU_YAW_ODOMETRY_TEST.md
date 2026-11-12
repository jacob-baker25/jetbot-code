# Experimental IMU-yaw-aided odometry

## Architecture

This mode is opt-in. Existing runs without a fourth mode argument continue to
use direct RF2O odometry and TF.

In `imu_yaw` mode:

1. The qualified Phidget publisher zeros while the robot is stationary and
   publishes `/imu/data_raw` in `base_link`. Publisher and fusion subscriber
   use best-effort QoS with depth 100 (two seconds at 50 Hz) so ordinary Jetson
   scheduling delays do not overflow the default five-message sensor queue.
2. RF2O publishes `/odom_rf2o_raw` with `publish_tf=false`.
3. `imu_yaw_odometry.py` buffers device-stamped yaw rates and queues each raw
   RF2O message until the IMU timeline brackets its timestamp. It then
   interpolates integrated yaw at that timestamp and composes RF2O's relative
   body translation using that yaw.
4. The node publishes `/odom_rf2o` and the only `odom -> base_link` TF.
5. Slam Toolbox continues to publish `map -> odom`.

The mode exits on nonfinite/reversed IMU data, an IMU gap over 100 ms, an IMU
stream stale over 500 ms, unexpected frames, or non-increasing raw odometry
stamps. Raw odometry gets at most 20 seconds to establish at least 20 messages
over three continuous seconds. A startup gap over two seconds resets that
qualification window rather than triggering a runtime failure. Only after the
continuous window passes is the strict two-second stale guard armed. This is
above the measured 1.342-second RF2O gap in the synchronized baseline capture,
while still bounding loss of scan-derived translation. A queued
odometry message may wait at most one second for matching IMU time. Input
receipt and fused-output counts are reported separately, so an input failure
cannot be mislabeled as a fusion synchronization failure.
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
python3 mapping_session.py imu_yaw_smoke_08 euclidean_heading narrow imu_yaw
```

Required output includes `Odometry mode: imu_yaw`, an odometry-source PASS,
20/20 scan-TF, and stationary-preflight PASS. Press `Q` without moving. Inspect:

```bash
EXP=$(ls -dt ~/autochair_ros2_ws/experiments/imu_yaw_smoke_08_* | head -1)
grep -E 'IMU_YAW_ODOM_(STARTUP_GAP|RUNTIME_GAP|READY|FATAL|STATUS)' "$EXP/imu_yaw_odom.log"
grep -E 'IMU_SOURCE_GAP|queue drops|invalid|No TF/odom' "$EXP/imu.log" | tail
```

Any FATAL marker blocks motion testing. A healthy status line has continuously
increasing `raw_received` and `published`, normally zero or one `pending`, and
zero `too_old_drops`.

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
