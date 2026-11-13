# Passive ROS IMU qualification

## Scope

`phidget_imu_publisher.py` publishes only the measurement established by the
November 10 bidirectional tests:

```text
/imu/data_raw angular_velocity.z = radians(-native Phidget angular_rate[2])
frame_id = base_link
```

Orientation and linear acceleration are explicitly marked unavailable. Roll
and pitch angular rates are not supplied as measurements. The node publishes
no TF, odometry, commands, fused pose, acceleration or magnetic field. RF2O and
Slam Toolbox remain unchanged and retain their existing TF authorities.

The provisional yaw-rate variance is `1e-5 (rad/s)^2`, conservatively above the
ordinary mounted stationary noise observed after removing one brief transient.
This is an engineering starting value, not a manufacturer-certified covariance.

## Deploy

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
bash ros2_slam/deploy_sensor_diagnostics.sh
```

No colcon build or new package installation is required.

## Stationary smoke test

Terminal A:

```bash
export LD_LIBRARY_PATH="$HOME/.local/phidget22/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source /opt/ros/eloquent/setup.bash
python3 ~/autochair_ros2_ws/local_nodes/phidget_imu_publisher.py --seconds 45
```

Keep the robot completely stationary through the three-second zeroing period.

Terminal B, after Terminal A says it is publishing:

```bash
source /opt/ros/eloquent/setup.bash
ros2 topic hz /imu/data_raw
```

Expected rate is approximately 50 Hz. Stop the topic-rate command with Ctrl-C;
do not interrupt the publisher until its 45 seconds finish.

## Synchronized no-fusion turn capture

Use three terminals. Start the publisher first and keep still while it zeros.

Terminal A:

```bash
export LD_LIBRARY_PATH="$HOME/.local/phidget22/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source /opt/ros/eloquent/setup.bash
python3 ~/autochair_ros2_ws/local_nodes/phidget_imu_publisher.py --seconds 150
```

Terminal B after A is publishing:

```bash
source /opt/ros/eloquent/setup.bash
python3 ~/autochair_ros2_ws/local_nodes/capture_sensors.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/imu_ros_turn_pair_01.jsonl \
  --seconds 120 --imu-topic /imu/data_raw
```

Terminal C:

```bash
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py imu_ros_turn_pair_01 euclidean_heading narrow
```

After readiness, manually turn approximately 90 degrees CCW, settle and press
`L`; then return clockwise to the original floor heading, settle and press `L`.
Press `Q`, but leave the robot still while the capture and publisher finish.
Do not use motors for this timing/coordinate qualification.

Summarize:

```bash
python3 ~/jetbot-code-f03243c/ros2_slam/analysis/summarize_capture.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/imu_ros_turn_pair_01.jsonl
```

Pass gates: `/imu/data_raw` near 50 Hz; frame only `base_link`; no duplicate or
backward stamps; no skipped integration intervals; reasonable near-zero final
integrated yaw after the out-and-back pair; RF2O and SLAM signs agree with the
ROS IMU. Receipt age quantifies publisher/recorder timing but is not acquisition
latency ground truth. No fusion change is permitted from this test alone.
