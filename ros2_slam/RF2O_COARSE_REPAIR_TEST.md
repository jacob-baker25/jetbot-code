# Coarse-level repair: deploy and qualify while stationary

2026-11-17. The integrated core is byte-for-byte (normalized text) the candidate
verified by rf2o_offline_coarse_01: source SHA256
c6c55ee42fc954255e7bb0a7e2d232681d53013da9613c1c27b90a7073656ec1.
All six rejected pairs recovered and both core regression suites passed on Jetson.
This supports the rejection-policy repair, not a claim of improved drift/map accuracy.

No movement test: user is remote. Close mapping and sensor probes first. Retain
batched acquisition, IMU yaw, narrow matcher, deadband off and existing guards.
Deployment first compiles/runs synthetic core tests plus six saved-pair checks;
failure prevents source copying. Backup exclusion and previous source hash guards
remain. Only RF2O needs a ROS rebuild. Runtime marker rejects an old binary.

## SSH: pull, guarded deployment, build, binary check

```bash
(
set -e
set -o pipefail
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
git log -1 --oneline
LOG="$HOME/autochair_ros2_ws/experiments/rf2o_coarse_build_01.log"
test ! -e "$LOG"
python3 ros2_slam/deploy_rf2o_timing.py 2>&1 | tee "$LOG"
cd ~/autochair_ros2_ws
source /opt/ros/eloquent/setup.bash
colcon build --base-paths src --packages-select rf2o_laser_odometry \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF 2>&1 | tee -a "$LOG"
source install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
RF2O_PREFIX=$(ros2 pkg prefix rf2o_laser_odometry)
RF2O_BIN="$RF2O_PREFIX/lib/rf2o_laser_odometry/rf2o_laser_odometry_node"
grep -aFq 'RF2O_PYRAMID_POLICY version=1 skip_rank_deficient_coarse=true require_finest=true' "$RF2O_BIN"
echo "BUILD AND PYRAMID POLICY CHECK PASSED"
)
```

Continue only after the final PASS line. If pull fails, send its terminal output.
If deployment/build/check fails, stop: no live trial. Copy the build log below.
The existing tf2 pedantic extra-semicolon warnings alone are not a build failure.

## SSH: one stationary validation

```bash
(
set -e
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/jetbot-code-f03243c
python3 ros2_slam/run_hallway_trial.py rf2o_coarse_stationary_01 \
  --phase stationary --odom imu_yaw --rf2o-log-level warn \
  --rf2o-deadband off --lidar-reader batched
)
```

Motors are disabled by stationary mode. Keep physically still. After MAPPING
SESSION READY, wait120seconds, press L, wait5seconds, then Q. Do not run the
separate full-input recorder in this validation; saved rejected fixtures and
prior raw clips are retained. The runner records timing/poses and bundles logs.
If any guard/process fails, let shutdown finish and copy its bundle; do not rerun.
If failure precedes bundle creation, send terminal output and preserve any printed
RAM directory. Do not enlarge timeouts or approve motion based on timing alone.

## Laptop PowerShell: copy available outputs

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/rf2o_coarse_build_01.log "C:\Senior Year\Senior Design\jetbot_analysis\experiments\rf2o_coarse_build_01.log"
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/rf2o_coarse_stationary_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\rf2o_coarse_stationary_01_bundle.tar.gz"
```

Copy only the build log if the trial was not started. Review runtime marker,
reference expiry/rejections, stream gaps and long stationary position excursion
separately. Small individual false increments can accumulate; this change cannot
be accepted as a drift fix from solver acceptance alone. Measured movement awaits
physical lab attendance and review. No SLAM/IMU/lidar retuning in this run.
