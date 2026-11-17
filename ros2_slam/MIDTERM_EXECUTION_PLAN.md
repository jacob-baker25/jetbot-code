# Tactical plan: a credible hallway demonstration in 2-3 weeks

2026-11-16. Minimum target: manually produce a measured short-hallway map and
demonstrate repeatable localization after restart. Next target: a short supervised
autonomous goal on that map. Full-building navigation and production-grade
recovery are outside this deadline. Demonstrate only what has actually passed.

Latest action (2026-11-17): use LIDAR_BATCH_TRIAL.md for the next run. The direct
RF2O repair passed timing but drifted while stationary; the subsequent IMU trial
stopped on a measured lidar overflow/restart gap. Do not advance to the motion
instructions below until acquisition and stationary accuracy are reviewed. The
next candidate changes Python serial batching only and needs no C++ rebuild.

2026-11-17 deployment correction: 067b2ea stopped in the standalone compiler
test with `cstdlib: fatal error: stdlib.h: No such file or directory`. The test
harness explicitly added `-isystem /usr/include`, disrupting GCC's include_next
search order. This was reproduced locally. The harness now leaves that built-in
directory in its normal position, while still adding the Eigen subdirectory and
supporting custom Eigen roots. No ROS source was deployed by that failed attempt;
no package reinstall or algorithm change is called for. Pull the correction and
repeat the build block below before attempting the stationary trial.

## Implemented candidate and why

The actual RF2O core was compiled locally with Eigen 3.3.7 and minimal ROS
message/clock stubs. Existing code failed on identical synthetic scans and
reported zero for 20 genuine 5 mm movements. The candidate now:

- Evaluates zero-change Cauchy weights without 10/0 or inf*0. Rejects nonfinite
  systems/solutions and rank-deficient systems instead of propagating them.
- Primes the initial reference from the first real scan. Initializes pose and
  motion state consistently and waits for the laser transform before doing so.
- Publishes only successful estimates. A failed attempt restores the last valid
  scan pyramid and preserves pose, stamp, covariance and motion prior.
- Bounds reference age at 350 ms; expiration stops RF2O/session instead of silently
  reconnecting across unknown motion. No watchdog was weakened.
- Makes the old 12 mm/0.45 deg deadband selectable. Default binary behavior remains
  legacy; the candidate trial explicitly uses off to preserve real slow motion.
- Records accepted/rejected counts and at most three rejected reference/current
  scan pairs plus motion priors, emitted only on shutdown. No full-rate recorder.

Synthetic results: identical scans stay valid; signed 10 cm slow travel estimates
about 10.5 cm with deadband off versus 0 with legacy. Signed 0.1 rad turns estimate
about 0.115 rad. Tests check sign/motion preservation, not calibrated physical
accuracy. Bad->good sequences match a control that spans the rejected interval.
Malformed scans, repeated stamps, nonfinite covariance and expired references
are rejected. The real node process body is tested for valid-only publication,
bounded capture and stop-on-expiration. Full ROS/ARM build remains a Jetson check.

This does NOT prove the ten recorded solver failures had the exact synthetic
cause, eliminate all drift or demonstrate real hallway performance. Removing
the deadband may reveal more stopped noise; the short physical trial decides.

## Pull, test, deploy and build (Jetson)

No mapping process should be running. Paste the complete block. It stops on
failure without closing SSH. The deployment validates known previous file hashes,
runs the actual core tests with installed Eigen/compiler BEFORE changing source,
and preserves backups with COLCON_IGNORE. Do not delete build/install directories.

```bash
(
set -e
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
git log -1 --oneline
python3 ros2_slam/deploy_rf2o_timing.py

cd ~/autochair_ros2_ws
source /opt/ros/eloquent/setup.bash
colcon build --base-paths src --packages-select rf2o_laser_odometry --symlink-install --cmake-args -DBUILD_TESTING=OFF

bash -lc '
source /opt/ros/eloquent/setup.bash
source "$HOME/autochair_ros2_ws/install/setup.bash" >/dev/null 2>&1 || true
RF2O_PREFIX=$(ros2 pkg prefix rf2o_laser_odometry) || exit 1
RF2O_BIN="$RF2O_PREFIX/lib/rf2o_laser_odometry/rf2o_laser_odometry_node"
printf "Selected executable: %s\n" "$RF2O_BIN"
grep -aFq "RF2O_ESTIMATE_POLICY version=1" "$RF2O_BIN" || exit 1
echo "ESTIMATE POLICY BINARY: FOUND"
'
echo "BUILD AND POLICY CHECK PASSED"
)
```

If any step fails, send its output and stop there. Do not launch an old binary.
The full-target build remains mandatory despite standalone tests. The printed
backup contains all replaced files and deployment.json for an exact rollback.

## One stationary qualification (Jetson)

```bash
(
set -e
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/jetbot-code-f03243c
python3 ros2_slam/run_hallway_trial.py rf2o_repair_stationary_01 --phase stationary --odom rf2o --rf2o-log-level warn --rf2o-deadband off
)
```

Stay still through startup and 60 seconds after READY; Q. The runner checks the
runtime policy marker/deadband before teleoperation. Copy the bundle regardless
of PASS/FAIL. A failure should now contain reason counters and bounded raw input.

Laptop PowerShell:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/rf2o_repair_stationary_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\rf2o_repair_stationary_01_bundle.tar.gz"
```

Review freshness, valid recovery, rejection reasons/rate, duplicates, finite
poses and maximum stopped excursion. A timing PASS alone is not motion approval.
Aim for roughly <=5 cm position and <=2 deg heading excursion during this short check;
these are project development screens, not universal navigation safety margins.
If it fails, inspect the captured input locally instead of repeating unchanged
stationary runs or adding an epsilon without evidence. Do not activate IMU yet.

## Next lab action after stationary data is reviewed and qualifies

Use the same unchanged hallway. Mark the start position/heading and 1 m position.
Use a consistent chassis reference point and measured right-angle heading marks.
Do not add lidar landmarks. Physical drift is allowed but must be measured.

```bash
(
set -e
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/jetbot-code-f03243c
python3 ros2_slam/run_hallway_trial.py rf2o_repair_motion_01 --phase route --odom rf2o --rf2o-log-level warn --rf2o-deadband off
)
```

Sequence: start -> left 90 deg -> original heading -> forward 1 m -> back to start
-> forward 1 m at a slower workable speed -> back to start -> Q. At each stop use
SPACE, wait 2 seconds, then L. Record actual forward/lateral/heading offsets, using
the same point on the robot. Slower speed should still turn the wheels reliably;
motor command 0.25 is duty setting, not a measured m/s value. No autonomous motion
is issued by this test. Keep the manual stop available.

Copy rf2o_repair_motion_01_bundle.tar.gz using the same scp pattern. Judge each
segment, not just return-to-start closure. If basic translation fails, do not
collect a full elbow map. If successful, map the original elbow route next.

## Time budget and decisions

- Next 1-3 working days: this candidate, stationary qualification, measured short
  motion. At most one evidence-driven correction cycle after the first result.
  Persistent failures trigger captured-input analysis/alternative assessment,
  not another week of threshold tuning.
- Remainder of week 1: short actual-hallway map with measured width/corner/endpoint
  references. Compare IMU-yaw assistance only if heading remains a material error
  and the direct estimator is valid. Keep whichever mode demonstrably works.
- Week 2: save/reload the map, qualify localization from marked starts, connect
  the existing controller interface and attempt one short supervised goal.
  Verify stale-data/obstacle stopping and footprint/clearance before a corner.
- Final 3 days: freeze the best demonstrated configuration, rehearse, save logs,
  metrics, maps and a video. Use any third week for repeatability, not new sensors
  or a platform migration. If autonomy is not qualified, present measured mapping
  and localization with manual traversal and clearly state the remaining gap.

Defer full-building coverage, lifelong mapping, broad Nav2 tuning, new fusion
architecture, ROS migration and cosmetic map cleanup. If valid RF2O translation
still fails in the real corridor, use the bounded data to assess another lidar
odometry method or independent translation input. IMU yaw cannot replace missing
distance information. The deadline is a reason to narrow scope, not hide invalid
estimates or relax motion guards.
