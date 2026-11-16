# RF2O calculation logging comparison

2026-11-16. Stage trial 02 (b835110) completed with verified C++ timing. Three
calculation/pose/logging calls took 217-223 ms while their internal solver timers
reported 23-28 ms. Odometry publication, TF publication and spin were short.
Logging is a suspect, not yet a confirmed cause. Nine eigensolver failures also
caused nine old estimates to be republished; that independent defect remains.

Use the current compiled binary. No deploy script or colcon build is needed.
The runner's optional --rf2o-log-level warn suppresses routine INFO from only
CLaserOdometry2D. Warnings/errors and the C++ timing summary remain enabled.
The default info preserves the previous launch command. The manifest saves the
requested level and actual RF2O command. Standard Eloquent per-logger syntax:
https://repo.test.ros2.org/en/eloquent/Tutorials/Logging-and-logger-configuration.html

On the Jetson, with no other mapping session running:

```bash
(
set -e
cd ~/jetbot-code-f03243c
git pull --ff-only origin experiment/scan-gate-diagnostics
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
python3 ros2_slam/run_hallway_trial.py rf2o_quiet_stationary_01 --phase stationary --odom rf2o --rf2o-log-level warn
)
```

Keep completely still through startup and 120 seconds after MAPPING SESSION
READY, then press Q. Stationary mode disables driving. Wait for the bundle.
If pull/startup fails, retain the output and stop; do not rebuild speculatively.

On laptop PowerShell, copy even if the report says FAIL:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/rf2o_quiet_stationary_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\rf2o_quiet_stationary_01_bundle.tar.gz"
```

Expected analysis: verify the actual INFO suppression and complete timing
summary, compare post-solver delays and odometry gaps, and count failures and
duplicate stamps. The existing duplicate-stamp failure may remain; the point
of this run is to isolate logging cost, not claim motion readiness. If the
archive is missing, preserve the printed RAM path and send the final error.

After this comparison, repair failed-estimate handling/recovery before moving
qualification. Do not make old estimates appear fresh, treat an unmeasured
interval as zero motion, or relax freshness limits to produce a passing report.
