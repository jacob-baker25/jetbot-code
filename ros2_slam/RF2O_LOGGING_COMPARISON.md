# RF2O calculation logging comparison

2026-11-16. Stage trial 02 (b835110) completed with verified C++ timing. Three
calculation/pose/logging calls took 217-223 ms while their internal solver timers
reported 23-28 ms. Odometry publication, TF publication and spin were short.
Logging is a suspect, not yet a confirmed cause. Nine eigensolver failures also
caused nine old estimates to be republished; that independent defect remains.

Use the current compiled binary. No deploy script or colcon build is needed.
The runner's optional --rf2o-log-level warn suppresses INFO within the RF2O
process only. Warnings/errors and direct C++ timing output remain enabled.
The default info preserves the previous launch command. The manifest saves the
requested level, process scope and actual RF2O command.

Compatibility correction: quiet trial 01 failed before initialization because
rcl 0.8.5 rejects logger:=severity syntax. Zero odometry; no timing comparison
was obtained. Use plain --log-level warn. Verified exact parser source:
https://github.com/ros2/rcl/blob/0.8.5/rcl/src/rcl/arguments.c#L1752
The runner now tests the installed shared rcl parser through rclpy.init/shutdown
before creating trial nodes. No node or motor interface is created by this probe.
If it fails, stop and send output; no full trial has started. Local mocked tests
verify ordering/commands, not the Jetson library itself.

On the Jetson, with no other mapping session running:

```bash
(
set -e
cd ~/jetbot-code-f03243c
git pull --ff-only origin experiment/scan-gate-diagnostics
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
python3 ros2_slam/run_hallway_trial.py rf2o_quiet_stationary_02 --phase stationary --odom rf2o --rf2o-log-level warn
)
```

Keep completely still through startup and 120 seconds after MAPPING SESSION
READY, then press Q. Stationary mode disables driving. Wait for the bundle.
If pull/startup fails, retain the output and stop; do not rebuild speculatively.

On laptop PowerShell, copy even if the report says FAIL:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/rf2o_quiet_stationary_02_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\rf2o_quiet_stationary_02_bundle.tar.gz"
```

Expected analysis: verify the actual INFO suppression and complete timing
summary, compare post-solver delays and odometry gaps, and count failures and
duplicate stamps. The existing duplicate-stamp failure may remain; the point
of this run is to isolate logging cost, not claim motion readiness. If the
archive is missing, preserve the printed RAM path and send the final error.

After this comparison, repair failed-estimate handling/recovery before moving
qualification. Do not make old estimates appear fresh, treat an unmeasured
interval as zero motion, or relax freshness limits to produce a passing report.
