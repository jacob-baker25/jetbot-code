# RF2O stage timing: locate the remaining delay

2026-11-16. Goal: continuous usable odometry, then measured hallway map geometry
and saved-map localization. Current step: locate delays that occur without IMU.

## Evidence and boundaries

The direct-RF2O control completed with 1,084 uninterrupted scans, no lidar buffer
recoveries and 20/20 scan-TF preflight. Its two active odometry gaps were 438 ms and
417 ms while scans continued. Both the observer and session trajectory recorded
stale odometry. The observer loop stayed below 31 ms. Existing solver log maxima
were 111 ms, but that log excludes parts of RF2O's callback/process/publish loop.
The separate worst-case motion transport check correctly failed at 350 ms.

The transferred rf2o_source_review_01 archive matches all six active local
source/header/build files after newline normalization. This confirms source
agreement, not historical binary provenance. Record the compiled implementation
marker in the new run. IMU hardware is not a sufficient explanation for a delay
that occurs without any IMU/fusion node. QoS blocking, execution/scheduling and
logging remain hypotheses; no one is selected as the cause yet.

This patch measures, without changing executor/rate/QoS, RF2O math, filtering,
scan geometry, matcher settings, or watchdog thresholds:

- spin_some and scan callback elapsed time;
- scan timestamp age and callback receipt intervals;
- complete odometryCalculation, including pose update/logging;
- complete process(), including warning/output preparation;
- odometry publish and TF publish separately;
- rate.sleep and intervals between main-loop starts.

Use steady_clock for elapsed durations. Record counters/maxima and the last 64
slow events in fixed memory, without per-event I/O. Events include end monotonic
time, scan timestamp and callback sequence. Nested stages overlap: do not sum
their durations. Stage elapsed time includes descheduling; it is not CPU time.
Scan age also uses ROS time for comparison with scan headers.

Startup prints RF2O_TIMING_ENABLED version=1 with the actual runtime middleware
identifier. Shutdown prints RF2O_TIMING_SUMMARY JSON. An exception leaving the
main loop also emits the summary before rethrowing. SIGKILL, initialization
failure, or a permanently stuck call may prevent a summary; absence is not PASS.
The runner places parsed diagnostics in timing_report.json; rf2o.log remains
the primary evidence. Existing informational logs remain for this measurement.
timing_diagnostics:=false disables the new recorder if needed later.

Middleware identity uses the long-standing rmw_get_implementation_identifier API;
the rmw/rmw_implementation dependencies name packages already used by ROS, not
an instruction to install or upgrade middleware. Reference discussion from the
ROS maintainer during the Eloquent period:
https://discourse.openrobotics.org/t/ros2-and-dds-ipr-issues/13481/20

## Install once and build only RF2O

No running mapping trial should overlap this source/build change. Preserve
unexpected edits; if pull, source validation, test, or build fails, stop there.

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
python3 ros2_slam/deploy_rf2o_timing.py
```

Deployment checks the six reviewed files against their known original or current
candidate hashes. Unexpected changes abort before edits. A portable C++14 test
must compile and pass on the Jetson before deployment. The script backs up all
replaced files, prints the backup path, and records hashes in deployment.json.
It copies only the node implementation/header, new timing header and two build
metadata files. It does not overwrite CLaserOdometry2D.cpp or Python local_nodes.

```bash
cd ~/autochair_ros2_ws
source /opt/ros/eloquent/setup.bash
colcon build --packages-select rf2o_laser_odometry --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

After build SUCCESS:

```bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
ros2 pkg prefix rf2o_laser_odometry
cd ~/jetbot-code-f03243c
python3 ros2_slam/run_hallway_trial.py rf2o_stage_stationary_01 --phase stationary --odom rf2o
```

Prefix must be /home/jetbot/autochair_ros2_ws/install/rf2o_laser_odometry. Keep
stationary through startup and60s after READY, then Q. Wait for the saved bundle.
The runner handles observation, RAM logging and archiving after shutdown.

Copy the archive on laptop PowerShell, even if the session fails:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/rf2o_stage_stationary_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\rf2o_stage_stationary_01_bundle.tar.gz"
```

If build/deployment fails, paste that output; do not launch the trial against an
older executable. If archiving fails, preserve the printed RAM path and error
before rebooting. A completed run without the enabled marker and final summary
does not qualify the new instrumentation.

## How the next result chooses the work

Correlate large observer odometry gaps with stage events by monotonic time and
scan stamp; review phase and healthy scan delivery alongside them.

- publish or TF stage long: inspect actual middleware/QoS and consumers; do not
  change scan matching to solve a communication delay.
- compute_pose_log long: examine computation, existing logging and scheduling;
  distinguish CPU work from waiting before reducing algorithm quality.
- spin, callback, or sleep long: examine dispatch, scheduling and clock behavior.
- stages normal but receiver late: focus on delivery/subscriber behavior. If
  no slow event repeats, one run cannot establish that the earlier cause vanished.

After healthy timing, proceed measured direct-RF2O turns/straight/original elbow.
Evaluate movement sensitivity of the existing stationary increment filter; zero
stationary drift is not proof of accurate slow motion. Only then compare adding
IMU yaw. Do not change hallway geometry, relax freshness limits or switch the
entire ROS stack based solely on these observations.

## Validation limits

75 local tests: 73 passed, 2 skipped because no local C++ compiler is available.
Python checks cover guarded deployment/no edits on unexpected input, backups,
summary parsing, existing runner/session behavior and Python 3.6 syntax. The
portable C++14 test covers bounded event retention, counts, timestamps and
disabled mode, and is mandatory during Jetson deployment. The actual ROS C++
build and runtime timing require the Jetson; neither has been claimed here.
