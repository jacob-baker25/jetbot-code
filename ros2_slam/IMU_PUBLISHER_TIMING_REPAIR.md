# IMU publisher timing repair and stationary qualification

2026-11-13. This supersedes the next stationary-run instructions in
RECORDER_REPAIR_AND_TURN_TEST.md. Motion tests remain conditional on timing
qualification. Keep the hallway and IMU mounting unchanged.

The failed recorder_fix_stationary_01 capture shows a 1.354-second delivery
pause after sample 251, followed by forward/backward header timestamp jumps.
The publisher's one-second age-triggered clock re-anchor caused those jumps.
It now preserves a fixed device-clock anchor through buffered delivery.
Periodic live publisher logging is removed because the pause coincides with
the first status print. Logging is a hypothesis for the pause, not a confirmed
root cause. Fusion watchdogs and algorithms are unchanged.

Timing diagnostics stay in bounded memory during publication and appear as
IMU_TIMING_SUMMARY after shutdown. A forced kill/power loss may lose them.
The first callback provides an approximate clock anchor; initial USB latency
and long-term device/host clock skew are not calibrated by this change.

## Deploy

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
bash ros2_slam/deploy_sensor_diagnostics.sh
```

If pull fails or unexpected edits appear, preserve the output and stop here.
No colcon build is needed. Do not run a separate IMU publisher; the session
starts it. No additional package installation or disk cleanup is required.

## Terminal A: quiet observer

```bash
source /opt/ros/eloquent/setup.bash
python3 ~/autochair_ros2_ws/local_nodes/observe_imu_timing.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/imu_clock_stationary_01.jsonl \
  --seconds 180 --quiet-memory --include-odom
```

## Terminal B: session

```bash
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py imu_clock_stationary_01 euclidean_heading narrow imu_yaw
```

Keep the robot stationary through zeroing/startup and 45 seconds after MAPPING
SESSION READY. Press Q. After automatic shutdown finishes, wait five seconds
and Ctrl+C in Terminal A to save the observer if it is still running.

If startup/session fails, do not repeat or loosen timeouts. Let shutdown finish,
wait five seconds, then stop/save the observer the same way. Use the collection
commands below even if the session has no final checkpoint.

## Collect either outcome

In Terminal B:

```bash
EXP=$(ls -dt ~/autochair_ros2_ws/experiments/imu_clock_stationary_01_*/ | head -1)
tail -30 "$EXP/imu_yaw_odom.log"
tail -5 "$EXP/imu.log"
cd ~/autochair_ros2_ws/experiments
tar -czf imu_clock_stationary_01_bundle.tar.gz \
  "$(basename "$EXP")" sensor_captures/imu_clock_stationary_01.jsonl
```

Only if the session completed with start and final checkpoints:

```bash
python3 ~/jetbot-code-f03243c/ros2_slam/analysis/check_session_timing.py \
  "$EXP" \
  ~/autochair_ros2_ws/experiments/sensor_captures/imu_clock_stationary_01.jsonl
```

Laptop PowerShell:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/imu_clock_stationary_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\imu_clock_stationary_01_bundle.tar.gz"
```

Use a new suffix for an explicitly requested repeat; preserve old captures.

## Decision

A completed run, passing matched timing report, and continuous monotonic IMU
timestamps are prerequisites to resuming measured turns. Review independent
receipt gaps and the new publisher summary too: timing-report PASS covers its
matched window, not every sample or startup. Stationary success does not
establish moving map accuracy or long-duration reliability.

If a pause recurs, compare callback_gap_s, queue_wait_s, publish_call_s,
spin_call_s, loop_gap_s, clock_read_s and sample_age_s with the independent
observer/fusion times. Device gaps are separate from delivery gaps. This
distinguishes delayed callback delivery from publisher processing or downstream
receipt delays before choosing another change. Do not mask a pause by assigning
current timestamps to old data or increasing the fusion stale threshold.
