# From supervised hallway mapping to autonomous traversal

2026-11-15. Goal: a repeatable, geometrically credible hallway map on which the
JetBot can localize and drive between goals. A stationary PASS is one prerequisite,
not the deliverable. Keep the actual hallway unchanged; measurement tape is for
ground truth, not added lidar landmarks.

## Evidence and decisions

The first tmpfs run completed: 5,180 continuous 20 ms IMU timestamps, 900 raw
and 900 fused messages, and passing recorder timing. A runtime raw-odometry
pause of 0.804s remains. Disk-output failures and tmpfs success support an I/O
contribution without proving one. Do not expand timeouts to declare success.

Earlier maps/straight/elbow comparisons mostly predate the lidar handedness
repair. They cannot establish today's map quality or justify a fixed odometry
scale correction. Subsequent turns were not physically measured accurately
enough to establish absolute heading error. The IMU constrains yaw, not missing
along-corridor translation information.

The next candidate makes existing repairs actually runnable and replaces manual
capture/deployment ambiguity with a source snapshot and automatic evidence bundle.
This is functional qualification, not a clean experiment isolating each change.
It is a new candidate; the earlier successful RAM run does not validate it.

Implemented:
- Run repository Python from a frozen /dev/shm snapshot. Leave installed local_nodes
  and its additional reporting/bias code intact. Use installed ROS/C++ packages.
- Include the previously tested fixed device-clock anchor and quiet IMU publisher.
  Preserve sample times through backlog; do not disguise late data as fresh.
- Start a quiet observer automatically for /scan timing, raw/fused odometry and IMU.
  It records timing metadata, not full laser arrays for replay. Its 10-minute cap
  ends the trial if exceeded. Q normally ends the trial first.
- Keep session files in tmpfs; stop/save observer before archiving to disk. Retain
  original RAM files on failure. Print the final persistent archive path. Snapshot
  hashes, manifest, results and timing report travel together.
- Stationary phase disables nonzero motor commands. Moving phases check scan and
  selected-odometry header AND receipt freshness, initially <=0.35s, future <=0.05s.
  Missing/stale data aborts driving and stops the session. These are conservative
  project limits, not validated stopping-distance guarantees. A blocked process
  can delay this software check and the existing command watchdog.
- Preserve failed/interrupted result status instead of leaving results.json running.

No changes to scan gate, matcher profile, TF ownership, gyro mounting sign or
fusion timeouts. No C++ build or package installation. No claim of autonomous
navigation support yet. The separate installed heading-report/hold-still features
are not used by this candidate; they remain available in the old installation.

## Immediate run: one terminal, stationary candidate qualification

On the Jetson (inspect status; retain unexpected edits, do not reset):

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
python3 ros2_slam/run_hallway_trial.py hallway_ready_01 --phase stationary
```

Do not deploy over local_nodes or launch another publisher/observer. Keep still
through zeroing/preflight and 60s after READY, then Q. Wait for Saved bundle.
If a preflight or process fails, it still attempts an archive after shutdown;
paste any archive error and retain the RAM path. RAM originals disappear at
reboot. A normal run needs at least 64 MiB free on disk and /dev/shm at launch;
large/long trials may need more. Use a new name for each run. No auto-deletion.

Laptop PowerShell:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/hallway_ready_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\hallway_ready_01_bundle.tar.gz"
```

The archive has both a recorded-timing result and a separate stricter transport
check. The latter requires >=100 active messages per required stream, no reverse
stamps, and active maximum ages/gaps <=0.35s, not just p95. An observer interruption
can affect these metrics, so review its loop timing too. On failure, compare
/scan vs raw odometry gaps: scan gap first suggests lidar/bridge delivery;
continuous scans but raw gaps focus work on RF2O; raw continuous but fused gaps
focus on fusion. This provides a decision from the next run without another
round of commands merely to collect the missing topic.

## Conditional physical tests

Once the candidate has healthy transport, no fatal errors, fresh checkpoints
and verified source/configuration, proceed directly to these short tests. Do
not accumulate more stationary runs without a specific unresolved question.

1. **Measured turn pair.** Tape perpendicular reference lines; align a visible
   robot heading reference (record alignment uncertainty, aim <=2 degrees).
   Run `python3 ros2_slam/run_hallway_trial.py hallway_turns_01 --phase turns
   --turn-speed 0.12`. `[` and `]` lower or raise turn speed by 0.02 while
   stopped; use the lowest setting that moves reliably. The exact endpoint need
   not be 90 degrees if its signed physical angle is measured and recorded.
   Turn left 90 degrees as viewed from above, SPACE, wait 5s, L. Turn right to
   the starting line, SPACE, wait 5s, L, then Q. Record physical angle and any
   translation at each checkpoint. Repeat opposite direction if the first is
   valid. Initial screening target: selected odometry and SLAM within 5 degrees
   of measured heading; compare raw RF2O too. These are project targets, not a
   claim that 5 degrees is sufficient clearance for autonomous navigation.
2. **Measured 9-foot straight segment.** New `--phase route` trial. Checkpoints
   at 3, 6 and 9 feet (0.9144, 1.8288, 2.7432m), stopping/settling before L.
   Measure lateral drift and actual heading, not just nominal distance. Initial
   screen: endpoint position error <=0.15m and length error <=10%, with no systematic
   worsening at each checkpoint. If rotation is good but along-hallway distance
   fails repeatedly, investigate scan observability/translation. Extra gyro tuning
   or an EKF cannot create absent position information; wheel encoders or another
   independent translation source become a justified option then.
3. **Original elbow route, out and back.** Use the same hall and measured path.
   Record turn/start/end poses and hallway widths. Assess wall straightness,
   correct corner direction/angle, measured widths/segment lengths, doubled walls,
   and revisited-wall alignment, not only endpoint closure. Initial screening:
   width error <=0.15m, repeated walls within 0.15m, corner error <=5 degrees;
   report measurement uncertainty. Tighten against actual clearance later.
   If timing and odometry are good but map geometry fails, compare raw scans,
   extrinsics, scan acquisition timing/motion distortion and matcher corrections
   before changing another parameter. Slow turns can test distortion sensitivity.

At every checkpoint record on paper: label, physical along/lateral offsets,
heading, alignment uncertainty, corrections/handling, nearby moving people.
Use start-relative coordinates (forward x, left y, CCW positive). No invented
ground truth from an estimator. Compare one raw-RF2O baseline route (`--odom rf2o`)
with the IMU candidate only after both are valid; repeat if differences are near
measurement uncertainty. Choose the simpler configuration if IMU adds no benefit.

The launcher owns passive-observer shutdown and must save a nonempty
`observer.jsonl` before timing analysis. An empty capture makes the timing result
invalid even when RF2O and lidar internal summaries appear healthy.

## Map use and autonomous traversal

After repeatable map geometry, save the occupancy map, its YAML and the matching
source/configuration manifest. Test localization against that fixed map from
several known hallway poses and after restarting. Measure pose error and recovery;
a live mapper correcting its own map is not the same as localization on a saved map.
Choose AMCL versus supported SLAM Toolbox localization only after checking the
actual Eloquent installation; do not copy Rolling launch commands into JetPack.

Before autonomous motor commands, establish a calibrated cmd_vel-to-wheel
interface with a command timeout, speed/turn response and stopping distance.
The existing teleop PWM values are not velocities in m/s. Verify odometry twist
and covariance semantics for the chosen consumer. Measure footprint, lidar
offsets, hallway clearance and localization uncertainty; use that error budget
to set costmap inflation/speeds and decide whether map errors are acceptable.

Then test one short goal in a clear hallway under supervision, obstacle stop,
lost-localization/sensor stop, restart recovery, then multi-goal elbow traversal.
Only these physical tests can establish autonomous success. A saved map alone
does not implement planning, collision avoidance or motor control.

Architecture reference: Nav2 requires a consistent map->odom->base_link chain,
and an odometry message for velocity information. Preserve one TF authority per
edge. These official docs inform the architecture, not compatibility claims for
the old Jetson packages:
- https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/
- https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom/

An EKF migration remains a possible later engineering choice. Current data do
not justify it as a cure for transport stalls, wrong scan timing or missing
corridor translation. A JetPack/ROS upgrade would consume substantial integration
time and is deferred unless a specific unsupported component blocks navigation.
