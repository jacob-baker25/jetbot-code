# Verified hallway experiment: parameter-loading repair

This plan supersedes the YAML-only experiment in commit 97ede4e. The previous
loader ignored YAML values. Deploy and rebuild this repair before testing.

## What changed

- All 28 SMapper parameters now read the configured value before calling Karto.
- Every setting is read back from Karto and printed as KARTO_CONFIG.
- mapping_session checks all 28 internal values before enabling its driving
  loop. Missing markers, wrong profiles and stale binaries fail startup.
- Each experiment saves slam_params.yaml, its SHA256 and the internal settings.
- baseline reproduces the previous effective mapper configuration.
- narrow changes only correlation_search_space_dimension: 0.5 -> 0.3.
- Both retain distance_variance_penalty: 0.5 and minimum_distance_penalty: 0.05.
  The latter is the old effective value, despite the old YAML requesting 0.5.

Use the same ordinary hallway, starting marks, route, speed, and turn directions.
A narrower search is a candidate to test, not a validated mapping solution.

## 1. Deploy and rebuild on the Jetson

Use the verified clean checkout, not the older corrupt ~/jetbot-code checkout.
Stop other mapping sessions first. If git status reports local changes, inspect
them before pulling; do not discard them.

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
git log -1 --oneline
bash ros2_slam/deploy_scan_gate_diagnostics.sh

cd ~/autochair_ros2_ws
source /opt/ros/eloquent/setup.bash
colcon build --packages-select slam_toolbox --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF
```

Do not continue after a failed build. No clean-cache or deletion is needed.

```bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
ros2 pkg prefix slam_toolbox
df -h /
```

Expected prefix: /home/jetbot/autochair_ros2_ws/install/slam_toolbox.

## 2. Stationary baseline smoke test

Put the robot on the floor, leave it stationary, and run:

```bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py mapper_repair_smoke euclidean_heading baseline
```

Expected new output:

```text
KARTO INTERNAL CONFIGURATION CHECK
Profile: baseline
correlation_search_space_dimension: 0.5
distance_variance_penalty: 0.5
minimum_distance_penalty: 0.05
Verified parameters: 28/28
RESULT: PASS
```

Gate-mode, scan/TF and stationary preflights must also pass. When the driving
controls appear, press Q without driving. Send any failure output for review.
A stationary test validates deployment; it does not establish motion accuracy.

## 3. Narrow-search physical experiment

Prepare marks before launch. Reference all distances to the wheel axle center.
Measure 3, 6, and 9 ft along the first leg and 6 ft along the perpendicular leg.
Record hallway width and photograph the starting alignment if practical.

```bash
python3 mapping_session.py elbow_narrow_01 euclidean_heading narrow
```

Expect the same new check, with Profile: narrow and search dimension 0.3.
The penalty values remain 0.5 and 0.05. All 28 settings and other preflights
must pass before driving.

Controls: W forward, S backward, A/D turn, Space stop, N named checkpoint, Q
finish/save/shutdown. The command watchdog stops motion when commands stop;
Space is the explicit stop. Keep the default drive and turn speeds.

Stop and press N at each point, entering these names:

1. straight_3ft
2. straight_6ft
3. before_90 (at the 9 ft mark)
4. after_90
5. far_end (6 ft beyond the corner, before turnaround)
6. after_180
7. return_corner (back at the elbow)
8. after_return_90 (facing the start)
9. return_6ft (6 ft from the original start)
10. return_3ft (3 ft from the original start)
11. start_before_final_turn (at the start position, before turning to face the
    original heading)
12. original_position (after final heading alignment)

The extra checkpoint before the final turn separates translation error from
turn-induced corrections. Drive corrections are allowed; do not lift or
manually reposition the robot during a run.

Press Q promptly after original_position. Do not move the robot after shutdown
until the axle-center longitudinal/lateral offset and heading error have been
measured. Record signed offsets, measurement method, and any uncertainty.

## 4. Copy this run

Use the exact directory name printed by the program, including its timestamp.
The following example uses a placeholder which must be replaced.

On Jetson:

```bash
cd ~/autochair_ros2_ws/experiments
RUN=elbow_narrow_01_YYYYMMDD_HHMMSS
tar -czf "${RUN}.tar.gz" "$RUN"
```

On Windows PowerShell:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/elbow_narrow_01_YYYYMMDD_HHMMSS.tar.gz `
  "C:\Senior Year\Senior Design\jetbot_analysis\experiments"
```

Also send the startup internal-configuration check, final summary, and physical
measurements. The saved map alone is insufficient.

## 5. Evaluation and next decision

Evaluate intermediate estimated distances against measured marks, return
displacement, final heading, repeated-wall alignment, gate handoff, and
corrections per scan. A smaller endpoint error or smaller corrections alone
are not proof of improvement. RF2O previously estimated only 2.138 m at a
2.743 m mark; stronger trust in odometry can preserve scale error.

Review the first narrow result before doing another run. A useful follow-up
if it improves is a baseline physical run with the repaired loader, followed
by a narrow repetition under comparable conditions.

The analyzer reports baseline, narrow, and historical runs separately:

```bash
python3 ~/jetbot-code-f03243c/ros2_slam/analysis/analyze_experiments.py \
  ~/autochair_ros2_ws/experiments --name 'elbow_narrow_01_*'
```

## Local validation

The regression suite tests actual C++ configuration and getter/setter bodies
against a Node test double, profile isolation, stale/wrong/missing log failures,
Python 3.6 syntax, the real session guard, and analysis classification:

```bash
python3 -m unittest discover -s ros2_slam/analysis -p 'test_*.py' -v
```

g++ is needed for the C++ regression; other tests use Python's standard library.
This does not replace the Eloquent build and motor-free smoke test on Jetson.
