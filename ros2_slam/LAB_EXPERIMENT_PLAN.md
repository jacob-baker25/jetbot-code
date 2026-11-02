# Next JetBot lab session

The legacy-versus-Euclidean diagnostic phase is complete. The analysis and
rationale for the next candidate are in `POST_EXPERIMENT_ANALYSIS.md`. This file
is the short operational checklist.

## 1. Deploy and build

On the Jetson:

```bash
cd ~/jetbot-code
git fetch origin
git checkout experiment/scan-gate-diagnostics
git pull --ff-only

bash ros2_slam/deploy_scan_gate_diagnostics.sh

cd ~/autochair_ros2_ws
source /opt/ros/eloquent/setup.bash
colcon build \
  --packages-select slam_toolbox \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DBUILD_TESTING=OFF
```

Do not continue if the build fails.

Verify the package-specific overlay:

```bash
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
ros2 pkg prefix slam_toolbox
```

The prefix must be under `~/autochair_ros2_ws/install/slam_toolbox`.

## 2. Prepare ground truth

- Tape the starting axle center and starting heading.
- Tape 3 ft, 6 ft, 9 ft, and turn positions.
- Use the same route and speed for both runs.
- Record final longitudinal, lateral, and heading offsets from the tape marks.
- Keep the hallway static during each run.

## 3. Run the candidate twice

```bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py elbow_heading_01 euclidean_heading
```

Return exactly to the tape marks, then run:

```bash
python3 mapping_session.py elbow_heading_02 euclidean_heading
```

For each run, verify the startup output says:

```text
Requested mode: euclidean_heading
Runtime mode:   euclidean_heading
RESULT: PASS
```

Use `N` for named checkpoints at 3 ft, 6 ft, before/after every turn, the far
end, the return corner, and the original position. Press `Q` after aligning at
the taped final pose.

## 4. Package and copy results

On the Jetson:

```bash
cd ~/autochair_ros2_ws/experiments
tar -czf elbow_heading_runs_$(date +%Y%m%d).tar.gz elbow_heading_01_* elbow_heading_02_*
```

On the Windows laptop, avoid a quoted path ending in a backslash. Use the full
destination filename:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/elbow_heading_runs_20261102.tar.gz `
  "C:\Senior Year\Senior Design\jetbot_analysis\experiments\elbow_heading_runs_20261102.tar.gz"
```

Change the date in both places if the run is performed on a later day.

## 5. Stop/go checks

- If gate mode, synchronization, or stationary preflight fails, do not drive.
- If both candidate runs have fewer than 15 `KARTO_SCAN_FINAL` lines, do not
  tune loop thresholds.
- If the graph is dense enough but walls remain duplicated, do one controlled
  run with a rigid asymmetric LiDAR-visible landmark at the elbow.
- Keep legacy and the original Euclidean mode unchanged as historical controls.
