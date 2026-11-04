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

## 3. Run the hallway-constrained candidate once

The first `euclidean_heading` run proved that the gate handoff works, but the
0.5 m correlation search and weak 0.5 distance-variance penalty allowed Karto
to make 0.26-0.29 m ambiguous corrections along the empty hallway.  This
candidate restores Karto's conservative translation settings:

```text
correlation_search_space_dimension: 0.3
distance_variance_penalty:           0.09
```

Do not add landmarks or alter the hallway.  The required environment is the
test environment.  Keep the route, speed, starting pose, and gate unchanged.

```bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py elbow_heading_constrained_01 euclidean_heading
```

Verify the startup output says:

```text
Requested mode: euclidean_heading
Runtime mode:   euclidean_heading
RESULT: PASS
```

Use `N` for named checkpoints at 3 ft, 6 ft, before/after every turn, the far
end, the return corner, and the original position. Press `Q` after aligning at
the taped final pose.

Do not run a repetition until the first constrained result has been reviewed.

## 4. Package and copy results

On the Jetson:

```bash
cd ~/autochair_ros2_ws/experiments
tar -czf elbow_heading_constrained_01_$(date +%Y%m%d).tar.gz \
  elbow_heading_constrained_01_*
```

On the Windows laptop, avoid a quoted path ending in a backslash. Use the full
destination filename:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/elbow_heading_constrained_01_20261104.tar.gz `
  "C:\Senior Year\Senior Design\jetbot_analysis\experiments\elbow_heading_constrained_01_20261104.tar.gz"
```

Change the date in both places if the run is performed on a later day.

## 5. Stop/go checks

- If gate mode, synchronization, or stationary preflight fails, do not drive.
- If the candidate has fewer than 15 `KARTO_SCAN_FINAL` lines, do not tune loop
  thresholds.
- If large low-confidence corrections remain, add response/covariance-based
  fallback to RF2O; do not alter the required hallway to make matching easier.
- Keep legacy and the original Euclidean mode unchanged as historical controls.
