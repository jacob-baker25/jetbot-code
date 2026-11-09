# RPLIDAR handedness repair and validation

## What was wrong

The Python RPLIDAR library reports the scanner's native angle, which increases
clockwise. ROS REP 103 requires positive yaw and planar angles to increase
counterclockwise. The old bridge put each native angle directly into a
positive-increment `LaserScan`, so it reflected every scan across an axis.

The bridge also published 360 samples at one degree per sample while declaring
`angle_max = 2*pi`. The last sample is actually at 359 degrees. The repaired
metadata now satisfies:

```text
angle_max = angle_min + (number_of_samples - 1) * angle_increment
```

The existing 180-degree static yaw is retained. It controls which way the
lidar's zero ray points on the robot; it cannot correct a reflection and is a
separate mounting parameter. Scan timestamp behavior, RF2O, scan delay, Karto,
the matcher profile, and motor controls are unchanged in this repair.

## Deploy

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
bash ros2_slam/deploy_lidar_geometry_fix.sh
```

No colcon build is required. The deployment prints the backup directory. Stop
if `git status --short` is not empty or the deployment reports a missing file.

## Decisive sign test

Use a taped 90-degree floor reference. The IMU must remain face-up and fixed to
the robot in the documented orientation. Use two SSH terminals.

Terminal A:

```bash
export LD_LIBRARY_PATH="$HOME/.local/phidget22/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 ~/autochair_ros2_ws/local_nodes/phidget_probe.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/lidar_fix_ccw90_01.jsonl \
  --seconds 45 --zero-gyro
```

Keep the robot still during the three-second zeroing period. Once recording
starts, leave it still for five seconds, physically rotate the whole robot 90
degrees counterclockwise when viewed from above (nose from 3 o'clock to 12
o'clock is one example), then leave it still through the end. A manual rotation
is preferred for this sign test because it removes motor-key naming ambiguity.

Terminal B:

```bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py lidar_fix_ccw90_01 euclidean_heading narrow
```

After the stack is ready, coordinate the same manual turn with Terminal A.
Press `L` after the turn has settled, then `Q`.

## Pass/fail rule

- The already-established raw sensor convention for this mounting is expected
  to integrate to approximately **-90 degrees** for physical CCW.
- With the repaired lidar conversion, ROS RF2O and SLAM must report
  approximately **+90 degrees** for that same physical CCW turn.
- The signs are the primary gate. Do not tune thresholds based on this run.
- Record the floor-reference error and the final RF2O/SLAM angles. Treat
  estimator agreement as supporting evidence, not physical ground truth.

If RF2O remains negative, restore nothing yet: preserve/copy the complete run
and lidar log so the installed bridge and actual wall directions can be audited.
If RF2O becomes positive but SLAM remains negative, preserve the run and stop;
that would localize the remaining convention mismatch downstream of RF2O.

This test validates coordinate handedness. It does not by itself prove that map
quality, real-time latency, or loop closure has improved.
