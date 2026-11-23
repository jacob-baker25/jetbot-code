# Passive replay capture and map-quality gate

## Scope

This candidate leaves the tested `guarded_strict` mapping profile unchanged.
`--record-replay` adds a separate `ros2 bag` subscriber after the normal ROS
stack is running. It records only to `/dev/shm`, is stopped before the sensor
stack, and is not placed inside the eMMC evidence bundle.

Recorded topics for IMU-yaw trials are:

- `/scan` and `/scan_slam`;
- `/odom_rf2o` and `/odom_rf2o_raw`;
- `/imu/data_raw`;
- `/tf` and `/tf_static`.

The runner will not permit a trial unless rosbag reports that every requested
topic was subscribed. At shutdown it requires a nonempty SQLite database,
runs `ros2 bag info`, archives the bag in RAM, and records its SHA-256 in the
normal trial manifest. A recorder, validation, or archive failure makes the
trial fail. RAM originals are retained.

## Safety sequence

1. Fetch and unit-test the exact candidate commit in the isolated checkout.
2. Check that Eloquent `ros2 bag record` is installed; do not install packages
   as part of a trial.
3. Run one stationary trial with `--record-replay` and raw JSON capture off.
4. Copy both the small eMMC evidence bundle and the separately printed replay
   archive from `/dev/shm` before rebooting.
5. Qualify recorder load from the normal timing report, LiDAR timing summary,
   message counts, and `ros2 bag info` before any moving capture.

Do not combine the first recorder test with denser graph insertion, range
clipping, timestamp changes, or motion. This is only a recorder qualification.

## Preliminary navigation geometry report

`analysis/analyze_map_navigation.py` reads the saved ROS PGM/YAML directly. It
inflates occupied and unknown cells by half the measured robot width plus the
chosen side clearance, checks map-frame goals for connectivity, and reports
whether recorded trajectory cells would be blocked. It is deliberately a
preliminary gate: held-out localization, a real Nav2 plan, and supervised
execution are still required.
