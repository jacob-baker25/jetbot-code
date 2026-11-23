# Passive replay capture and map-quality gate

## Scope

This candidate leaves the tested `guarded_strict` mapping profile unchanged.
`--record-replay` adds a separate bounded `rclpy` subscriber after the normal
ROS stack is running. It records a versioned SQLite replay dataset only to
`/dev/shm`, is stopped before the sensor stack, and is not placed inside the
eMMC evidence bundle. This fallback is used because the Jetson's Eloquent
image does not include the optional `ros2 bag` extension.

Recorded topics for IMU-yaw trials are:

- `/scan` and `/scan_slam`;
- `/odom_rf2o` and `/odom_rf2o_raw`;
- `/imu/data_raw`;
- `/tf` (the known static laser transform is recorded in the trial manifest).

The runner will not permit a trial until the recorder has actually received
every requested topic. At shutdown it requires zero queue drops, no writer
errors, a nonempty message count for every topic, and a successful SQLite
integrity check. It then archives the capture in RAM and records its SHA-256 in
the normal trial manifest. A recorder, validation, or archive failure makes
the trial fail. RAM originals are retained.

## Safety sequence

1. Fetch and unit-test the exact candidate commit in the isolated checkout.
2. Unit-test the bounded recorder without opening hardware.
3. Run one stationary trial with `--record-replay` and raw JSON capture off.
4. Copy both the small eMMC evidence bundle and the separately printed replay
   archive from `/dev/shm` before rebooting.
5. Qualify recorder load from the normal timing report, LiDAR timing summary,
   replay message counts, SQLite integrity result, and zero-drop summary before
   any moving capture.

The first qualified stationary capture contained 17,215 contiguous messages
over 110.2 seconds, with all six topics present, zero drops, no writer errors,
and a maximum queue depth of 8/4096.  All 1,466 delayed scans matched their raw
scan payloads exactly.  This qualifies the capture path; it does not qualify a
map or any matcher change.

`local_nodes/replay_player.py` is the next deliberately narrow layer.  Its
`slam_inputs` mode republishes `/scan_slam`, `/odom_rf2o`, and only the recorded
`odom -> base_link` transforms at the captured receipt cadence.  It shifts all
message stamps together onto the current ROS clock.  The original SLAM-owned
`map -> odom` output is filtered, so it cannot leak the baseline solution into
a candidate matcher.  Keep `--rate 1.0` for qualification; faster playback is
not evidence-equivalent until separately tested.

The first end-to-end stationary replay regenerated the live map exactly: all
9,768 occupancy pixels, map dimensions, origin, resolution, and occupancy
counts matched.  The player nevertheless failed its conservative scheduling
gate because 16 messages exceeded 0.25 seconds late (maximum 0.576 seconds).
Treat map reproduction and scheduler qualification as separate gates.  The
player now reports late bursts, sequence positions, topics, and percentile
lateness so an identical second replay can distinguish a single startup or
SLAM-processing stall from sustained replay overload before moving data is
used for matcher comparisons.

The instrumented repeat passed the scheduling gate.  It scheduled all 9,143
selected events, published 1,466 delayed scans, 1,091 fused odometry messages,
and 1,091 odom-to-base transforms, and filtered all 5,495 baseline map-to-odom
outputs.  There were zero late messages: median lateness was 0.000224 s, p95
0.00531 s, p99 0.00805 s, and maximum 0.124 s against the 0.25 s limit.  Its
map raster again matched both the live stationary map and the first replay at
all 9,768 pixels (raster SHA-256
`485af89d80f408fa53c8a562bf53db5d17cab42cba953186deb929d53d57a318`).
The copied replay-result archive SHA-256 is
`eeab2dde38fb81368df18ea588e24b1955604cbf2bd34343bbce9dba288764f5`.

This qualifies the replay mechanism for a controlled moving capture, subject
to the same per-run zero-late-message gate.  It does not qualify moving scan
matching: the stationary input created only Karto state 0.  Preserve the first
timing-failed replay as evidence, but do not use a replay with nonzero late
messages in matcher comparisons.

Do not combine the first recorder test with denser graph insertion, range
clipping, timestamp changes, or motion. This is only a recorder qualification.

The laser transform remains configurable as `--laser-x`, `--laser-y`,
`--laser-z`, and `--laser-yaw`. Their defaults exactly preserve the tested
configuration (0, 0, 0.1778 m, pi rad); provisional measurements must not
silently replace it.

## Preliminary navigation geometry report

`analysis/analyze_map_navigation.py` reads the saved ROS PGM/YAML directly. It
inflates occupied and unknown cells by half the measured robot width plus the
chosen side clearance, checks map-frame goals for connectivity, and reports
whether recorded trajectory cells would be blocked. It is deliberately a
preliminary gate: held-out localization, a real Nav2 plan, and supervised
execution are still required.

The current straight-clearance defaults are a 0.22 m by 0.22 m footprint and
0.05 m clearance per side, giving a 0.32 m straight-passage target. This tool
does not infer a turning radius from half the width and reports turning
clearance as unevaluated.
