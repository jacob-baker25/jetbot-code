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

`run_replay_trial.py` packages the now-qualified manual procedure.  It refuses
conflicting sensor/mapping processes and existing output names, verifies the
selected overlay prefix and input hashes, owns the complete process groups for
static TF and SLAM, enforces the player timing gate, saves the final map, and
archives a machine-readable manifest even on failure.  Use this runner for
moving baseline and one-factor candidate replays instead of pasting the manual
multi-process shell sequence.

Do not combine the first recorder test with denser graph insertion, range
clipping, timestamp changes, or motion. This is only a recorder qualification.

## First moving one-factor comparisons

The qualified closed-L moving capture also reproduced exactly under replay:
all 55,357 occupancy pixels, dimensions, origin, and resolution matched the
live `guarded_strict` map. Replay scheduled 9,676 selected events with zero
late messages and a maximum lateness of 0.124 s. This makes the capture usable
for controlled, one-factor map-quality comparisons.

Relaxing `sequential_match_maximum_translation` from 0.05 m to 0.12 m is
rejected. It accepted two more sequential matches, but shifted the final Karto
pose by about 0.19 m and reduced occupied-cell coherence: connected components
increased from 262 to 284, isolated occupied cells increased from 142 to 150,
and occupied cells with at least two four-connected neighbors fell from 1,075
to 964. Physical closure alone therefore does not justify the relaxation.

The next replay keeps the qualified 0.05 m motion guard and changes only
`max_laser_range`, from 12.0 m to 6.0 m. The route legs are about 1.8 m, so
6.0 m retains ample local wall geometry while directly testing whether distant
returns contribute to the false rays and fragmented walls. This is an offline
candidate, not a new default: accept it only if transport timing still passes,
trajectory closure is not degraded, and map coherence improves. Even if it
passes at home, longer Evans Hall sight lines require a later held-out test.

The 6.0 m range replay passed transport timing and preserved the strict
matcher trajectory exactly: it made the same 12 accepted and 16 rejected
decisions and ended at the same final pose. It reduced eight-connected
occupied components from 262 to 237 and isolated occupied cells from 142 to
125, but visual review showed the same central false ray fans with only their
distant ends removed. This is consistent with the capture: only 2,773 of
477,503 finite ranges (0.58%) exceeded 6.0 m. Keep this as a safe diagnostic,
but do not promote it as the map-quality solution.

The next candidate returns to the 12.0 m strict baseline and changes only
`minimum_travel_heading`, from 0.5 rad (28.6 degrees) to 0.25 rad (14.3
degrees). The map's most obvious false structures radiate from turning areas,
and the baseline created only 29 Karto states over the complete route. This
candidate should add intermediate turn keyframes without weakening the 0.05 m
translation correction guard, changing range data, or increasing straight-line
keyframe density. Replay will determine whether the added states improve wall
continuity or instead add more motion-distorted scans. Reject it if scheduling,
closure, matcher rejection behavior, or map coherence degrades.

The 0.25 rad heading candidate is rejected. It increased Karto states from 29
to 34 and accepted corrections from 12 to 17, but its final pose moved about
0.30 m and 17 degrees from the strict result. Occupied components increased
from 262 to 301, isolated occupied cells increased from 142 to 173, and visual
review showed additional radial structure. More turn keyframes therefore add
bad evidence rather than repairing the map.

The recorded scan timing supplies a stronger causal lead. A revolution takes
a median 0.072 s. The bridge stamps a scan when its completed revolution is
published, so its acquisition interval is `[stamp - scan_time, stamp]`.
Interpolating the recorded IMU-yaw odometry over that interval shows a p95
rotation of 4.08 degrees, a maximum of 5.70 degrees, and 205 of 1,558 scans
rotating by more than one degree during acquisition. At three
meters, one degree already spans approximately one 0.05 m map cell. The next
offline diagnostic keeps the exact strict mapper and suppresses only scans
whose recorded acquisition interval rotated by more than one degree. The
filter is opt-in in replay, reports its exact count and maximum observed
rotation, and does not alter odometry or TF cadence. It is not yet a live
sensor change; promote it to the delayed-scan node only if replay improves map
coherence without degrading closure or scheduler timing.

The initial 1, 2, and 3 degree replay sweep mistakenly evaluated the interval
after the publication stamp. It still selected turning scans and produced a
useful qualitative result—every threshold removed the accepted turn
corrections and reduced radial artifacts—but those runs are not exact filter
qualification. The corrected preceding interval changes the counts only
slightly (205, 161, and 121 scans respectively), but requires one exact repeat
before any live change. Do not continue the threshold sweep: its central
finding is already stable, and discarding turn scans sacrifices scan matching.

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
