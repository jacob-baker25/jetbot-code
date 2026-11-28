# Map quality recovery plan

## Objective and decision rule

Produce a repeatable occupancy map that preserves the demonstrated closed-route
pose closure and leaves the real hallway connected after applying the documented
0.16 m center-to-obstacle clearance.  A visually different map is not success.
The map must support a saved-map localization trial and a supervised short route.

The current replay database has completed its purpose.  Deskewing, rotation
filtering, denser keyframes, range limiting, relaxed matcher corrections, and
stationary-only selection did not make it navigation-ready.  Do not run another
parameter sweep on that capture.  The next work must identify the faulty layer
before changing mapping code again.

## Current evidence

- The strict baseline closes within 0.074 m, but contains false barriers and
  fragmented walls.
- Offline planar deskew changes the scans and modestly reduces occupied
  fragmentation, but leaves the false wall and reduces the largest 0.16 m
  clearance-connected region from about 2.98 m2 to 2.33 m2.
- The complete outbound and return streams overlap within one 0.05 m cell for
  87--93% of active cells, while the 29 mapper keyframes overlap for only
  44--58%.
- A final scan-selection replay publishes 803 revolutions acquired below
  0.003 m translation and 1 degree rotation.  It closes within 0.055 m and
  2.90 degrees, but its largest clearance-connected region is only 1.08 m2 and
  false barriers remain.
- The bridge preserves RPLIDAR quality values, but the mapper does not use them.
- The captured replay contains resampled 360-bin scans, not the original raw
  measurement order.  Scan duration is publication-to-publication time, and
  the bridge stamps after a completed revolution.  Exact beam timing is
  therefore not available in this database.
- LiDAR height is measured, but base-to-LiDAR x/y and yaw remain provisional.

These results make intra-scan motion a secondary contributor rather than the
complete explanation.  The leading alternatives are unstable/incorrect LiDAR
returns, an incorrect physical transform or scan plane, and inter-scan
translation-pose error in a geometrically ambiguous hallway.

## Phase 1: existing-data stationary repeatability audit

### Action

Analyze every stationary episode in the captured database without running
SLAM.  For each episode, measure:

- valid bins per scan and RPLIDAR return quality;
- per-angle median range and median absolute deviation;
- the fraction of repeatedly observed bins stable within 0.05 m;
- pose drift during the episode; and
- endpoint overlap between temporally separated episodes at nearly the same
  recorded pose, especially the initial and final stops.

### Interpretation

- **Poor within-stop repeatability:** prioritize sensor mounting, levelness,
  reflections, raw quality filtering, and resampling.  Do not tune odometry.
- **Good within-stop repeatability but poor same-pose revisit agreement:**
  inspect base-to-LiDAR calibration, environment changes, and accumulated pose
  error.
- **Good within-stop and revisit agreement:** the LiDAR is sufficiently
  repeatable for the scene; prioritize inter-stop translation pose and mapper
  observability.

This phase uses the immutable database and must emit thresholds, counts, and a
machine-readable report.  It is diagnostic only and cannot alter a map.

### Phase 1 result (2026-11-25)

The audit observed all 1,558 mapper scans, classified 802 as stationary, and
formed 13 stationary episodes containing 685 scans.  Within an episode, the
median stable-bin fraction was 1.0 and the median episode's 95th-percentile
per-bin range MAD was 0.00642 m.  Twelve of thirteen episodes had every
well-observed bin stable within 0.05 m.  Random stationary LiDAR noise is
therefore not the dominant map failure in this capture.

The initial and final stationary episodes were separated by 83.4 s.  Recorded
odometry placed them 0.055 m and 2.00 degrees apart, but only 53.8% and 56.7%
of their endpoint cells agreed within one 0.05 m cell.  A bounded rigid search
over +/-0.20 m and +/-10 degrees found a +0.100 m x, -0.075 m y, -2.0 degree
correction that raised the two directional overlaps to 93.5% and 94.5%.  The
search evaluated 6,069 poses and used no SLAM or map feedback.

This is positive evidence for Branch B: the same sensor observations are
highly repeatable and can be reconciled by a small pose correction, while the
recorded inter-stop pose does not align them.  It is not yet independent ground
truth because odometry selected the episodes and a geometric search found the
correction.  The next physical action is therefore the measured transform plus
taped translation-pose validation in Phase 3; another deskew or matcher change
is not justified first.

## Phase 2: controlled sensor and transform validation

Perform this only after Phase 1 determines what must be distinguished.

1. Measure base-link origin and LiDAR x, y, z, and yaw.  Check that the acrylic
   mount is rigid and level and that the scan plane misses the chassis, floor,
   cables, and wheels.
2. Place the stationary robot in a controlled scene containing one flat wall
   and one 90-degree corner at measured distances.  Remove people and reflective
   movable objects.
3. Capture at least 600 raw fractional-angle measurements with acquisition
   order and quality enabled.  Repeat without moving the robot, then repeat
   after a measured 90-degree reorientation.
4. Evaluate wall residual, corner angle, per-angle variance, invalid-return
   rate, quality distribution, and repeatability.  A candidate quality cutoff
   is allowed only if the controlled data shows that it removes unstable
   returns without erasing the wall or corner.

### Gate

Do not resume moving SLAM until stationary walls are continuous and repeated
captures agree within one 0.05 m cell for at least 90% of well-observed wall
endpoints.  If this fails, repair the mount/scan plane or preprocessing first.

## Phase 3: independent translation-pose validation

Use a taped hard-floor path with a distinctive stationary corner visible to the
LiDAR.  Mark 0 m, 1 m, and 2 m.  Start stationary, drive slowly to each mark,
stop for five seconds, and record checkpoints.  Repeat forward and backward.
Heading should remain fixed; this is not a mapping trial.

Compare RF2O raw translation, fused translation, lateral error, and the rigid
alignment of median stationary scans.  Require distance error below 0.05 m per
meter and lateral error below 0.05 m over 2 m before relying on that pose stream
for map construction.

- If scan alignment shows the physical motion correctly but odometry does not,
  improve or replace translation sensing.  Wheel encoders, optical flow, or
  visual odometry are stronger long-term options than widening the scan matcher.
- If odometry and stationary scans both agree, the remaining issue is mapper
  observability/keyframe construction rather than raw sensing.

## Phase 4: select one implementation branch

### Phase 3 result and Phase 4 gate (2026-11-27)

The measured 0--3--6--3--0 ft translation trial completed with valid timing and
no RF2O estimate rejections.  At the full 6 ft mark, fused radial distance was
1.810 m versus 1.829 m measured (-0.019 m).  The return 3 ft checkpoint was
0.913 m versus 0.914 m measured (-0.001 m), and final fused closure was 0.034 m
and 1.2 degrees.  The first 3 ft stop read 0.968 m (+0.054 m), slightly outside
the 0.05 m/m target, but the other measured stops do not show a systematic
scale error.  A swivel-caster event caused a visible return-path veer and a
manual heading correction, so this trial does not independently qualify the
lateral-error requirement.

The replay contained 3,213 stationary scans in seven dwell-qualified episodes.
Median within-stop p95 range MAD was 0.00815 m and the initial/final median
endpoints overlapped within one 0.05 m cell by 89.6% and 91.2% using the
recorded pose.  A 0.075 m lateral correction raised overlap to 95.6% and 94.7%.
This supports short-route pose consistency but leaves a small lateral ambiguity
that could be physical final offset, caster motion, or pose error.  It does not
justify replacing translation sensing before testing mapper observability.

The next authorized test is therefore a single Branch C offline comparison,
not another parameter sweep.  Consolidate each genuine stationary episode to
one per-angle temporal-median scan while replaying the original odometry and TF
unchanged.  Episode selection is fixed before looking at map output: at most
0.003 m and 1 degree motion during a scan, at most 0.03 m and 2 degrees spread
within an episode, no gap above 0.30 s, at least eight scans, and a valid return
in at least 60% of episode scans.  The old L-route should yield 13 consolidated
scans; the translation validation should yield seven.

Compare the L-route temporal-median map against the immutable strict baseline
and the prior 803-scan stationary-only replay.  Continue Branch C only if it
removes or materially weakens the known false barrier without degrading pose
closure or 0.16 m-clearance connectivity.  If the sparse old capture fails,
one new forward-only route with five-second stops every 0.25--0.30 m is allowed;
that capture tests the planned operating method rather than another threshold.
If both the old-capture comparison and the dense-stop confirmation fail, stop
Branch C and revisit lateral pose sensing or an alternative mapper.

### Sparse stationary-median result (2026-11-27)

The bounded-memory confirmation replay passed scheduling with zero late
messages (maximum lateness 0.135 s) and published exactly 13 median scans.  Its
map was byte-for-byte identical to the earlier timing-failed diagnostic map,
so the result is reproducible and not caused by the original scheduler stalls.

Median consolidation reduced occupied 8-connected components from 177 in the
803-scan stationary-only map to 150, and isolated occupied cells from 102 to
71.  However, mapped free cells fell from 6,859 to 5,600 and the largest region
retaining the conservative 0.16 m center clearance fell from 0.870 m2 to
0.708 m2.  The central false-barrier structure remains visible.  The old
capture therefore fails Branch C qualification: suppressing repeated scans is
not enough when its physical stops are roughly 0.75--1 m apart.

No more replays or thresholds are authorized on that capture.  The single
remaining Branch C test is a new forward-only L-route with stops every 1 ft
(0.305 m), five seconds stationary at each stop and after each turn, and no
reverse driving.  Record the normal continuous streams once, then produce two
offline maps from the immutable database: unfiltered strict baseline and the
fixed stationary-median policy.  Judge both with the same false-opening review,
occupied fragmentation, and 0.16 m-clearance connectivity.  If the dense-stop
median map does not materially improve connectivity and preserve closure, stop
Branch C rather than tuning its thresholds.

### Dense-stop capture status (2026-11-27)

The dense-stop L-route capture completed at commit `3b4c372` with valid timing,
fresh streams, 4,205 recorded `/scan_slam` messages, and 30 checkpoints.  The
fixed drive speed was 0.15.  Most consecutive translation checkpoints were
0.25--0.35 m apart.  Fused-odometry start-to-final closure was 0.009 m and 2.5
degrees; the live strict SLAM pose closed within 0.075 m and 1.7 degrees.  The
live map is not navigation-usable: after conservative 0.16 m center-clearance
inflation its largest connected region is 695 cells (1.738 m2), and 68 of 185
unique trajectory cells are blocked after inflation.

The operator reported heading corrections, displaced physical foot markers,
and manually freeing the caster after the 180-degree turn.  These disturbances
mean the run is not independent geometric ground truth and cannot qualify a
navigation map.  They do not invalidate the planned mapper comparison because
the strict baseline and stationary-median candidate will replay the same
immutable sensor and pose streams.  Run the fixed selector audit first, then
exactly one strict replay and one stationary-median replay.  Do not adjust the
episode thresholds after inspecting either map.

### Dense replay insertion-gate correction (2026-11-27)

The fixed selector reproducibly found 46 qualified episodes from 3,087 of
4,205 scans.  The strict replay retained 9,508 free cells and a largest
0.16 m-clearance component of 694 cells.  The first median replay reduced
occupied fragmentation but retained only 6,746 free cells, reduced the largest
clearance component to 626 cells, and left the trajectory blocked fraction
unchanged at 36.8 percent.

The SLAM log showed that this replay did not actually exercise dense keyframes:
46 median scans were published, but the mapper inserted only 15 because its
unchanged 0.50 m travel gate discarded most of the deliberately spaced 0.30 m
stops.  The repository already contains and tests the
`guarded_strict_stable025` configuration, which differs from strict only by a
0.25 m minimum travel distance.  One final median replay with that profile is
authorized as an implementation correction chosen from the physical stop
spacing, not from map output.  All episode-selection and sequential-matcher
safety thresholds remain fixed.  If the corrected replay does not materially
improve clearance connectivity and the known false barrier, stop Branch C;
do not try another travel distance or median threshold.

### Branch C final result and pose isolation (2026-11-27)

The corrected 0.25 m gate inserted 24 scans instead of 15, confirming that the
dense-stop implementation correction took effect.  It did not qualify: the
largest 0.16 m-clearance component fell to 613 cells (1.533 m2), compared with
694 cells (1.735 m2) for strict baseline, and the false-wall/fan structures
remained.  The trajectory blocked fraction improved from 36.8 to 32.9 percent
but remained far from a connected navigation route.  Eighteen of 23 sequential
matches were rejected by the fixed safety guards, mostly because the requested
translation corrections exceeded 0.05 m.  Branch C is stopped.

A Karto-free attribution projected the same 46 stationary median scans directly
at their recorded fused-odometry poses.  Outbound and return endpoints agreed
within one 0.05 m cell by only 64.6 and 62.3 percent.  The 0.25 m keyframe subset
agreed by only 63.5 and 64.7 percent.  A bounded global correction search over
+/-0.20 m and +/-10 degrees could raise the keyframe symmetric minimum only to
68.4 percent; its best coarse correction was +0.05 m x, +0.05 m y, and 0 degrees.
The all-median search did not improve the original symmetric minimum.  The
inter-pass discrepancy therefore varies along the route and cannot be repaired
by one end-of-route transform.

This evidence selects Branch B.  Stationary LiDAR observations are repeatable,
deskew and stop medians do not restore connectivity, and pose-only projection
already contains the duplicated geometry.  The next work must independently
measure or replace translation/planar pose; no further scan-selection, travel
gate, or sequential-matcher sweep is authorized on this capture.

### Branch A: sensor/timing defect

Timestamp scan boundaries in the serial reader, preserve raw received order,
publish a ROS-compliant first-ray timestamp, and deskew raw fractional points
before one final resampling step.  Use direct high-rate IMU integration for
yaw and the independently validated translation source.  Validate on the known
wall/corner before any hallway map.

### Branch B: translation-pose defect

Keep the current conservative matcher and correct the pose source.  Do not use
the LiDAR-derived RF2O translation as both the error source and its own ground
truth.  Qualify the replacement on the taped path, then replay the controlled
corner and a closed route.

### Branch C: sensors and pose pass, hallway remains ambiguous

Use deliberate stop-and-scan mapping: stop every 0.25--0.30 m and after each
turn, form one temporal-median scan from several stationary revolutions, and
insert only that scan.  This is different from filtering the existing capture,
whose stops were 0.75--1 m apart.  If needed, compare one alternative mapper
only after this controlled input is available; switching SLAM stacks before
the sensor and pose gates pass is not authorized.

## Final qualification

The selected branch must pass, in order:

1. controlled wall/corner geometry;
2. taped translation and turn-pair tests;
3. a closed route with position closure at most 0.15 m and heading closure at
   most 10 degrees;
4. no false barrier across the known opening;
5. a connected route after 0.16 m clearance inflation; and
6. a held-out hallway followed by saved-map localization and one supervised
   short autonomous route.

Preserve every baseline and use exact source/input hashes.  Reject a candidate
that improves closure while reducing navigable connectivity.  Stop a branch
after one controlled correction and one held-out confirmation fail; do not
continue threshold tuning without new evidence.
