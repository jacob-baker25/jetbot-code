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
