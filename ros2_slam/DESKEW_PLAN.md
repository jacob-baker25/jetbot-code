# LiDAR deskew plan

## Goal

Improve wall geometry without giving up the loop closure already demonstrated
by the guarded-strict SLAM configuration.  The change is successful only if it
produces a more useful occupancy map while preserving repeatable closure and
safe navigation clearance.

Deskewing is a correction for motion during one LiDAR revolution.  It is not a
general cure for bad calibration, moving objects, weak scan matching, or poor
driving.  For that reason it remains opt-in until it passes every gate below.

## Evidence and current hypothesis

- The bridge publishes a completed revolution with a header stamp taken after
  acquisition, but `slam_toolbox` treats the whole scan as if every ray were
  captured at that one time.
- The recorded closed-L run contains 1,558 scans.  A scan takes about 72 ms,
  and turns move several degrees during some individual scans.
- Rejecting motion-heavy scans did not improve the map enough: a 1-degree
  cutoff removed 205 scans and reduced mapped free space without creating scan
  corrections.  That supports correcting useful measurements rather than
  discarding them.
- The current cartesian resampler reverses the clockwise hardware bins into
  ROS counter-clockwise bins.  For a 360-bin scan, acquisition rank is therefore
  `(-ros_bin) mod 360`.
- The recorded fused odometry supplies IMU yaw and RF2O translation.  It is
  available throughout the scan and is the best motion estimate in the current
  hardware/software stack.

## Fixed experimental controls

The first deskew comparison changes one factor only.  It uses the same captured
database, guarded-strict parameters, transform, replay cadence, and map-saving
path as the strict baseline.  Scan rejection is disabled when deskewing is
enabled.

The offline transform:

1. treats the scan interval as `[header_stamp - scan_time, header_stamp]`;
2. interpolates base position and unwrapped yaw from `/odom_rf2o`;
3. assigns each ROS bin its approximate clockwise acquisition time;
4. transforms each endpoint into the odometry frame at its beam time;
5. transforms it back into the laser frame at the scan-end reference pose;
6. rebins the endpoint, keeping the nearest return on collisions; and
7. publishes the corrected scan at the original end stamp with
   `time_increment = 0`, because all output points share one reference pose.

The source capture is never edited.  Scans without full odometry coverage are
passed through unchanged and counted in the result instead of extrapolated.

## Gates

### 1. Geometry and timing gate

- Stationary input must remain geometrically unchanged.
- Synthetic translation, rotation, yaw wrap, acquisition order, collision,
  and missing-coverage cases must pass unit tests.
- The real capture must produce finite diagnostics with no writer/database
  changes and no silent scan loss.

### 2. Same-data replay gate

Compare the deskew replay with the exact strict baseline and the rejected
rotation-filter trials.  Require:

- zero late replay messages and a saved map;
- final closure no worse than 0.15 m and 10 degrees;
- no major loss of mapped free space;
- fewer isolated occupied cells/components or visibly straighter, more
  continuous walls; and
- no new doubled walls or closed false passage at the L corner.

If the map is merely different, or closure improves while navigable structure
gets worse, reject the candidate rather than tuning the matcher around it.

### 3. Sensitivity gate

Only after the first replay passes, test bounded timing alternatives (for
example, a small scan-time scale adjustment) one at a time.  A result that
depends on a narrow magic value is not robust enough for the robot.

### 4. Live opt-in gate

Implement live deskew only after the offline evidence is positive.  The live
node must wait for motion history, expose coverage/correction counters, fall
back safely, and leave the existing raw scan and RF2O paths available.  Verify
the exact LiDAR model, scan direction, scan boundary, rate, mounting transform,
and timestamp convention before enabling it.

### 5. Physical and navigation gates

Run stationary, straight, turn-pair, and closed-route trials, followed by a
held-out hallway.  Preserve closure across repeated runs.  Finally inflate the
map with the documented 0.22 m footprint plus 0.05 m clearance per side and
verify that real passages remain connected before autonomous navigation.

## Stop conditions and fallback

Stop pursuing this implementation if corrected replay cannot beat the strict
baseline on the same capture, if results are highly sensitive to uncertain
timing, or if correction damages closure.  The safe fallback is guarded-strict
mapping with slower turns and deliberate pauses; scan filtering remains a
diagnostic, not the production solution.

## Reproducibility

Every candidate run records the source commit, database and parameter hashes,
laser transform, deskew flag, timing diagnostics, logs, map hashes, and archive
hash.  Baselines are retained and never overwritten.
