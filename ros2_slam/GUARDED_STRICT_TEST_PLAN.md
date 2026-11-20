# Guarded-strict mapping candidate

## Decision and evidence

Test one change: reduce the sequential scan matcher's maximum translation
correction from 0.12 m to 0.05 m. Keep the response threshold, rotation limit,
odometry, Cartesian lidar resampling, scan cadence, nearby-chain matching, loop
closure, and optimizer behavior unchanged.

The 0.05 m boundary separates the measured cases without being tuned to a
single exact value:

- The successful guarded turn pair accepted five matches; its largest
  translation correction was 0.0361 m.
- The mixed route accepted suspicious corrections of 0.0806 m and 0.0825 m.
- A 0.05 m limit leaves about 0.014 m above the observed successful maximum
  while rejecting both suspicious corrections.

This is a candidate, not a claimed fix. It can improve robustness when a
hallway scan has an ambiguous translation match. It can also make the map
follow odometry more closely after real wheel slip or a genuinely larger
alignment error. It does not repair odometry bias, lidar geometry, or false
constraints added by nearby-chain or loop matching.

## Instrumentation

The candidate records, without changing their decisions:

- nearby-chain match attempts and accepted corrections;
- coarse and fine loop-closure attempts and accepted corrections;
- the largest pose change applied by each graph optimization;
- the exact pinned source commit in exported-source trials.

These records distinguish a sequential-match improvement from a false nearby
link, loop closure, or later graph optimization.

## Qualification sequence

1. Run the Python 3.6-compatible unit tests and the recorded-result separation
   test.
2. Build the complete `slam_toolbox` overlay from the pinned commit on the
   Jetson. Do not modify the installed workspace or the earlier guarded
   overlay.
3. Run a stationary smoke capture to verify the compiled profile, process
   startup/shutdown, archive, transport timing, and new diagnostics.
4. Repeat the same marked mixed route used for the guarded run, with the same
   odometry, lidar, route, checkpoints, and approximate driving procedure.
5. If the first moving run meets every predeclared criterion, repeat it once
   before adopting the profile.

## Predeclared moving-run criteria

- Final SLAM closure error no greater than 0.10 m.
- Final SLAM heading closure error no greater than 5 degrees.
- SLAM-to-fused-odometry disagreement no greater than 0.10 m at each marked
  checkpoint.
- No timing/configuration/preflight failure and a complete saved bundle.
- The map must retain the physical corner and show the real hallway opening;
  a numerically closed trajectory with a false wall is not a pass.

Stop after one moving run if timing fails, the profile is not verified as
`guarded_strict`, motion behavior is unsafe, or the map/trajectory clearly
regresses. Do not tune another threshold during the same comparison.
