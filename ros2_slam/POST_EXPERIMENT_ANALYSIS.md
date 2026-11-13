# Post-experiment analysis and highest-value next step

Historical analysis from November 2. For the current November 4 deployment and
test procedure, follow [LAB_EXPERIMENT_PLAN.md](LAB_EXPERIMENT_PLAN.md).
The heading-aware gate has since passed its handoff test (26 accepts, 27 Karto
states), but the physical hallway map remained inaccurate. An audit also found
that SMapper ignored YAML overrides. The current repair reads and verifies all
28 internal mapper settings, preserves a baseline, and tests search width alone
with the `narrow` profile. Earlier landmark and two-parameter tuning proposals
are superseded. The required test environment remains the unchanged hallway.

## Executive decision

Do not spend another lab run comparing `legacy` with `euclidean`, and do not
tune loop-closure or odometry-penalty parameters yet. The existing data already
establish that the legacy distance expression is invalid, while the first
Euclidean correction is being undermined by a second scan gate and by the
wrapper's omission of heading.

The next candidate is `euclidean_heading`. It makes the wrapper mirror Karto's
downstream rule exactly:

- process after at least 0.500 m of Euclidean translation, **or**
- process after at least 0.500 rad (28.6 degrees) of rotation.

The historical `legacy` and `euclidean` modes remain available so all existing
results remain reproducible.

## Which runs are valid

Primary comparison:

| Run | Classification |
|---|---|
| `elbow_legacy_01_20261102_105642` | valid legacy |
| `elbow_legacy_02_20261102_112157` | valid legacy |
| `elbow_euclidean_01_retry_20261102_111158` | valid Euclidean |
| `elbow_euclidean_02_20261102_111833` | valid Euclidean |
| `elbow_euclidean_01_20261102_110246` | exclude: aborted and runtime legacy |
| `elbow_euclidean_01_20261102_110339` | exclude: requested Euclidean, runtime legacy |
| `elbow_euclidean_02_20261102_111749` | exclude: aborted startup |

The two valid runs per mode are descriptive evidence, not a statistically
powered study. Manual differences in the path and final pose also mean that
start-to-final estimator displacement must not be called physical error.

## What the valid runs show

| Metric | Legacy (n=2) | Euclidean (n=2) |
|---|---:|---:|
| Mean RF2O start-to-final displacement | 0.452 m | 0.837 m |
| Mean SLAM start-to-final displacement | 0.458 m | 0.672 m |
| Mean absolute SLAM heading closure | 13.8 deg | 18.9 deg |
| Mean wrapper gate accepts | 43.5 | 14.0 |
| Mean actual Karto pose-graph states | 13.5 | 9.0 |
| Mean sequential-match response | 0.361 | 0.305 |
| Accepted loop closures | 0 | 0 |

The Euclidean mode reduced the number of wrapper accepts and translational
corrections, but its maps remained visibly ghosted and distorted. It did not
produce a successful loop closure. The legacy maps were denser but also
smeared. Neither mode is a satisfactory final mapping configuration.

The sequential matches also became less reliable in the Euclidean sample:
4 of 16 responses were below 0.20, compared with 2 of 25 legacy responses.
Euclidean reduced mean translational match corrections, but increased mean
heading corrections. This is consistent with a graph that is too sparse at the
turns, not evidence that the invalid legacy metric should be restored.

The only external final-pose observation was approximately 6 in (0.152 m) to
the right for Euclidean run 01. That run reported 0.674 m of SLAM closure and
13.9 degrees of heading closure. It is evidence that the run was not accurate,
but it is not a full error vector because longitudinal and heading ground truth
were not measured.

## Root causes, ordered by confidence and value

### 1. The legacy expression is mathematically invalid (confirmed)

The legacy code uses a coordinate-dependent cross-term instead of `dy * dy`.
Its output changes with the robot's absolute coordinates, so it cannot be a
valid distance metric. The valid legacy runs admitted scans after only
0.058-0.069 m in some locations even though the configured travel distance is
0.500 m.

### 2. The corrected wrapper and Karto disagree about distance (confirmed)

The first Euclidean wrapper uses `0.8 * minimum_travel_distance^2`, which is a
0.447 m displacement threshold for the current 0.500 m setting. It updates its
reference pose as soon as that threshold is crossed. Karto then applies the
full 0.500 m threshold and rejects the scan. The wrapper subsequently measures
from the rejected pose, so a scan often has to travel roughly another 0.447 m
before Karto sees it. This explains why each Euclidean route produced only nine
pose-graph states over an approximately 8-9 m trajectory.

### 3. The wrapper blocks pure-turn scans (confirmed)

Karto's intended rule is distance **or** heading. The outer wrapper checks only
translation, so it can reject every scan during a nearly in-place 90- or
180-degree turn before Karto can apply its heading rule. Earlier experiment
checkpoints show SLAM remaining numerically identical to RF2O across large
turns, which is the expected symptom.

This matters especially in a hallway: rotations expose the corner and side
walls that constrain heading, while long straight segments between parallel
walls are geometrically ambiguous.

### 4. The current graph is too sparse for the configured loop search (confirmed)

The Euclidean runs ended with nine Karto states while
`loop_match_minimum_chain_size` is 10. No run accepted a loop closure. More
valid turn and distance states must reach the graph before it is sensible to
relax loop thresholds.

### 5. RF2O drift is real but is not the first parameter to tune (supported)

The representative floor stationary run drifted only about 0.03 m and 0.6
degrees over two minutes. The three-hour unattended run accumulated substantial
drift, but gradually. Motion runs show variable return error and turn-induced
translation, so RF2O remains a limitation. However, scan selection is currently
preventing SLAM from making the corrections it was designed to make. Fix that
interface before changing RF2O or weakening scan-matcher constraints.

### 6. Empty-hallway symmetry is a remaining environmental risk (supported)

Parallel walls create many similar scan matches. If the corrected candidate
processes enough turn scans but the map is still duplicated, the fastest next
test is a rigid asymmetric 2D-LiDAR landmark near the elbow or turnaround (for
example, a box-and-chair arrangement outside the driving path), not a broad
parameter sweep.

## Changes implemented for the next test

1. Added `euclidean_heading`, using the exact Karto distance-or-heading
   thresholds.
2. Added acceptance reasons and both thresholds to `SCAN_GATE_ACCEPT` logs.
3. Preserved `legacy` and the original `euclidean` behavior unchanged.
4. Changed the scan/TF preflight to require a passing rolling 20-scan window,
   with up to 60 attempts. This allows startup to stabilize while still failing
   a persistent synchronization problem.
5. Added `run_status`, runtime gate mode, and preflight results to
   `results.json`, including early-abort sessions.
6. Added `analysis/analyze_experiments.py`, a standard-library-only analyzer
   that classifies invalid runs and reports gate, Karto, trajectory, closure,
   and warning metrics.

## Next lab protocol

### Physical setup

1. Put the JetBot on the floor in the same hallway.
2. Tape a cross for the center of the starting axle and a separate straight
   heading line. Returning only “near” the start is not enough.
3. Tape the 3 ft, 6 ft, and 9 ft stations and the turn location.
4. Keep people and movable objects out of the scan area during a run.
5. Photograph the marks and record physical final longitudinal, lateral, and
   heading error after every run.

### Candidate runs

Run two repetitions without changing any YAML parameter:

```bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py elbow_heading_01 euclidean_heading
python3 mapping_session.py elbow_heading_02 euclidean_heading
```

Use the same route and low speed for both runs. Stop at every named checkpoint
before pressing `N`:

1. `start`
2. `straight_3ft`
3. `straight_6ft`
4. `before_90`
5. `after_90`
6. `far_end`
7. `after_180`
8. `return_corner`
9. `after_return_90`
10. `original_position`

Press `Q` only after the robot is aligned to the taped final heading. The
program adds its own `final` checkpoint after settling.

## Pass criteria and decision tree

A run is technically valid only if:

- requested and runtime mode both report `euclidean_heading`;
- the rolling scan/TF window passes at 90% or better;
- the stationary preflight passes;
- the run reaches `run_status: complete`.

The candidate is promising if both repetitions show:

- at least 18 Karto states on the full route;
- processed-state efficiency of at least 90%, calculated as
  `KARTO_SCAN_FINAL / (SCAN_GATE_ACCEPT + 1)`;
- heading-triggered accepts during every turn;
- SLAM closure within 0.25 m and 10 degrees when the robot is physically
  returned to the marked start;
- a single consistent elbow and wall pair rather than duplicated/rotated walls.

Then use this decision tree:

- **Enough states and a clean map:** promote `euclidean_heading` to the default
  and run a larger mapping route.
- **Enough states but ghosted parallel walls:** add one rigid asymmetric
  LiDAR-visible landmark and repeat once. If that helps, observability is the
  limiting factor; then evaluate stronger odometry priors as a separate change.
- **Fewer than 15 states or efficiency below 90%:** stop tuning mapping
  parameters. Inspect the gate-to-Karto handoff and logs because the selection
  pipeline is still not behaving as designed.
- **Repeated RF2O physical return error above 0.5 m:** prioritize RF2O and scan
  timing/motion-distortion work before loop tuning.
- **No loop closure despite a clean local map:** investigate loop candidate
  generation and chain size next; do not lower match-quality thresholds merely
  to force a closure.

## Reproducible analysis command

After copying the experiment folders to the laptop:

```powershell
wsl python3 ./ros2_slam/analysis/analyze_experiments.py `
  "../experiments" `
  --name "elbow_heading_*"
```

The report intentionally calls the start-to-final values “closure,” not
“error,” unless physical ground truth is supplied.
