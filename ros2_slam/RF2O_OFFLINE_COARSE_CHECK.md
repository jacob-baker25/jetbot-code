# Saved-scan check of the coarse-level rejection hypothesis

2026-11-17. Run from home over SSH with all mapping/probe sessions stopped.
No live experiment, sensor access, motor commands, colcon build or deployment.

Both rf2o_input runs stopped with RF2O_REFERENCE_EXPIRED despite continued scan
delivery. The movement-labelled run has zero moving trajectory rows; treat it as
stationary, not as measured movement. The first recorder stopped before the first
run's failure; the second raw clip includes the later run's failure window.

Six saved rejected pairs report rank_deficient_system. A separate NumPy
reconstruction of the unwarped pyramid found rank two at the coarsest (23-bin)
level but rank three at all finer levels. This suggests rejecting a deficient
coarse level prematurely discards usable detail. It does not prove the entire
recorded motion/filter state or establish a drift correction. Local actual-C++
replay was blocked by WSL service error 0x80072747; do not report it as passed.

The offline tool compiles the unchanged RF2O core and a temporary candidate
against existing ROS interface stubs and actual installed Eigen headers. Candidate
skips a rank-deficient non-final pyramid level, preserves its identity transform,
and requires a successfully solved finest level. Rank checks, other invalid-input
checks, 0.35 s reference expiry and deadband policy remain intact. Installed code
is never replaced. Both variants run saved-pair replay and existing identical-scan,
signed slow-translation/turn, rollback, invalid-input and expiry regressions.

Replay restores each pair's saved motion prior, but initializes its reference
afresh. It is not an exact reconstruction of the full original trajectory state.
Hypothesis supported means recorded rejections reproduced and candidate accepted
the pairs with both regression suites passing. Inspect displacement/heading too;
acceptance alone is not a position-accuracy test. No automatic deployment follows.

Local verification so far: seven Python tests pass (nested archive reading without
extraction, input validation, prior/null preservation, guarded temporary patch,
compiler-failure reporting, Python 3.6 syntax). All six real pairs load correctly.
Actual C++ compilation/comparison remains pending on the Jetson.

## SSH: update diagnostics only

```bash
(
set -e
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
git log -1 --oneline
test -f ros2_slam/analysis/check_rf2o_coarse_candidate.py
test -s ~/autochair_ros2_ws/experiments/rf2o_input_batch_01_all.tar.gz
)
```

If the pull fails, stop and send its output. Do not run a deployment script.

## SSH: one offline comparison, then package its report even on failure

```bash
(
set -e
cd ~/jetbot-code-f03243c
OUT="$HOME/autochair_ros2_ws/experiments/rf2o_offline_coarse_01"
test ! -e "$OUT"
test ! -e "${OUT}.tar.gz"
if python3 ros2_slam/analysis/check_rf2o_coarse_candidate.py \
  ~/autochair_ros2_ws/experiments/rf2o_input_batch_01_all.tar.gz \
  --output "$OUT"
then
  echo "Offline comparison finished; inspect report before any live change."
else
  echo "Offline comparison failed; preserving diagnostics."
fi
test -s "$OUT/report.json"
cat "$OUT/report.json"
tar -czf "${OUT}.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
ls -lh "${OUT}.tar.gz"
)
```

Compilation may take several minutes on the Nano. The tool announces each variant;
compiler output is in the report directory. It has bounded subprocess timeouts.
Temporary compiled files are removed on normal completion/failure; original input
data and installed workspace sources are read-only. An output collision requires
a new name, not deleting an earlier result. No ROS environment sourcing is needed.

Laptop PowerShell:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/rf2o_offline_coarse_01.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\rf2o_offline_coarse_01.tar.gz"
```

Do not rerun either live stationary or movement experiment yet. If the offline
comparison fails, return that report; no sensor or motion troubleshooting is
required to collect it. A successful comparison supports the next guarded code
change, followed by stationary validation; measured movement waits for the lab.
