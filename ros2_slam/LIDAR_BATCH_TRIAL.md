# Bounded lidar batching: one acquisition comparison

2026-11-17. Goal: remove avoidable serial-call overhead from the existing normal
scan path before attempting a larger native-driver migration. This is a candidate,
not a claim that the Jetson's backlog cause has been proven or fixed.

## Evidence and choice

imu_repair_stationary_01 hit 3082/3000 buffered bytes. Driver recovery produced a
1.2-second scan gap; RF2O correctly rejected its expired reference and exited.
IMU timestamps remained continuous. The driver made 30,183 serial response reads
in that short run; normal measurements are five bytes each. The current Python
generator also pauses serial consumption while the bridge publishes a scan.

Native C++ acquisition remains an alternative, but its exact installed revision
is not present in the laptop checkout. A switch requires auditing its scan order,
timing, parameters and frame convention before using it with the corrected stack.
A worker thread adds scheduling/ownership/shutdown complexity and still retains
thousands of tiny reads. Measured bridge conversion/publication took at most
15 ms, so threading alone is not an established solution to the observed gap.

The smaller candidate reads 200 bytes (40 normal packets) at a time, then uses the
installed vendor decoder unchanged. At the configured 256000 baud, 200 bytes
represent at least 7.8 ms of 8-N-1 wire time; a batch can wait longer if the source
or scheduling stalls. The existing finite serial timeout remains in force.

Original 3000-byte overflow threshold and stop/start recovery remain. Pending
software bytes count toward the limit at scan boundaries. Short reads, malformed
packets and unexpected descriptors fail rather than fabricating data. Following
a restart, partial scans are discarded until a new full revolution is assembled.
There is no unbounded queue, new thread, resynchronization heuristic or relaxed
RF2O/fusion/motion timeout. Default reader remains the previous blocking mode;
candidate is selected explicitly with --lidar-reader batched.

Header timestamps still describe host publication, as in the baseline. This does
not solve acquisition-time reconstruction or motion deskew. Batching adds a small
bounded software buffer, but its actual latency must be measured on the Jetson.

## Local verification and limits

Tests cover byte order, exact read counts, partial reads, decoder failures,
overflow boundary, prefetched-byte accounting, discarded restart fragments,
runtime policy validation, Python 3.6 syntax and no-hardware contract failure.
Using the actual vendor source copied from the Jetson with a simulated serial
stream, both readers produced identical two full scans (800 measurements):
801 serial reads became 21. A separate 400-packet test uses 10 reads instead of
400. These are syscall-count/transport tests, not hardware throughput guarantees.
The real vendor decoder's known packets/interface are checked on the Jetson
before any trial hardware is opened. No ROS or RF2O C++ code was changed.

References checked:
- Inspected installed source: laptop experiments/rplidar_driver_inspection_01.txt.
- Vendor implementation: https://github.com/Roboticia/RPLidar/blob/master/rplidar.py
- Serial read semantics: https://pyserial.readthedocs.io/en/latest/pyserial_api.html
- Native alternative: https://github.com/Slamtec/rplidar_ros/tree/ros2

## Jetson: pull and run once

Close any separate lidar or IMU probes. Keep the IMU in the tested mounting and
the robot still in the same environment. No deploy script or colcon build needed:
the runner snapshots these Python sources into RAM. Keep still during startup
and 120 seconds after MAPPING SESSION READY, then Q. Motors are disabled by the
stationary phase. Do not proceed to driving based on a timing PASS alone.

```bash
(
set -e
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
git log -1 --oneline
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/rf2o_laser_odometry/share/rf2o_laser_odometry/local_setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
python3 ros2_slam/run_hallway_trial.py lidar_batch_imu_stationary_01 --phase stationary --odom imu_yaw --rf2o-log-level warn --rf2o-deadband off --lidar-reader batched
)
```

Copy from laptop PowerShell regardless of PASS/FAIL:

```powershell
scp jetbot@100.86.57.31:/home/jetbot/autochair_ros2_ws/experiments/lidar_batch_imu_stationary_01_bundle.tar.gz "C:\Senior Year\Senior Design\jetbot_analysis\experiments\lidar_batch_imu_stationary_01_bundle.tar.gz"
```

If the pre-launch decoder check or imports fail, no hardware trial is started;
send the terminal output (there may be no bundle). If runtime fails, copy the
automatic bundle; if archiving fails, keep the printed /dev/shm directory and
send the final output. Do not repeat an unchanged failed run.

## Decision after this run

Verify batched runtime marker, read/packet counts, max pending bytes, batch-read
and cycle timing, overflow/recovery/partial-discard counts, scan gaps, raw/fused
age and uniqueness, plus stationary heading/translation. Read count reduction
alone is not success. Need a completed run with freshness within existing bounds
and no recovery gap. Then assess accuracy independently: provisional <=2 degree
heading and <=5 cm position excursion screens. Prior direct RF2O wandered 10 cm;
this acquisition change does not claim to fix that translation problem.

If overflow persists, use new backlog and batch timing evidence to decide on
native acquisition/system-load isolation. Do not enlarge buffers or repeat
speculative IMU tuning. Keep this to one evidence-producing candidate cycle.
To select the previous reader explicitly use --lidar-reader blocking with a new
trial name; no source rollback or rebuilding is necessary. Do not run a second
baseline automatically: the existing failed baseline is already archived.
