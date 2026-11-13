# Hallway mapping and MOT0110 preparation — November 5, 2026

## Decision and priority

Tomorrow (November 6), validate the new IMU, then test whether better rotational
information improves the existing hallway mapper. Do not assume fusion fixes
translation ambiguity, wrong scan geometry, or processing latency. Keep the
hallway unchanged; floor measurement marks are ground truth, not lidar features.

Current evidence: narrow elbow closure improved 0.890 -> 0.240 m, but physical
return was within 0.03 m. Straight-run RF2O displacement was 2.636 m versus
nominal 2.7432 m; SLAM radial displacement was 2.742 m but y=-0.302 m. All five
matches shifted negative y (-6,-6,-3,-6,-8 cm). Physical lateral offset and
initial alignment remain unmeasured. Do not label the IMU a proven solution or
claim a constant 19% odometry scale error. Loop closure was not accepted.

## Findings from code audit

1. RF2O publishes pose and twist but does not fill their covariance matrices.
   Its lateral velocity is hard-coded zero. Zero-filled covariance is not
   measured certainty, and zero lateral velocity is a model assumption, not a
   second sensor measurement. Do not blindly fuse every field.
2. Current RF2O publishes odom->base_link. A future fusion filter must replace
   that TF authority, not publish a competing transform. SLAM keeps map->odom.
3. The Python lidar bridge bins angles by integer degree, overwrites duplicate
   bins, stamps at publish time, and sets time_increment from the scan period.
   LaserScan specifies first-ray acquisition time. The sorted bins do not
   establish acquisition order. This needs raw inspection before claiming
   correct scan motion compensation or applying an arbitrary timestamp offset.
4. The bridge reports 360 bins at 1 degree/bin but angle_max=2*pi. RF2O already
   compensates using (N-1)*increment; Karto has 360-degree special handling.
   Record/check this inconsistency, but do not change it in the IMU comparison
   without a separate compatibility test.
5. The current laser static transform assumes x=y=0 and yaw=pi. Measure the
   physical mounting, confirm angle handedness against real walls, and check
   the installed source agrees with this laptop copy. A wrong transform can
   cause turn-dependent error. Neither this nor scan distortion is proven to
   explain the observed maps.
6. Latest TF and latest RF2O were sampled independently. New logging preserves
   TF stamps/age so stale readings can be identified. It deliberately does not
   pretend to make historical samples synchronous.

## Prepared changes

- mapping_session.py appends SLAM TF timestamp/age, map->odom TF timestamp,
  and RF2O age to trajectory.csv; checkpoint SLAM records retain timestamp/age.
- capture_sensors.py passively records /scan, /scan_slam, /odom_rf2o,
  /odometry/filtered, selected Imu topic, /tf and transient-local /tf_static.
  Five-minute/64 MiB limits, 150 MiB free-space reserve, exclusive file creation.
  Includes full received scans and covariances, preserving nonfinite values as
  strings. Best-effort reception is not lossless. JSONL is not rosbag and no
  replay implementation is supplied. Compare load and timing with capture on.
- phidget_probe.py records manufacturer Spatial callbacks without ROS or motor
  control, preserving device timestamp in ms and vendor g/deg/s/gauss units.
  No automatic zeroing, heating, magnetic calibration, firmware update or TF.
- summarize_capture.py produces counts/rates, timing anomalies, scan metadata,
  and raw gyro integrals. The integral is sensor Z, not automatically robot yaw.
  Gyro mean is a bias estimate only when the entire capture is stationary.
- deploy_sensor_diagnostics.sh deploys Python-only files with backups. No rebuild.

## Hardware and software preparation from home

The ordered device is PhidgetSpatial Precision 3/3/3 MOT0110_0. Prefer direct
USB with the matching cable; no VINT hub needed for that connection. Check the
actual connector/cable supplied. Mount rigidly, clear of the lidar scan plane,
with documented axes and measured transform to base_link. Keep wiring secure
and away from drive components. Do not assume magnetic north equals hallway
heading. The optional temperature stabilization is disabled by default; leave
its setting unchanged initially and record warm-up drift.

Before installing anything on the Jetson, run read-only checks over SSH:

```bash
uname -m
getconf GNU_LIBC_VERSION
python3 --version
python3 -m pip show Phidget22
ldconfig -p | grep -i phidget
dpkg-query -W libusb-1.0-0-dev gcc make 2>/dev/null
df -h /
```

Native libphidget22 and Python Phidget22 bindings are both needed. Recent PyPI
ARM64 wheels advertise glibc>=2.28; the Jetson's actual libc must be checked.
Do not assume the latest wheel runs on this old OS. The manufacturer supports
a native source build with libusb development headers plus Python bindings.
Select and pin compatible versions after these checks; retain exact versions
and build instructions with results. Do not upgrade JetPack/Ubuntu/Python or
run a broad apt upgrade to obtain this sensor. Inspect simulated apt changes
before installing dependencies, given the previously observed NVIDIA upgrades.
Do not run an uninspected remote installation script. Set vendor udev rules
for normal-user USB access using the manufacturer's Linux instructions.

The probe is prepared but vendor-library import, hardware attachment, ROS QoS,
and performance must be checked on Jetson. No working Eloquent IMU driver or
fusion installation is claimed from local unit tests.

## Deployment and smoke test (can do from home; no driving)

After these changes are pushed:

```bash
cd ~/jetbot-code-f03243c
git status --short
git pull --ff-only origin experiment/scan-gate-diagnostics
bash ros2_slam/deploy_sensor_diagnostics.sh
source /opt/ros/eloquent/setup.bash
source ~/autochair_ros2_ws/install/slam_toolbox/share/slam_toolbox/local_setup.bash
cd ~/autochair_ros2_ws/local_nodes
python3 capture_sensors.py --help
python3 phidget_probe.py --help
mkdir -p ~/autochair_ros2_ws/experiments/sensor_captures
```

Use two SSH terminals, source ROS in both. Terminal A, start the recorder:

```bash
python3 ~/autochair_ros2_ws/local_nodes/capture_sensors.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/timing_smoke_01.jsonl --seconds 90
```

Terminal B, run stationary only:

```bash
cd ~/autochair_ros2_ws/local_nodes
python3 mapping_session.py timing_smoke_01 euclidean_heading narrow
```

After readiness, wait ten seconds, then Q without driving. In A, Ctrl-C after
the session stops if needed. Confirm scan, odometry and TF counts are nonzero;
an absent IMU is expected now. Summarize:

```bash
python3 ~/jetbot-code-f03243c/ros2_slam/analysis/summarize_capture.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/timing_smoke_01.jsonl
```

Inspect new TF age fields; negative ages can reflect future-dated transforms,
not automatically a clock fault. Missing data or bad ages require investigation.
Repeated filenames intentionally fail rather than overwrite prior evidence.

## Tomorrow's order and decision gates

### A. IMU alone, approximately 20–30 minutes after driver readiness

1. Plug in, confirm USB enumeration, library access and serial number. Leave
   stationary a few minutes, then capture 60 seconds on the mounted robot:

```bash
python3 ~/autochair_ros2_ws/local_nodes/phidget_probe.py \
  ~/autochair_ros2_ws/experiments/sensor_captures/imu_stationary_01.jsonl --seconds 60
```

2. Summarize with summarize_capture.py. Repeat stationary capture after warm-up.
   Measure mean gyro bias, variability, timestamp monotonicity and missing
   intervals. Provisional screening targets: no timestamp reversals/queue
   drops; no repeated >0.25 s sample gaps; <2 degrees raw stationary Z integral
   per minute. These are project gates, not manufacturer accuracy guarantees.
3. Separate short captures of measured +90, -90, +180 degree rotations and a
   return to original direction. Use a floor angle reference, not commanded
   motor duration. Z-up sensor rotation counterclockwise viewed from above
   should agree with ROS positive yaw after the mounting transform. Target
   <=3 degrees error for 90-degree turns, <=5 for 180, repeated both directions.
   Raw integral includes bias during pauses; measure/subtract stationary bias
   in analysis before interpreting longer trials, retaining raw data.
4. Observe whether robot power/motor operation changes magnetic readings; do
   not use magnetometer-based heading merely because a quaternion is available.
   IMU-derived orientation and AHRS orientation are different modes.

If A fails, fix units, axes, drift, timestamps or compatibility. Do not enable
fusion and obscure the issue behind a full mapping experiment.

### B. Establish lidar and physical reference, approximately 15–20 minutes

- Align the axle reference and robot heading with a measured hallway line;
  measure final lateral offset and heading, not just distance traveled.
- Measure actual hallway width in at least three locations, straight travel
  marks, and corner angle. Photograph/annotate reference measurements if useful.
- Stationary capture near the existing walls: inspect raw scan wall straightness,
  spacing, handedness, missing bins and outliers. Check lidar mounting level
  and visibility of robot chassis/cables. No added scene landmarks.
- Repeat the nine-foot straight run with the recorder and settled paired
  checkpoints. This becomes the raw-data baseline, not another blind full elbow.
  Keep default speed, gate and narrow matching settings unchanged. If raw scan
  geometry itself is wrong, repair/validate that before fusion comparisons.

### C. Fusion design and implementation gate

Only after A/B, adapt a compatible ROS2 driver or bridge to publish sensor_msgs/Imu
with rad/s, m/s^2, correct frame and timestamps. Preserve hardware and receipt
times. Mark orientation unavailable when not provided; all-zero covariance is
unknown, not perfect accuracy. Do not silently use Phidget deg/s as rad/s.

Start with RF2O forward velocity and IMU yaw rate in a 2D estimator; assess
whether a nonholonomic lateral-velocity constraint is justified by actual skid.
Do not fuse correlated RF2O pose and twist as independent evidence. Assign
empirical/conservative covariances and verify RF2O velocity units/timing first.
Gyro-only yaw drifts; do not claim absolute heading. Start without acceleration
integration or magnetic heading. Retain RF2O-only fallback and log both outputs.

Filter alone owns odom->base_link; disable RF2O TF when selecting fused mode.
SLAM remains map->odom and uses the fused odometry transform. Do not feed SLAM's
corrected pose back as independent input to the local filter. Upstream fusion
does not directly constrain Karto's global translational corrections.

No guessed covariance EKF YAML or unvalidated driver is enabled by this change.
This stage requires measured inputs and Eloquent-compatible package verification.

### D. Controlled evaluation, only if fusion is ready

Same route/environment/speed/settings and recorder in both modes. One RF2O-only
and one fused straight run first; compare intermediate longitudinal/lateral
errors, heading error, sample age, and matching corrections. If promising,
one elbow per mode; then repeat the pair in reverse order to reduce order bias.
If time is short, prioritize valid A/B evidence over an unverified fused demo.

## What makes a map useful

Before claiming navigation readiness, evaluate: wall spacing against tape
measure, wall duplication/thickness, hallway straightness, corner angle,
checkpoint pose error, and repeatability. Endpoint closure and match scores
alone are insufficient. Initial screening targets, to be revised against robot
footprint and required clearance: measured wall spacing error <=10 cm, straight
endpoint lateral error <=10 cm, heading error <=3 degrees, and repeatable elbow
return error <=15 cm. No doubled walls intruding into intended traversable space.
These are provisional project targets, not a safety certification or navigation
guarantee. Do not change map resolution to make misaligned scans look better.

## Sources and limits

- [Exact MOT0110 product and guide](https://www.phidgets.com/?prodid=1205)
- [Manufacturer Linux setup](https://cdn.phidgets.com/docs/OS_-_Linux)
- [Official Python distribution](https://pypi.org/project/phidget22/)
- [Phidget Spatial units and callback example](https://www.phidgets.com/education/learn/device-tutorials/spatial-sensor-1102/)
- [Spatial orientation modes](https://cdn.phidgets.com/docs/Spatial_Guide)
- [Eloquent LaserScan definition](https://raw.githubusercontent.com/ros2/common_interfaces/eloquent/sensor_msgs/msg/LaserScan.msg)
- [Fusion frames/covariance guidance](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst)

Modern documentation guides design, not proof of binary compatibility with
Eloquent/Python3.6/JetPack4.5. Hardware tests and native-library checks remain.
