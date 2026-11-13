#!/usr/bin/env python3

import csv
import hashlib
import json
import math
import os
import select
import signal
import shlex
import shutil
import subprocess
import sys
import termios
import time
import tty
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from jetbot import Robot
from ament_index_python.packages import get_package_share_directory
from matcher_profiles import PROFILE_FILES, expected_parameters, check_karto_config


# ============================================================
# PATHS
# ============================================================

WORKSPACE = os.path.expanduser("~/autochair_ros2_ws")
LOCAL_NODES = os.path.join(WORKSPACE, "local_nodes")
EXPERIMENT_ROOT = os.path.join(WORKSPACE, "experiments")


# ============================================================
# FRAMES
# ============================================================

MAP_FRAME = "map"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
LASER_FRAME = "laser_frame"


# ============================================================
# HARDWARE / TF
# ============================================================

LASER_Z = 0.1778
LASER_YAW = 3.14159265
LIDAR_ANGLE_CONVENTION = "hardware_clockwise_to_ros_counterclockwise"


# ============================================================
# MOTOR SETTINGS
# ============================================================

MOTOR_DIR_L = -1.0
MOTOR_DIR_R = -1.0

BIAS_L = 1.0
BIAS_R = 1.0

DRIVE_SPEED = 0.25
TURN_SPEED = 0.18

WATCHDOG_SECONDS = 0.15


# ============================================================
# PREFLIGHT SETTINGS
# ============================================================

READY_TIMEOUT_SECONDS = 60.0

INITIAL_SETTLE_SECONDS = 5.0

SCAN_TF_TEST_COUNT = 20
SCAN_TF_MIN_SUCCESS_RATE = 0.90
SCAN_TF_MAX_ATTEMPTS = 60
SCAN_TF_TIMEOUT_SECONDS = 15.0

STATIONARY_TEST_SECONDS = 5.0

MAX_STATIONARY_RF2O_DISTANCE_M = 0.15
MAX_STATIONARY_RF2O_HEADING_DEG = 7.5

MAX_STATIONARY_SLAM_DISTANCE_M = 0.15
MAX_STATIONARY_SLAM_HEADING_DEG = 7.5

CHECKPOINT_SETTLE_SECONDS = 0.5
FINAL_SETTLE_SECONDS = 5.0
CHECKPOINT_MAX_AGE_SECONDS = 0.5
CHECKPOINT_FRESH_WAIT_SECONDS = 2.0


# ============================================================
# HELPERS
# ============================================================

def clamp(value, lo=-1.0, hi=1.0):

    return max(
        lo,
        min(hi, value)
    )


def quaternion_to_yaw(x, y, z, w):

    siny_cosp = 2.0 * (
        w * z
        +
        x * y
    )

    cosy_cosp = 1.0 - 2.0 * (
        y * y
        +
        z * z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


def wrap_degrees(angle):

    while angle > 180.0:
        angle -= 360.0

    while angle <= -180.0:
        angle += 360.0

    return angle


def distance_2d(a, b):

    dx = b["x"] - a["x"]
    dy = b["y"] - a["y"]

    return math.sqrt(
        dx * dx
        +
        dy * dy
    )


def timestamp_string():

    return time.strftime(
        "%Y%m%d_%H%M%S"
    )


# ============================================================
# MAIN SESSION NODE
# ============================================================

class MappingSession(Node):

    def __init__(
        self,
        experiment_name,
        scan_distance_gate_mode,
        matcher_profile="baseline",
        odom_mode="rf2o"
    ):

        super().__init__(
            "mapping_session"
        )

        self.experiment_name = (
            experiment_name
        )

        self.scan_distance_gate_mode = (
            scan_distance_gate_mode
        )
        self.matcher_profile = matcher_profile
        self.odom_mode = odom_mode
        self.effective_karto_parameters = {}

        self.session_name = (
            "{}_{}".format(
                experiment_name,
                timestamp_string()
            )
        )

        self.experiment_dir = (
            os.path.join(
                EXPERIMENT_ROOT,
                self.session_name
            )
        )

        os.makedirs(
            self.experiment_dir,
            exist_ok=True
        )

        # Run from an immutable per-session copy of the installed profile.
        # Missing installation fails before initializing the motor interface.
        profile_source = os.path.join(
            get_package_share_directory("slam_toolbox"), "config",
            PROFILE_FILES[matcher_profile]
        )
        self.slam_params_file = os.path.join(self.experiment_dir, "slam_params.yaml")
        shutil.copyfile(profile_source, self.slam_params_file)
        with open(self.slam_params_file, "rb") as profile_file:
            self.slam_params_sha256 = hashlib.sha256(profile_file.read()).hexdigest()
        self.odom_source_sha256 = {}
        odom_source_names = (
            (
                "phidget_imu_publisher.py",
                "imu_planar.py",
                "imu_yaw_odometry.py",
                "imu_yaw_odometry_core.py",
            )
            if self.odom_mode == "imu_yaw"
            else ()
        )
        for source_name in odom_source_names:
            source_path = os.path.join(LOCAL_NODES, source_name)
            if os.path.isfile(source_path):
                with open(source_path, "rb") as source_file:
                    self.odom_source_sha256[source_name] = hashlib.sha256(
                        source_file.read()
                    ).hexdigest()

        self.session_start_unix = (
            time.time()
        )

        self.session_end_unix = None
        self.run_status = "created"
        self.runtime_gate_mode = None
        self.preflight_results = {
            "karto_config": None,
            "scan_tf": None,
            "stationary": None,
            "odom_source": None,
        }

        self.robot = Robot()

        self.speed = DRIVE_SPEED
        self.turn_speed = TURN_SPEED

        self.moving = False
        self.last_motor_command = 0.0

        self.last_odom = None
        self.last_raw_odom = None
        self.checkpoint_failures = []
        self.odom_message_count = 0
        self.last_map = None

        self.checkpoints = []

        self.processes = []
        self.log_files = []
        self.process_log_paths = {}

        self.trajectory_sample_count = 0
        self.trajectory_path = os.path.join(
            self.experiment_dir,
            "trajectory.csv"
        )
        self.trajectory_file = open(
            self.trajectory_path,
            "w",
            newline=""
        )
        self.log_files.append(
            self.trajectory_file
        )
        self.trajectory_writer = csv.writer(
            self.trajectory_file
        )
        self.trajectory_writer.writerow([
            "time_unix",
            "elapsed_s",
            "rf2o_stamp_s",
            "moving",
            "last_checkpoint",
            "rf2o_x_m",
            "rf2o_y_m",
            "rf2o_yaw_deg",
            "slam_x_m",
            "slam_y_m",
            "slam_yaw_deg",
            "map_to_odom_x_m",
            "map_to_odom_y_m",
            "map_to_odom_yaw_deg",
            "slam_minus_rf2o_x_m",
            "slam_minus_rf2o_y_m",
            "slam_minus_rf2o_yaw_deg",
            "slam_tf_stamp_s",
            "slam_tf_age_s",
            "map_to_odom_tf_stamp_s",
            "rf2o_age_s",
        ])

        self.stack_stopped = False

        self.stdin_fd = None
        self.old_terminal_settings = None

        # --------------------------------------------
        # TF
        # --------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = (
            TransformListener(
                self.tf_buffer,
                self
            )
        )

        # --------------------------------------------
        # Preflight scan/TF test
        # --------------------------------------------

        self.scan_tf_test_active = False
        self.scan_tf_test_count = 0
        self.scan_tf_test_success = 0
        self.scan_tf_test_fail = 0
        self.scan_tf_recent_results = deque(
            maxlen=SCAN_TF_TEST_COUNT
        )

        # --------------------------------------------
        # Subscribers
        # --------------------------------------------

        self.create_subscription(
            Odometry,
            "/odom_rf2o",
            self.odom_callback,
            10
        )

        if self.odom_mode == "imu_yaw":
            self.create_subscription(
                Odometry, "/odom_rf2o_raw", self.raw_odom_callback, 10
            )

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            "/scan_slam",
            self.delayed_scan_callback,
            qos_profile_sensor_data
        )

        self.create_timer(
            0.1,
            self.record_trajectory_sample
        )


    # ========================================================
    # ROS CALLBACKS
    # ========================================================

    def odom_callback(self, msg):

        self.odom_message_count += 1

        q = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.last_odom = {
            "stamp_s": float(
                msg.header.stamp.sec
                +
                msg.header.stamp.nanosec
                *
                1e-9
            ),
            "x": float(
                msg.pose.pose.position.x
            ),
            "y": float(
                msg.pose.pose.position.y
            ),
            "yaw_deg": float(
                math.degrees(yaw)
            )
        }


    def map_callback(self, msg):

        self.last_map = msg

    def raw_odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.last_raw_odom = {
            "stamp_s": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw_deg": math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w)),
        }


    def delayed_scan_callback(self, msg):

        if not self.scan_tf_test_active:
            return

        if self.scan_tf_test_count >= SCAN_TF_MAX_ATTEMPTS:
            return

        self.scan_tf_test_count += 1

        stamp = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec
        )

        success = False

        try:

            self.tf_buffer.lookup_transform(
                ODOM_FRAME,
                msg.header.frame_id,
                stamp
            )

            self.scan_tf_test_success += 1
            success = True

        except Exception:

            self.scan_tf_test_fail += 1

        self.scan_tf_recent_results.append(
            success
        )


    def record_trajectory_sample(self):

        if self.last_odom is None:
            return

        slam = self.get_slam_pose()
        map_to_odom = self.lookup_pose(
            MAP_FRAME,
            ODOM_FRAME
        )

        if slam is None or map_to_odom is None:
            return

        rf2o = self.last_odom
        last_checkpoint = ""

        if self.checkpoints:
            last_checkpoint = self.checkpoints[-1][
                "label"
            ]

        self.trajectory_writer.writerow([
            time.time(),
            time.time() - self.session_start_unix,
            rf2o["stamp_s"],
            int(self.moving),
            last_checkpoint,
            rf2o["x"],
            rf2o["y"],
            rf2o["yaw_deg"],
            slam["x"],
            slam["y"],
            slam["yaw_deg"],
            map_to_odom["x"],
            map_to_odom["y"],
            map_to_odom["yaw_deg"],
            slam["x"] - rf2o["x"],
            slam["y"] - rf2o["y"],
            wrap_degrees(
                slam["yaw_deg"]
                -
                rf2o["yaw_deg"]
            ),
            slam["stamp_s"],
            slam["age_s"],
            map_to_odom["stamp_s"],
            self.get_clock().now().nanoseconds / 1e9 - rf2o["stamp_s"],
        ])

        self.trajectory_sample_count += 1

        if self.trajectory_sample_count % 10 == 0:
            self.trajectory_file.flush()


    # ========================================================
    # EXISTING PROCESS CHECK
    # ========================================================

    def check_for_existing_processes(self):

        patterns = [
            "rplidar_python_bridge.py",
            "scan_delay_node.py",
            "phidget_imu_publisher.py",
            "imu_yaw_odometry.py",
            "rf2o_laser_odometry_node",
            "sync_slam_toolbox_node",
            "online_sync_launch.py",
            "static_transform_publisher",
        ]

        try:

            output = subprocess.check_output(
                [
                    "ps",
                    "-eo",
                    "pid,args"
                ],
                universal_newlines=True
            )

        except Exception as exc:

            print(
                "WARNING: Could not inspect "
                "running processes: {}".format(
                    exc
                )
            )

            return True

        found = []

        own_pid = os.getpid()

        for line in output.splitlines():

            parts = line.strip().split(
                None,
                1
            )

            if len(parts) != 2:
                continue

            try:
                pid = int(parts[0])

            except ValueError:
                continue

            if pid == own_pid:
                continue

            command = parts[1]

            for pattern in patterns:

                if pattern in command:

                    found.append(
                        (
                            pid,
                            command
                        )
                    )

                    break

        if not found:
            return True

        print()
        print("=" * 70)
        print(
            "EXISTING MAPPING PROCESSES DETECTED"
        )
        print("=" * 70)

        for pid, command in found:

            print(
                "PID {:>6}: {}".format(
                    pid,
                    command
                )
            )

        print()
        print(
            "Stop the previous mapping stack "
            "before beginning a new experiment."
        )

        print("=" * 70)
        print()

        return False


    # ========================================================
    # PROCESS MANAGEMENT
    # ========================================================

    def start_process(
        self,
        name,
        command
    ):

        log_path = os.path.join(
            self.experiment_dir,
            "{}.log".format(name)
        )

        log_file = open(
            log_path,
            "w"
        )

        self.log_files.append(
            log_file
        )

        self.process_log_paths[name] = (
            log_path
        )

        shell_command = (
            "source /opt/ros/eloquent/setup.bash && "
            "source {}/install/setup.bash "
            ">/dev/null 2>&1 || true; "
            "exec env PYTHONUNBUFFERED=1 {}"
        ).format(
            WORKSPACE,
            command
        )

        process = subprocess.Popen(
            [
                "bash",
                "-lc",
                shell_command
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        self.processes.append(
            (
                name,
                process
            )
        )

        print(
            "Started {:<12} PID {}".format(
                name,
                process.pid
            )
        )

        return process


    def process_is_alive(self, name):

        for (
            process_name,
            process
        ) in self.processes:

            if process_name == name:

                return (
                    process.poll()
                    is None
                )

        return False


    def failed_process_names(self):

        return [
            name
            for name, process in self.processes
            if process.poll() is not None
        ]


    def require_processes_alive(self):

        failed = self.failed_process_names()

        if failed:
            self.stop_robot()
            raise RuntimeError(
                "Required ROS process exited: {}".format(
                    ", ".join(failed)
                )
            )


    def verify_odom_source(self):

        passed = False
        errors = []
        initial_count = self.odom_message_count
        deadline = time.monotonic() + 5.0
        required_new_messages = 5
        qualification_required = self.odom_mode == "imu_yaw"
        qualification_seen = not qualification_required
        while time.monotonic() < deadline:
            self.require_processes_alive()
            rclpy.spin_once(self, timeout_sec=0.1)
            if qualification_required:
                try:
                    with open(
                        self.process_log_paths["imu_yaw_odom"],
                        "r", errors="replace"
                    ) as stream:
                        qualification_seen = "IMU_YAW_ODOM_READY" in stream.read()
                except (IOError, OSError, KeyError):
                    qualification_seen = False
            if (self.odom_message_count - initial_count >= required_new_messages and
                    qualification_seen):
                break
        observed = self.odom_message_count - initial_count
        age_s = None
        if self.last_odom is not None:
            age_s = self.get_clock().now().nanoseconds / 1e9 - self.last_odom["stamp_s"]
        if observed < required_new_messages:
            errors.append(
                "Only {} new /odom_rf2o messages in readiness window; need {}".format(
                    observed, required_new_messages
                )
            )
        if qualification_required and not qualification_seen:
            errors.append(
                "IMU-yaw raw odometry did not complete continuous startup qualification"
            )
        if age_s is None or age_s < -0.1 or age_s > 0.5:
            errors.append("Final /odom_rf2o age outside [-0.1, 0.5] seconds: {}".format(
                age_s
            ))
        passed = not errors

        self.preflight_results["odom_source"] = {
            "passed": passed,
            "mode": self.odom_mode,
            "new_messages_observed": observed,
            "required_new_messages": required_new_messages,
            "final_message_age_s": age_s,
            "qualification_marker_seen": qualification_seen,
            "errors": errors,
        }

        print()
        print("=" * 70)
        print("ODOMETRY SOURCE CHECK")
        print("=" * 70)
        print("Requested mode: {}".format(self.odom_mode))
        print("RESULT: {}".format("PASS" if passed else "FAIL"))
        for error in errors:
            print("  " + error)
        print("=" * 70)
        print()
        return passed


    def verify_slam_gate_mode(self):

        print()
        print("=" * 70)
        print("SLAM GATE MODE CHECK")
        print("=" * 70)

        log_path = self.process_log_paths.get(
            "slam"
        )

        deadline = time.monotonic() + 5.0
        marker = "Scan distance gate mode:"
        actual_mode = None

        while time.monotonic() < deadline:

            try:

                with open(
                    log_path,
                    "r",
                    errors="replace"
                ) as log_file:

                    for line in log_file:

                        if marker in line:

                            value = line.split(
                                marker,
                                1
                            )[1].strip()

                            if value:
                                actual_mode = (
                                    value.split()[0]
                                )

            except (IOError, OSError, TypeError):
                pass

            if actual_mode is not None:
                break

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        print(
            "Requested mode: {}".format(
                self.scan_distance_gate_mode
            )
        )

        print(
            "Runtime mode:   {}".format(
                actual_mode
                if actual_mode is not None
                else "unavailable"
            )
        )

        passed = (
            actual_mode
            ==
            self.scan_distance_gate_mode
        )

        self.runtime_gate_mode = actual_mode

        print(
            "RESULT: {}".format(
                "PASS" if passed else "FAIL"
            )
        )
        print("=" * 70)
        print()

        if not passed:

            print(
                "Requested SLAM gate mode does not "
                "match the running node."
            )

        return passed


    def verify_karto_config(self):
        print("\n" + "=" * 70)
        print("KARTO INTERNAL CONFIGURATION CHECK")
        print("Profile: {}".format(self.matcher_profile))
        deadline = time.monotonic() + 5.0
        actual, errors = {}, ["Karto configuration log unavailable"]
        while time.monotonic() < deadline:
            try:
                with open(self.process_log_paths["slam"], "r", errors="replace") as stream:
                    actual, errors = check_karto_config(stream.read(), self.matcher_profile)
            except (IOError, OSError, KeyError):
                pass
            if len(actual) == len(expected_parameters(self.matcher_profile)):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.effective_karto_parameters = actual
        passed = not errors
        self.preflight_results["karto_config"] = {
            "passed": passed, "profile": self.matcher_profile, "errors": errors,
            "expected": expected_parameters(self.matcher_profile), "actual": actual,
        }
        for name in ("correlation_search_space_dimension", "distance_variance_penalty",
                     "minimum_distance_penalty"):
            print("{}: {}".format(name, actual.get(name, "unavailable")))
        print("Verified parameters: {}/{}".format(
            len(actual), len(expected_parameters(self.matcher_profile))))
        for error in errors:
            print("  " + error)
        print("RESULT: {}".format("PASS" if passed else "FAIL"))
        print("=" * 70 + "\n")
        if not passed:
            print("Do not drive. Deploy/rebuild Slam Toolbox and verify the installed profile.")
        return passed


    def start_stack(self):

        print()
        print("Starting ROS stack...")
        print()

        # 1. Qualified IMU zero, before the LiDAR motor introduces vibration.
        if self.odom_mode == "imu_yaw":
            self.start_process(
                "imu",
                (
                    "env LD_LIBRARY_PATH=\"$HOME/.local/phidget22/lib"
                    "${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}\" "
                    "python3 {}/phidget_imu_publisher.py"
                ).format(LOCAL_NODES)
            )
            time.sleep(4.0)

        # 2. LiDAR
        self.start_process(
            "lidar",
            (
                "python3 "
                "{}/rplidar_python_bridge.py"
            ).format(
                LOCAL_NODES
            )
        )

        time.sleep(2.0)

        # 3. Static TF
        self.start_process(
            "static_tf",
            (
                "ros2 run tf2_ros "
                "static_transform_publisher "
                "0.00 0.00 {} "
                "{} 0 0 "
                "{} {}"
            ).format(
                LASER_Z,
                LASER_YAW,
                BASE_FRAME,
                LASER_FRAME
            )
        )

        time.sleep(1.0)

        # 4. Optional yaw-aided odometry subscriber.
        if self.odom_mode == "imu_yaw":
            self.start_process(
                "imu_yaw_odom",
                "python3 -u {}/imu_yaw_odometry.py".format(LOCAL_NODES)
            )
            time.sleep(1.0)
            rf2o_command = (
                "ros2 run rf2o_laser_odometry rf2o_laser_odometry_node "
                "--ros-args -p odom_topic:=/odom_rf2o_raw "
                "-p publish_tf:=false"
            )
        else:
            rf2o_command = (
                "ros2 run rf2o_laser_odometry rf2o_laser_odometry_node"
            )

        # 5. RF2O (direct baseline or raw input to IMU-yaw odometry).
        self.start_process("rf2o", rf2o_command)

        time.sleep(2.0)

        # 6. Delay relay
        self.start_process(
            "scan_delay",
            (
                "python3 "
                "{}/scan_delay_node.py"
            ).format(
                LOCAL_NODES
            )
        )

        time.sleep(1.0)

        # 7. SLAM
        self.start_process(
            "slam",
            (
                "ros2 launch "
                "slam_toolbox "
                "online_sync_launch.py "
                "use_sim_time:=false "
                "scan_distance_gate_mode:={} "
                "params_file:={}"
            ).format(
                self.scan_distance_gate_mode,
                shlex.quote(self.slam_params_file)
            )
        )

        print()
        print(
            "Waiting for ROS stack..."
        )


    def stop_stack(self):

        if self.stack_stopped:
            return

        self.stack_stopped = True

        print()
        print(
            "Stopping ROS stack..."
        )

        self.stop_robot()

        # Processes are stopped in reverse startup order.

        for (
            name,
            process
        ) in reversed(
            self.processes
        ):

            if process.poll() is not None:
                continue

            print(
                "Stopping {}...".format(
                    name
                )
            )

            try:

                os.killpg(
                    os.getpgid(
                        process.pid
                    ),
                    signal.SIGINT
                )

            except Exception:
                pass

            try:

                process.wait(
                    timeout=5
                )

            except Exception:

                try:

                    os.killpg(
                        os.getpgid(
                            process.pid
                        ),
                        signal.SIGTERM
                    )

                except Exception:
                    pass

        for log_file in self.log_files:

            try:

                log_file.flush()
                log_file.close()

            except Exception:
                pass

        print(
            "ROS stack stopped."
        )


    # ========================================================
    # ROS / TF HELPERS
    # ========================================================

    def spin_for(self, seconds):

        end_time = (
            time.monotonic()
            +
            seconds
        )

        while (
            rclpy.ok()
            and
            time.monotonic()
            <
            end_time
        ):

            self.require_processes_alive()

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )


    def lookup_pose(
        self,
        target_frame,
        source_frame
    ):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time()
                )
            )

        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException
        ):

            return None

        t = (
            transform.transform.translation
        )

        q = (
            transform.transform.rotation
        )

        yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        return {
            "x": float(t.x),
            "y": float(t.y),
            "yaw_deg": float(
                math.degrees(yaw)
            ),
            # Preserve the TF time: latest RF2O and latest map->base need
            # not describe the same instant. Do not silently time-align them.
            "stamp_s": (transform.header.stamp.sec
                        + transform.header.stamp.nanosec / 1e9),
            "age_s": (self.get_clock().now().nanoseconds / 1e9
                      - transform.header.stamp.sec
                      - transform.header.stamp.nanosec / 1e9),
        }


    def get_slam_pose(self):

        return self.lookup_pose(
            MAP_FRAME,
            BASE_FRAME
        )


    # ========================================================
    # READY CHECK
    # ========================================================

    def wait_for_system(self):

        start_time = (
            time.monotonic()
        )

        last_print = 0.0

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            slam_pose = (
                self.get_slam_pose()
            )

            laser_tf = (
                self.lookup_pose(
                    BASE_FRAME,
                    LASER_FRAME
                )
            )

            odom_tf = (
                self.lookup_pose(
                    ODOM_FRAME,
                    BASE_FRAME
                )
            )

            odom_ready = (
                self.last_odom
                is not None
            )

            map_ready = (
                self.last_map
                is not None
                and
                self.last_map.info.width > 0
                and
                self.last_map.info.height > 0
            )

            slam_ready = (
                slam_pose
                is not None
            )

            laser_tf_ready = (
                laser_tf
                is not None
            )

            odom_tf_ready = (
                odom_tf
                is not None
            )

            failed_processes = self.failed_process_names()
            processes_ready = not failed_processes

            if failed_processes:
                print()
                print(
                    "ERROR: Required ROS process exited during startup: {}".format(
                        ", ".join(failed_processes)
                    )
                )
                for process_name in failed_processes:
                    log_path = self.process_log_paths.get(process_name)
                    if not log_path:
                        continue
                    print("--- last lines of {} ---".format(log_path))
                    try:
                        with open(log_path, "r", errors="replace") as stream:
                            lines = stream.readlines()
                        for line in lines[-20:]:
                            print(line.rstrip())
                    except (IOError, OSError) as exc:
                        print("Could not read log: {}".format(exc))
                print()
                return False

            ready = all([
                odom_ready,
                map_ready,
                slam_ready,
                laser_tf_ready,
                odom_tf_ready,
                processes_ready,
            ])

            if ready:

                print()
                print(
                    "ROS stack ready."
                )
                print()

                return True

            now = (
                time.monotonic()
            )

            if (
                now - last_print
                >
                5.0
            ):

                print(
                    (
                        "Waiting: "
                        "odom={} "
                        "map={} "
                        "laser_tf={} "
                        "odom_tf={} "
                        "slam_tf={} "
                        "processes={}"
                    ).format(
                        odom_ready,
                        map_ready,
                        laser_tf_ready,
                        odom_tf_ready,
                        slam_ready,
                        processes_ready,
                    )
                )

                last_print = now

            if (
                now - start_time
                >
                READY_TIMEOUT_SECONDS
            ):

                print()
                print(
                    "ERROR: Stack did not become "
                    "ready within {:.0f} seconds.".format(
                        READY_TIMEOUT_SECONDS
                    )
                )

                return False

        return False


    # ========================================================
    # SCAN / TF TIMING PREFLIGHT
    # ========================================================

    def run_scan_tf_preflight(self):

        print()
        print("=" * 70)
        print(
            "SCAN / TF SYNCHRONIZATION CHECK"
        )
        print("=" * 70)

        self.scan_tf_test_count = 0
        self.scan_tf_test_success = 0
        self.scan_tf_test_fail = 0
        self.scan_tf_recent_results.clear()

        self.scan_tf_test_active = True

        start = time.monotonic()
        passed = False

        while (
            rclpy.ok()
            and
            self.scan_tf_test_count
            <
            SCAN_TF_MAX_ATTEMPTS
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if len(self.scan_tf_recent_results) >= SCAN_TF_TEST_COUNT:

                recent_success = sum(
                    1 for value in self.scan_tf_recent_results if value
                )

                recent_rate = (
                    float(recent_success)
                    /
                    float(SCAN_TF_TEST_COUNT)
                )

                if recent_rate >= SCAN_TF_MIN_SUCCESS_RATE:
                    passed = True
                    break

            if time.monotonic() - start > SCAN_TF_TIMEOUT_SECONDS:

                break

        self.scan_tf_test_active = False

        total = (
            self.scan_tf_test_count
        )

        success = (
            self.scan_tf_test_success
        )

        failed = (
            self.scan_tf_test_fail
        )

        if total > 0:

            rate = (
                float(success)
                /
                float(total)
            )

        else:

            rate = 0.0

        recent_total = len(
            self.scan_tf_recent_results
        )

        recent_success = sum(
            1 for value in self.scan_tf_recent_results if value
        )

        recent_rate = (
            float(recent_success) / float(recent_total)
            if recent_total > 0
            else 0.0
        )

        print(
            "Delayed scans attempted: {}".format(
                total
            )
        )

        print(
            "TF success:          {}".format(
                success
            )
        )

        print(
            "TF failures:         {}".format(
                failed
            )
        )

        print(
            "Overall success:     {:.1f}%".format(
                rate * 100.0
            )
        )

        print(
            "Latest {} scans:      {}/{} ({:.1f}%)".format(
                recent_total,
                recent_success,
                recent_total,
                recent_rate * 100.0
            )
        )

        self.preflight_results["scan_tf"] = {
            "passed": passed,
            "attempts": total,
            "successes": success,
            "failures": failed,
            "overall_success_rate": rate,
            "window_size": recent_total,
            "window_successes": recent_success,
            "window_success_rate": recent_rate,
        }

        if passed:

            print(
                "RESULT: PASS"
            )

        else:

            print(
                "RESULT: FAIL"
            )

        print("=" * 70)
        print()

        return passed


    # ========================================================
    # STATIONARY PREFLIGHT
    # ========================================================

    def snapshot_pose(self):

        if self.last_odom is None:
            return None

        slam = self.get_slam_pose()

        if slam is None:
            return None

        snapshot = {
            "rf2o": dict(
                self.last_odom
            ),
            "slam": dict(
                slam
            )
        }
        raw = self.last_raw_odom if self.odom_mode == "imu_yaw" else self.last_odom
        if raw is not None:
            snapshot["raw_rf2o"] = dict(raw)
        return snapshot

    def checkpoint_ages(self, snapshot):
        """Check measured pose times, not just recent callback execution."""
        now = self.get_clock().now().nanoseconds / 1e9
        ages = {}
        for key in ("rf2o", "slam", "raw_rf2o"):
            pose = snapshot.get(key) if snapshot else None
            if pose is None or not math.isfinite(pose.get("stamp_s", float("nan"))):
                return None
            age = now - pose["stamp_s"]
            if not -0.05 <= age <= CHECKPOINT_MAX_AGE_SECONDS:
                return None
            ages[key] = age
        return ages


    def run_stationary_preflight(self):

        print()
        print("=" * 70)
        print(
            "STATIONARY PREFLIGHT CHECK"
        )
        print("=" * 70)

        print(
            "Robot must remain completely "
            "stationary for {:.0f} seconds.".format(
                STATIONARY_TEST_SECONDS
            )
        )

        self.spin_for(
            1.0
        )

        start = (
            self.snapshot_pose()
        )

        if start is None:

            self.preflight_results["stationary"] = {
                "passed": False,
                "reason": "pose_unavailable_at_start",
            }

            print(
                "RESULT: FAIL — pose unavailable."
            )

            print("=" * 70)
            print()

            return False

        self.spin_for(
            STATIONARY_TEST_SECONDS
        )

        end = (
            self.snapshot_pose()
        )

        if end is None:

            self.preflight_results["stationary"] = {
                "passed": False,
                "reason": "pose_unavailable_at_end",
            }

            print(
                "RESULT: FAIL — pose unavailable."
            )

            print("=" * 70)
            print()

            return False

        rf2o_distance = (
            distance_2d(
                start["rf2o"],
                end["rf2o"]
            )
        )

        rf2o_heading = abs(
            wrap_degrees(
                end["rf2o"]["yaw_deg"]
                -
                start["rf2o"]["yaw_deg"]
            )
        )

        slam_distance = (
            distance_2d(
                start["slam"],
                end["slam"]
            )
        )

        slam_heading = abs(
            wrap_degrees(
                end["slam"]["yaw_deg"]
                -
                start["slam"]["yaw_deg"]
            )
        )

        print(
            (
                "RF2O translation: "
                "{:.3f} m"
            ).format(
                rf2o_distance
            )
        )

        print(
            (
                "RF2O rotation:    "
                "{:.1f} deg"
            ).format(
                rf2o_heading
            )
        )

        print(
            (
                "SLAM translation: "
                "{:.3f} m"
            ).format(
                slam_distance
            )
        )

        print(
            (
                "SLAM rotation:    "
                "{:.1f} deg"
            ).format(
                slam_heading
            )
        )

        passed = all([
            rf2o_distance
            <=
            MAX_STATIONARY_RF2O_DISTANCE_M,

            rf2o_heading
            <=
            MAX_STATIONARY_RF2O_HEADING_DEG,

            slam_distance
            <=
            MAX_STATIONARY_SLAM_DISTANCE_M,

            slam_heading
            <=
            MAX_STATIONARY_SLAM_HEADING_DEG,
        ])

        self.preflight_results["stationary"] = {
            "passed": passed,
            "duration_s": STATIONARY_TEST_SECONDS,
            "rf2o_distance_m": rf2o_distance,
            "rf2o_heading_deg": rf2o_heading,
            "slam_distance_m": slam_distance,
            "slam_heading_deg": slam_heading,
        }

        if passed:

            print(
                "RESULT: PASS"
            )

        else:

            print(
                "RESULT: FAIL"
            )

            print()
            print(
                "Do not begin the experiment."
            )

        print("=" * 70)
        print()

        return passed


    # ========================================================
    # MOTOR CONTROL
    # ========================================================

    def drive(
        self,
        left,
        right
    ):

        left_command = clamp(
            left
            *
            MOTOR_DIR_L
            *
            BIAS_L
        )

        right_command = clamp(
            right
            *
            MOTOR_DIR_R
            *
            BIAS_R
        )

        self.robot.set_motors(
            left_command,
            right_command
        )


    def forward(self):

        self.drive(
            self.speed,
            self.speed
        )

        self.moving = True

        self.last_motor_command = (
            time.monotonic()
        )


    def backward(self):

        self.drive(
            -self.speed,
            -self.speed
        )

        self.moving = True

        self.last_motor_command = (
            time.monotonic()
        )


    def left(self):

        self.drive(
            self.turn_speed,
            -self.turn_speed
        )

        self.moving = True

        self.last_motor_command = (
            time.monotonic()
        )


    def right(self):

        self.drive(
            -self.turn_speed,
            self.turn_speed
        )

        self.moving = True

        self.last_motor_command = (
            time.monotonic()
        )


    def stop_robot(self):

        try:

            self.drive(
                0.0,
                0.0
            )

        except Exception:
            pass

        self.moving = False


    def update_watchdog(self):

        if not self.moving:
            return

        elapsed = (
            time.monotonic()
            -
            self.last_motor_command
        )

        if (
            elapsed
            >
            WATCHDOG_SECONDS
        ):

            self.stop_robot()


    # ========================================================
    # CHECKPOINTS
    # ========================================================

    def capture_checkpoint(
        self,
        label
    ):

        self.stop_robot()

        self.spin_for(
            CHECKPOINT_SETTLE_SECONDS
        )

        deadline = time.monotonic() + CHECKPOINT_FRESH_WAIT_SECONDS
        snapshot = self.snapshot_pose()
        ages = self.checkpoint_ages(snapshot)
        while ages is None and rclpy.ok() and time.monotonic() < deadline:
            self.spin_for(0.05)
            snapshot = self.snapshot_pose()
            ages = self.checkpoint_ages(snapshot)
        if ages is None:
            self.checkpoint_failures.append({
                "label": label, "time_unix": time.time(),
                "reason": "pose missing, future-dated or older than 0.5 seconds",
            })
            print("Checkpoint NOT recorded: poses unavailable or stale. Robot stopped; retry L when data recovers.")
            self.write_results()
            return None

        checkpoint = {
            "index": len(
                self.checkpoints
            ),
            "label": label,
            "time_unix": time.time(),
            "rf2o": snapshot["rf2o"],
            "slam": snapshot["slam"],
            "raw_rf2o": snapshot["raw_rf2o"],
            "pose_age_s": ages,
            "odom_mode": self.odom_mode,
        }

        self.checkpoints.append(
            checkpoint
        )

        self.print_checkpoint(
            checkpoint
        )

        self.write_results()

        return checkpoint


    def calculate_segment(
        self,
        previous,
        current
    ):

        return {
            "rf2o_distance_m": (
                distance_2d(
                    previous["rf2o"],
                    current["rf2o"]
                )
            ),

            "rf2o_heading_deg": (
                wrap_degrees(
                    current["rf2o"]["yaw_deg"]
                    -
                    previous["rf2o"]["yaw_deg"]
                )
            ),

            "slam_distance_m": (
                distance_2d(
                    previous["slam"],
                    current["slam"]
                )
            ),

            "slam_heading_deg": (
                wrap_degrees(
                    current["slam"]["yaw_deg"]
                    -
                    previous["slam"]["yaw_deg"]
                )
            ),
        }


    def print_checkpoint(
        self,
        checkpoint
    ):

        rf2o = checkpoint["rf2o"]
        slam = checkpoint["slam"]

        print()
        print("=" * 70)

        print(
            "CHECKPOINT {} — {}".format(
                checkpoint["index"],
                checkpoint["label"]
            )
        )

        print("=" * 70)

        print(
            (
                "ODOM  "
                "x={:+.3f} m  "
                "y={:+.3f} m  "
                "yaw={:+.1f} deg"
            ).format(
                rf2o["x"],
                rf2o["y"],
                rf2o["yaw_deg"]
            )
        )

        print(
            (
                "SLAM  "
                "x={:+.3f} m  "
                "y={:+.3f} m  "
                "yaw={:+.1f} deg"
            ).format(
                slam["x"],
                slam["y"],
                slam["yaw_deg"]
            )
        )

        raw = checkpoint["raw_rf2o"]
        print("RAW RF2O x={:+.3f} m y={:+.3f} m yaw={:+.1f} deg".format(
            raw["x"], raw["y"], raw["yaw_deg"]))
        print("Pose ages: odom={:.3f}s raw={:.3f}s SLAM={:.3f}s".format(
            checkpoint["pose_age_s"]["rf2o"], checkpoint["pose_age_s"]["raw_rf2o"],
            checkpoint["pose_age_s"]["slam"]))

        if len(
            self.checkpoints
        ) >= 2:

            previous = (
                self.checkpoints[-2]
            )

            segment = (
                self.calculate_segment(
                    previous,
                    checkpoint
                )
            )

            print()
            print(
                "Since previous checkpoint "
                "({}):".format(
                    previous["label"]
                )
            )

            print(
                (
                    "  RF2O: "
                    "{:.3f} m, "
                    "{:+.1f} deg"
                ).format(
                    segment[
                        "rf2o_distance_m"
                    ],
                    segment[
                        "rf2o_heading_deg"
                    ],
                )
            )

            print(
                (
                    "  SLAM: "
                    "{:.3f} m, "
                    "{:+.1f} deg"
                ).format(
                    segment[
                        "slam_distance_m"
                    ],
                    segment[
                        "slam_heading_deg"
                    ],
                )
            )

        print("=" * 70)
        print()


    # ========================================================
    # MAP SAVING
    # ========================================================

    def save_map(self):

        if self.last_map is None:

            print(
                "No map available to save."
            )

            return

        msg = self.last_map

        width = msg.info.width
        height = msg.info.height
        resolution = (
            msg.info.resolution
        )

        pgm_path = os.path.join(
            self.experiment_dir,
            "map.pgm"
        )

        yaml_path = os.path.join(
            self.experiment_dir,
            "map.yaml"
        )

        png_path = os.path.join(
            self.experiment_dir,
            "map.png"
        )

        with open(
            pgm_path,
            "wb"
        ) as pgm_file:

            header = (
                "P5\n"
                "# CREATOR: mapping_session.py\n"
                "{} {}\n"
                "255\n"
            ).format(
                width,
                height
            )

            pgm_file.write(
                header.encode(
                    "ascii"
                )
            )

            for y in reversed(
                range(height)
            ):

                for x in range(
                    width
                ):

                    value = msg.data[
                        y * width + x
                    ]

                    if value < 0:
                        pixel = 205

                    elif value >= 65:
                        pixel = 0

                    elif value <= 19:
                        pixel = 254

                    else:
                        pixel = 205

                    pgm_file.write(
                        bytes([pixel])
                    )

        origin = (
            msg.info.origin
        )

        q = (
            origin.orientation
        )

        origin_yaw = (
            quaternion_to_yaw(
                q.x,
                q.y,
                q.z,
                q.w
            )
        )

        with open(
            yaml_path,
            "w"
        ) as yaml_file:

            yaml_file.write(
                "image: map.pgm\n"
            )

            yaml_file.write(
                "resolution: {}\n".format(
                    resolution
                )
            )

            yaml_file.write(
                (
                    "origin: "
                    "[{}, {}, {}]\n"
                ).format(
                    origin.position.x,
                    origin.position.y,
                    origin_yaw
                )
            )

            yaml_file.write(
                "negate: 0\n"
            )

            yaml_file.write(
                "occupied_thresh: 0.65\n"
            )

            yaml_file.write(
                "free_thresh: 0.196\n"
            )

        png_saved = False

        try:

            from PIL import Image

            Image.open(
                pgm_path
            ).save(
                png_path
            )

            png_saved = True

        except Exception:
            pass

        print()
        print("Map saved:")

        print(
            "  {}".format(
                pgm_path
            )
        )

        print(
            "  {}".format(
                yaml_path
            )
        )

        if png_saved:

            print(
                "  {}".format(
                    png_path
                )
            )

        print(
            (
                "  {} x {} pixels = "
                "{:.2f} m x {:.2f} m"
            ).format(
                width,
                height,
                width * resolution,
                height * resolution
            )
        )


    # ========================================================
    # RESULTS
    # ========================================================

    def build_results_data(self):

        segments = []

        for index in range(
            1,
            len(self.checkpoints)
        ):

            previous = (
                self.checkpoints[
                    index - 1
                ]
            )

            current = (
                self.checkpoints[
                    index
                ]
            )

            segment = (
                self.calculate_segment(
                    previous,
                    current
                )
            )

            segment["from"] = (
                previous["label"]
            )

            segment["to"] = (
                current["label"]
            )

            segments.append(
                segment
            )

        map_info = None

        if self.last_map is not None:

            resolution = (
                self.last_map.info.resolution
            )

            map_info = {
                "width_pixels": int(
                    self.last_map.info.width
                ),

                "height_pixels": int(
                    self.last_map.info.height
                ),

                "resolution_m_per_pixel": float(
                    resolution
                ),

                "width_m": float(
                    self.last_map.info.width
                    *
                    resolution
                ),

                "height_m": float(
                    self.last_map.info.height
                    *
                    resolution
                ),
            }

        return {
            "experiment_name": (
                self.experiment_name
            ),

            "session_name": (
                self.session_name
            ),

            "session_start_unix": (
                self.session_start_unix
            ),

            "session_end_unix": (
                self.session_end_unix
            ),

            "run_status": self.run_status,

            "runtime_gate_mode": self.runtime_gate_mode,
            "effective_karto_parameters": self.effective_karto_parameters,

            "preflight": self.preflight_results,

            "configuration": {
                "session_telemetry_version": 2,
                "legacy_rf2o_field_source": "IMU-assisted odometry" if self.odom_mode == "imu_yaw" else "raw RF2O",
                "checkpoint_max_age_s": CHECKPOINT_MAX_AGE_SECONDS,
                "matcher_profile": self.matcher_profile,
                "odom_mode": self.odom_mode,
                "imu_yaw_configuration": (
                    {
                        "imu_topic": "/imu/data_raw",
                        "imu_qos_reliability": "best_effort",
                        "imu_qos_depth": 100,
                        "raw_odom_topic": "/odom_rf2o_raw",
                        "output_odom_topic": "/odom_rf2o",
                        "max_imu_gap_s": 0.10,
                        "max_imu_stale_s": 0.50,
                        "max_raw_odom_stale_s": 2.00,
                        "max_fusion_wait_s": 1.00,
                        "raw_odom_stable_samples": 20,
                        "raw_odom_stable_duration_s": 3.0,
                        "raw_odom_startup_timeout_s": 20.0,
                        "rf2o_publish_tf": False,
                    }
                    if self.odom_mode == "imu_yaw"
                    else None
                ),
                "odom_source_sha256": self.odom_source_sha256,
                "slam_params_file": self.slam_params_file,
                "slam_params_sha256": self.slam_params_sha256,
                "drive_speed": (
                    self.speed
                ),

                "turn_speed": (
                    self.turn_speed
                ),

                "watchdog_seconds": (
                    WATCHDOG_SECONDS
                ),

                "laser_z_m": (
                    LASER_Z
                ),

                "laser_yaw_rad": (
                    LASER_YAW
                ),

                "lidar_angle_convention": (
                    LIDAR_ANGLE_CONVENTION
                ),

                "lidar_scan_bins": 360,

                "slam_scan_topic": (
                    "/scan_slam"
                ),

                "slam_scan_delay_seconds": (
                    0.30
                ),

                "scan_distance_gate_mode": (
                    self.scan_distance_gate_mode
                ),
            },

            "map": map_info,

            "checkpoints": (
                self.checkpoints
            ),
            "checkpoint_failures": self.checkpoint_failures,

            "segments": segments,
        }


    def write_results(self):

        path = os.path.join(
            self.experiment_dir,
            "results.json"
        )

        with open(
            path,
            "w"
        ) as output:

            json.dump(
                self.build_results_data(),
                output,
                indent=2,
                sort_keys=True
            )


    def write_summary(self):

        path = os.path.join(
            self.experiment_dir,
            "summary.txt"
        )

        with open(
            path,
            "w"
        ) as output:

            output.write(
                "Experiment: {}\n".format(
                    self.experiment_name
                )
            )

            output.write(
                "Session: {}\n\n".format(
                    self.session_name
                )
            )

            for checkpoint in (
                self.checkpoints
            ):

                output.write(
                    "Checkpoint {}: {}\n".format(
                        checkpoint["index"],
                        checkpoint["label"]
                    )
                )

                output.write(
                    (
                        "  RF2O: "
                        "x={:+.3f} "
                        "y={:+.3f} "
                        "yaw={:+.1f} deg\n"
                    ).format(
                        checkpoint[
                            "rf2o"
                        ]["x"],

                        checkpoint[
                            "rf2o"
                        ]["y"],

                        checkpoint[
                            "rf2o"
                        ]["yaw_deg"],
                    )
                )

                output.write(
                    (
                        "  SLAM: "
                        "x={:+.3f} "
                        "y={:+.3f} "
                        "yaw={:+.1f} deg\n"
                    ).format(
                        checkpoint[
                            "slam"
                        ]["x"],

                        checkpoint[
                            "slam"
                        ]["y"],

                        checkpoint[
                            "slam"
                        ]["yaw_deg"],
                    )
                )

                output.write("\n")

            if len(
                self.checkpoints
            ) >= 2:

                output.write(
                    "\nSEGMENTS\n"
                )

                output.write(
                    "========\n\n"
                )

                for index in range(
                    1,
                    len(self.checkpoints)
                ):

                    previous = (
                        self.checkpoints[
                            index - 1
                        ]
                    )

                    current = (
                        self.checkpoints[
                            index
                        ]
                    )

                    segment = (
                        self.calculate_segment(
                            previous,
                            current
                        )
                    )

                    output.write(
                        "{} -> {}\n".format(
                            previous["label"],
                            current["label"]
                        )
                    )

                    output.write(
                        (
                            "  RF2O: "
                            "{:.3f} m, "
                            "{:+.1f} deg\n"
                        ).format(
                            segment[
                                "rf2o_distance_m"
                            ],

                            segment[
                                "rf2o_heading_deg"
                            ],
                        )
                    )

                    output.write(
                        (
                            "  SLAM: "
                            "{:.3f} m, "
                            "{:+.1f} deg\n\n"
                        ).format(
                            segment[
                                "slam_distance_m"
                            ],

                            segment[
                                "slam_heading_deg"
                            ],
                        )
                    )


    # ========================================================
    # TERMINAL UI
    # ========================================================

    def print_controls(self):

        print()
        print("=" * 70)
        print(
            "MAPPING SESSION READY"
        )
        print("=" * 70)

        print()
        print("Driving:")
        print("  W      forward")
        print("  S      backward")
        print("  A      turn left")
        print("  D      turn right")
        print(
            "  SPACE  immediate stop"
        )

        print()
        print("Measurements:")
        print(
            "  L      automatic checkpoint"
        )
        print(
            "  N      named checkpoint"
        )

        print()
        print("Speed:")
        print(
            "  +      increase drive speed"
        )
        print(
            "  -      decrease drive speed"
        )

        print()
        print("Finish:")
        print(
            "  Q      final checkpoint, "
            "save map, shutdown"
        )

        print()
        print(
            (
                "Drive speed: {:.2f} | "
                "Turn speed: {:.2f}"
            ).format(
                self.speed,
                self.turn_speed
            )
        )

        print("=" * 70)
        print()


    def prompt_checkpoint_name(self):

        self.stop_robot()

        termios.tcsetattr(
            self.stdin_fd,
            termios.TCSADRAIN,
            self.old_terminal_settings
        )

        print()

        try:

            name = input(
                "Checkpoint name: "
            ).strip()

        except Exception:
            name = ""

        if not name:

            name = (
                "checkpoint_{:02d}".format(
                    len(
                        self.checkpoints
                    )
                )
            )

        tty.setcbreak(
            self.stdin_fd
        )

        return name


    def teleop_loop(self):

        self.stdin_fd = (
            sys.stdin.fileno()
        )

        self.old_terminal_settings = (
            termios.tcgetattr(
                self.stdin_fd
            )
        )

        tty.setcbreak(
            self.stdin_fd
        )

        self.print_controls()
        print("ODOM display: {}. RAW RF2O is recorded separately at checkpoints.".format(
            "IMU yaw + RF2O translation" if self.odom_mode == "imu_yaw" else "RF2O"))

        try:

            running = True

            while (
                running
                and
                rclpy.ok()
            ):

                self.require_processes_alive()

                rclpy.spin_once(
                    self,
                    timeout_sec=0.01
                )

                readable, _, _ = (
                    select.select(
                        [sys.stdin],
                        [],
                        [],
                        # spin_once already provides a bounded idle wait.
                        # Do not limit all ROS callbacks to ~33/s by waiting
                        # another 30 ms for a key after every callback.
                        0.0
                    )
                )

                if readable:

                    key = (
                        sys.stdin.read(
                            1
                        ).lower()
                    )

                    if key == "w":
                        self.forward()

                    elif key == "s":
                        self.backward()

                    elif key == "a":
                        self.left()

                    elif key == "d":
                        self.right()

                    elif key == " ":

                        self.stop_robot()

                        print(
                            "\rSTOP                    ",
                            end="",
                            flush=True
                        )

                    elif key == "l":

                        label = (
                            "checkpoint_{:02d}".format(
                                len(
                                    self.checkpoints
                                )
                            )
                        )

                        self.capture_checkpoint(
                            label
                        )

                    elif key == "n":

                        label = (
                            self.prompt_checkpoint_name()
                        )

                        self.capture_checkpoint(
                            label
                        )

                    elif key in [
                        "+",
                        "="
                    ]:

                        self.speed = min(
                            0.60,
                            self.speed + 0.05
                        )

                        print(
                            (
                                "\rDrive speed: {:.2f}     "
                            ).format(
                                self.speed
                            ),
                            end="",
                            flush=True
                        )

                    elif key == "-":

                        self.speed = max(
                            0.15,
                            self.speed - 0.05
                        )

                    elif key == "q":

                        self.stop_robot()

                        print()
                        print(
                            "Finishing experiment..."
                        )

                        running = False

                self.update_watchdog()

        finally:

            self.stop_robot()

            if (
                self.old_terminal_settings
                is not None
            ):

                termios.tcsetattr(
                    self.stdin_fd,
                    termios.TCSADRAIN,
                    self.old_terminal_settings
                )


    # ========================================================
    # FINAL SUMMARY
    # ========================================================

    def print_final_summary(self):

        print()
        print("=" * 70)
        print(
            "EXPERIMENT COMPLETE" if self.run_status == "complete" else "EXPERIMENT INCOMPLETE"
        )
        print("=" * 70)

        print(
            "Checkpoints recorded: {}".format(
                len(
                    self.checkpoints
                )
            )
        )

        if len(
            self.checkpoints
        ) >= 2:

            total = (
                self.calculate_segment(
                    self.checkpoints[0],
                    self.checkpoints[-1]
                )
            )

            print()
            print("Start -> Final")

            print(
                (
                    "  RF2O position delta: "
                    "{:.3f} m"
                ).format(
                    total[
                        "rf2o_distance_m"
                    ]
                )
            )

            print(
                (
                    "  RF2O heading delta:  "
                    "{:+.1f} deg"
                ).format(
                    total[
                        "rf2o_heading_deg"
                    ]
                )
            )

            print(
                (
                    "  SLAM position delta: "
                    "{:.3f} m"
                ).format(
                    total[
                        "slam_distance_m"
                    ]
                )
            )

            print(
                (
                    "  SLAM heading delta:  "
                    "{:+.1f} deg"
                ).format(
                    total[
                        "slam_heading_deg"
                    ]
                )
            )

        print()
        print("Experiment directory:")

        print(
            "  {}".format(
                self.experiment_dir
            )
        )

        print()
        print(
            "Copy to laptop:"
        )

        print(
            (
                "  scp -r "
                "jetbot@<JETBOT_ADDRESS>:{} ."
            ).format(
                self.experiment_dir
            )
        )

        print("=" * 70)
        print()


    # ========================================================
    # COMPLETE SESSION
    # ========================================================

    def run(self):

        print()
        print("=" * 70)

        print(
            "EXPERIMENT: {}".format(
                self.experiment_name
            )
        )

        print("=" * 70)

        print("Output directory:")

        print(
            "  {}".format(
                self.experiment_dir
            )
        )
        print("Odometry mode:")
        print("  {}".format(self.odom_mode))

        if not self.check_for_existing_processes():

            print(
                "Experiment not started."
            )

            self.run_status = "failed_existing_processes"
            self.session_end_unix = time.time()
            self.write_results()
            return 2

        self.run_status = "stack_starting"
        self.write_results()
        self.start_stack()

        if not self.wait_for_system():

            self.run_status = "failed_system_ready"
            self.session_end_unix = time.time()
            self.write_results()
            self.stop_stack()

            return 1

        if not self.verify_odom_source():

            self.run_status = "failed_odom_source"
            self.session_end_unix = time.time()
            self.write_results()
            self.stop_stack()

            return 7

        if not self.verify_slam_gate_mode():

            self.run_status = "failed_gate_mode"
            self.session_end_unix = time.time()
            self.write_results()
            self.stop_stack()

            return 5

        if not self.verify_karto_config():
            self.run_status = "failed_karto_config"
            self.session_end_unix = time.time()
            self.write_results()
            self.stop_stack()
            return 6

        print(
            (
                "Initial settling for "
                "{:.0f} seconds..."
            ).format(
                INITIAL_SETTLE_SECONDS
            )
        )

        self.spin_for(
            INITIAL_SETTLE_SECONDS
        )

        # --------------------------------------------
        # Critical synchronization check
        # --------------------------------------------

        if not self.run_scan_tf_preflight():

            print(
                "Scan/TF synchronization failed."
            )

            self.run_status = "failed_scan_tf_preflight"
            self.session_end_unix = time.time()
            self.write_results()
            self.stop_stack()

            return 3

        # --------------------------------------------
        # Stationary drift check
        # --------------------------------------------

        if not self.run_stationary_preflight():

            self.run_status = "failed_stationary_preflight"
            self.session_end_unix = time.time()
            self.write_results()
            self.stop_stack()

            return 4

        # --------------------------------------------
        # Official experiment start
        # --------------------------------------------

        self.run_status = "running"
        if self.capture_checkpoint("start") is None:
            self.run_status = "failed_start_checkpoint"
            self.session_end_unix = time.time()
            self.write_results()
            self.stop_stack()
            return 8

        self.teleop_loop()

        print(
            (
                "Allowing SLAM to settle for "
                "{:.0f} seconds..."
            ).format(
                FINAL_SETTLE_SECONDS
            )
        )

        self.spin_for(
            FINAL_SETTLE_SECONDS
        )

        final_checkpoint = self.capture_checkpoint("final")

        self.session_end_unix = (
            time.time()
        )

        self.run_status = "complete" if final_checkpoint is not None else "failed_final_checkpoint"

        self.save_map()

        self.write_results()
        self.write_summary()

        self.print_final_summary()

        self.stop_stack()

        return 0 if final_checkpoint is not None else 9


# ============================================================
# ENTRY POINT
# ============================================================

def main():

    if len(sys.argv) not in [2, 3, 4, 5]:

        print()
        print(
            "Usage:"
        )

        print(
            "  python3 mapping_session.py "
            "EXPERIMENT_NAME "
            "[legacy|euclidean|euclidean_heading] [baseline|narrow] "
            "[rf2o|imu_yaw]"
        )

        print()
        print(
            "Example:"
        )

        print(
            "  python3 mapping_session.py "
            "elbow_baseline_01 legacy"
        )

        sys.exit(1)

    experiment_name = (
        sys.argv[1]
    )

    scan_distance_gate_mode = "legacy"

    if len(sys.argv) >= 3:
        scan_distance_gate_mode = sys.argv[2]

    if scan_distance_gate_mode not in [
        "legacy",
        "euclidean",
        "euclidean_heading"
    ]:

        print(
            "Gate mode must be "
            "'legacy', 'euclidean', or "
            "'euclidean_heading'."
        )
        sys.exit(1)

    matcher_profile = sys.argv[3] if len(sys.argv) >= 4 else "baseline"
    if matcher_profile not in PROFILE_FILES:
        print("Matcher profile must be 'baseline' or 'narrow'.")
        sys.exit(1)

    odom_mode = sys.argv[4] if len(sys.argv) == 5 else "rf2o"
    if odom_mode not in ["rf2o", "imu_yaw"]:
        print("Odometry mode must be 'rf2o' or 'imu_yaw'.")
        sys.exit(1)

    rclpy.init()

    node = MappingSession(
        experiment_name,
        scan_distance_gate_mode,
        matcher_profile,
        odom_mode
    )

    exit_code = 1

    try:

        exit_code = (
            node.run()
        )

    except KeyboardInterrupt:

        print()
        print(
            "Interrupted."
        )

        node.stop_robot()
        node.stop_stack()

    except Exception as exc:

        print()
        print(
            "FATAL ERROR: {}".format(
                exc
            )
        )

        node.stop_robot()
        node.stop_stack()

        raise

    finally:

        try:

            node.destroy_node()

        except Exception:
            pass

        if rclpy.ok():

            rclpy.shutdown()

    sys.exit(
        exit_code
    )


if __name__ == "__main__":
    main()
