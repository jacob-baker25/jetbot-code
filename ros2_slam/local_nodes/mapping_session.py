#!/usr/bin/env python3

import json
import math
import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty

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

STATIONARY_TEST_SECONDS = 5.0

MAX_STATIONARY_RF2O_DISTANCE_M = 0.15
MAX_STATIONARY_RF2O_HEADING_DEG = 7.5

MAX_STATIONARY_SLAM_DISTANCE_M = 0.15
MAX_STATIONARY_SLAM_HEADING_DEG = 7.5

CHECKPOINT_SETTLE_SECONDS = 0.5
FINAL_SETTLE_SECONDS = 5.0


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

    def __init__(self, experiment_name):

        super().__init__(
            "mapping_session"
        )

        self.experiment_name = (
            experiment_name
        )

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

        self.session_start_unix = (
            time.time()
        )

        self.session_end_unix = None

        self.robot = Robot()

        self.speed = DRIVE_SPEED
        self.turn_speed = TURN_SPEED

        self.moving = False
        self.last_motor_command = 0.0

        self.last_odom = None
        self.last_map = None

        self.checkpoints = []

        self.processes = []
        self.log_files = []

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

        # --------------------------------------------
        # Subscribers
        # --------------------------------------------

        self.create_subscription(
            Odometry,
            "/odom_rf2o",
            self.odom_callback,
            10
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


    # ========================================================
    # ROS CALLBACKS
    # ========================================================

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.last_odom = {
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


    def delayed_scan_callback(self, msg):

        if not self.scan_tf_test_active:
            return

        if (
            self.scan_tf_test_count
            >=
            SCAN_TF_TEST_COUNT
        ):
            return

        self.scan_tf_test_count += 1

        stamp = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec
        )

        try:

            self.tf_buffer.lookup_transform(
                ODOM_FRAME,
                msg.header.frame_id,
                stamp
            )

            self.scan_tf_test_success += 1

        except Exception:

            self.scan_tf_test_fail += 1


    # ========================================================
    # EXISTING PROCESS CHECK
    # ========================================================

    def check_for_existing_processes(self):

        patterns = [
            "rplidar_python_bridge.py",
            "scan_delay_node.py",
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

        shell_command = (
            "source /opt/ros/eloquent/setup.bash && "
            "source {}/install/setup.bash "
            ">/dev/null 2>&1 || true; "
            "exec {}"
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


    def start_stack(self):

        print()
        print("Starting ROS stack...")
        print()

        # 1. LiDAR
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

        # 2. Static TF
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

        # 3. RF2O
        self.start_process(
            "rf2o",
            (
                "ros2 run "
                "rf2o_laser_odometry "
                "rf2o_laser_odometry_node"
            )
        )

        time.sleep(2.0)

        # 4. Delay relay
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

        # 5. SLAM
        self.start_process(
            "slam",
            (
                "ros2 launch "
                "slam_toolbox "
                "online_sync_launch.py "
                "use_sim_time:=false"
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

        # Processes are stopped in reverse startup order:
        # SLAM -> delay -> RF2O -> TF -> LiDAR

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
            )
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

            processes_ready = all([
                self.process_is_alive(
                    "lidar"
                ),
                self.process_is_alive(
                    "static_tf"
                ),
                self.process_is_alive(
                    "rf2o"
                ),
                self.process_is_alive(
                    "scan_delay"
                ),
                self.process_is_alive(
                    "slam"
                ),
            ])

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

        self.scan_tf_test_active = True

        start = time.monotonic()

        while (
            rclpy.ok()
            and
            self.scan_tf_test_count
            <
            SCAN_TF_TEST_COUNT
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if (
                time.monotonic()
                -
                start
                >
                10.0
            ):

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

        print(
            "Delayed scans tested: {}".format(
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
            "Success rate:        {:.1f}%".format(
                rate * 100.0
            )
        )

        passed = (
            total
            >=
            SCAN_TF_TEST_COUNT
            and
            rate
            >=
            SCAN_TF_MIN_SUCCESS_RATE
        )

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

        return {
            "rf2o": dict(
                self.last_odom
            ),
            "slam": dict(
                slam
            )
        }


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

        snapshot = (
            self.snapshot_pose()
        )

        if snapshot is None:

            print()
            print(
                "Checkpoint failed: "
                "pose unavailable."
            )
            print()

            return None

        checkpoint = {
            "index": len(
                self.checkpoints
            ),
            "label": label,
            "time_unix": time.time(),
            "rf2o": snapshot["rf2o"],
            "slam": snapshot["slam"],
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
                "RF2O  "
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

            "configuration": {
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

                "slam_scan_topic": (
                    "/scan_slam"
                ),

                "slam_scan_delay_seconds": (
                    0.30
                ),
            },

            "map": map_info,

            "checkpoints": (
                self.checkpoints
            ),

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

        try:

            running = True

            while (
                running
                and
                rclpy.ok()
            ):

                rclpy.spin_once(
                    self,
                    timeout_sec=0.01
                )

                readable, _, _ = (
                    select.select(
                        [sys.stdin],
                        [],
                        [],
                        0.03
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
            "EXPERIMENT COMPLETE"
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

        if not self.check_for_existing_processes():

            print(
                "Experiment not started."
            )

            return 2

        self.start_stack()

        if not self.wait_for_system():

            self.stop_stack()

            return 1

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

            self.stop_stack()

            return 3

        # --------------------------------------------
        # Stationary drift check
        # --------------------------------------------

        if not self.run_stationary_preflight():

            self.stop_stack()

            return 4

        # --------------------------------------------
        # Official experiment start
        # --------------------------------------------

        self.capture_checkpoint(
            "start"
        )

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

        self.capture_checkpoint(
            "final"
        )

        self.session_end_unix = (
            time.time()
        )

        self.save_map()

        self.write_results()
        self.write_summary()

        self.print_final_summary()

        self.stop_stack()

        return 0


# ============================================================
# ENTRY POINT
# ============================================================

def main():

    if len(sys.argv) != 2:

        print()
        print(
            "Usage:"
        )

        print(
            "  python3 mapping_session.py "
            "EXPERIMENT_NAME"
        )

        print()
        print(
            "Example:"
        )

        print(
            "  python3 mapping_session.py "
            "elbow_test_01"
        )

        sys.exit(1)

    experiment_name = (
        sys.argv[1]
    )

    rclpy.init()

    node = MappingSession(
        experiment_name
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
