from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_distance_gate_mode = LaunchConfiguration('scan_distance_gate_mode')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_scan_distance_gate_mode_argument = DeclareLaunchArgument(
        'scan_distance_gate_mode',
        default_value='legacy',
        description='Scan distance gate: legacy or euclidean')

    start_sync_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("slam_toolbox") + '/config/mapper_params_online_sync.yaml',
          {
            'use_sim_time': use_sim_time,
            'scan_distance_gate_mode': scan_distance_gate_mode
          }
        ],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_scan_distance_gate_mode_argument)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
