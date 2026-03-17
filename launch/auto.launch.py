"""
auto.launch.py — launch simulation + autonomous KFS collection.

Starts:
  1. Gazebo with the meihua_forest world
  2. robot_state_publisher
  3. ros_gz_bridge  (bridges /cmd_vel, /odom, /tf)
  4. navigator      (waypoint follower)
  5. kfs_collector   (task planner / state machine)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('r2_sim')

    # ── World ────────────────────────────────────────────────────────────────
    world_file = os.path.join(pkg, 'worlds', 'meihua_forest.sdf')

    # ── Robot description ────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg, 'urdf', 'r2_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── Config files ────────────────────────────────────────────────────────
    kfs_config = os.path.join(pkg, 'config', 'kfs_layout.yaml')
    bridge_config = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')

    return LaunchDescription([
        # 1. Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-r'],
            output='screen',
        ),

        # 2. Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # 3. Gazebo ↔ ROS2 bridge (YAML config for reliable type mapping)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p',
                       f'config_file:={bridge_config}'],
            output='screen',
        ),

        # 4. Waypoint navigator
        Node(
            package='r2_sim',
            executable='navigator',
            output='screen',
        ),

        # 5. KFS collector (autonomous task planner)
        Node(
            package='r2_sim',
            executable='kfs_collector',
            parameters=[{
                'config_path': kfs_config,
                'team': 'r2',
                'collect_duration': 2.0,
                'place_duration': 2.0,
                'start_x': -1.34,
                'start_y':  5.54,
                'start_yaw': 3.14159265,
            }],
            output='screen',
        ),
    ])
