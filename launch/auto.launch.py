"""
auto.launch.py — launch full match simulation.

Starts:
  1. Gazebo with the meihua_forest world
  2. robot_state_publisher
  3. ros_gz_bridge  (bridges /cmd_vel, /odom, /tf)
  4. r1_sim         (scripted R1 behaviour — staff pick + assembly)
  5. navigator      (R2 waypoint follower)
  6. kfs_collector  (R2 full match state machine)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('r2_sim')

    # ── Paths ────────────────────────────────────────────────────────────────
    world_file = os.path.join(pkg, 'worlds', 'meihua_forest.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'r2_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
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

        # 3. Gazebo ↔ ROS2 bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p',
                       f'config_file:={bridge_config}'],
            output='screen',
        ),

        # 4. R1 simulator (scripted, no physics)
        Node(
            package='r2_sim',
            executable='r1_sim',
            output='screen',
        ),

        # 5. R2 waypoint navigator
        Node(
            package='r2_sim',
            executable='navigator',
            output='screen',
        ),

        # 6. R2 full match state machine
        Node(
            package='r2_sim',
            executable='kfs_collector',
            parameters=[{
                'config_path': kfs_config,
                'team': 'r2',
                'collect_duration': 2.0,
                'place_duration': 2.0,
                'pick_spearhead_duration': 2.0,
                'assemble_duration': 3.0,
                'start_x': -1.34,
                'start_y':  5.54,
                'start_yaw': 3.14159265,
            }],
            output='screen',
        ),
    ])
