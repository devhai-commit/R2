"""
auto.launch.py — launch full match simulation with both R1 and R2.

Starts:
  1. Gazebo with the meihua_forest world (contains R1 model)
  2. robot_state_publisher (R2)
  3. ros_gz_sim create — spawns R2 from URDF into Gazebo
  4. ros_gz_bridge  (bridges R1 + R2 cmd_vel, odom, tf, scan)
  5. R2 navigator   (waypoint follower on default topics)
  6. R2 kfs_collector (full match state machine)

R1 is keyboard-controlled — run in a SEPARATE terminal:
  ros2 run r2_sim r1_sim
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
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

    # Gazebo resolves model://r2_sim/meshes/... by searching GZ_SIM_RESOURCE_PATH
    # for a directory named 'r2_sim'. pkg points to .../share/r2_sim, so the
    # parent (.../share/) is what Gazebo needs.
    gz_resource = os.path.dirname(pkg)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    combined_resource = f'{gz_resource}:{existing}' if existing else gz_resource

    return LaunchDescription([
        # Set resource path so Gazebo can resolve package://r2_sim/meshes/
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', combined_resource),

        # 1. Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-r'],
            output='screen',
        ),

        # 2. Robot state publisher (R2)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # 3. Spawn R2 robot from URDF into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'r2_game_field',
                '-name', 'r2_robot',
                '-topic', '/robot_description',
                '-x', '-1.34',
                '-y', '5.54',
                '-z', '0.0',
                '-Y', '3.14159',
            ],
            output='screen',
        ),

        # 4. Gazebo ↔ ROS2 bridge (R1 + R2 topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p',
                       f'config_file:={bridge_config}'],
            output='screen',
        ),

        # ── R2 (autonomous) ─────────────────────────────────────────────

        # 5. R2 waypoint navigator (default topics)
        Node(
            package='r2_sim',
            executable='navigator',
            name='r2_navigator',
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

        # NOTE: R1 is keyboard-controlled.
        # Run in a separate terminal:  ros2 run r2_sim r1_sim
    ])
