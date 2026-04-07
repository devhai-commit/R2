"""
r2_gazebo.launch.py — Launch R2 robot in Gazebo with meihua_forest world.

Starts:
  1. Gazebo Harmonic with worlds/meihua_forest.sdf
  2. robot_state_publisher (R2 URDF)
  3. ros_gz_sim create — spawns R2 into Gazebo
  4. ros_gz_bridge (cmd_vel, odom, joint_states, imu, lift, gripper, tf, clock)

Drive R2 manually:
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('r2_sim')

    # ── Paths ─────────────────────────────────────────────────────────────
    world_file = os.path.join(pkg, 'worlds', 'meihua_forest.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'r2_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    bridge_config = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')

    # ── GZ_SIM_RESOURCE_PATH ─────────────────────────────────────────────
    # Gazebo needs to find:
    #   - model://r2_sim/meshes/...  → parent of pkg (share/)
    #   - models referenced in world → worlds/models/
    gz_share = os.path.dirname(pkg)                          # .../share/
    gz_world_models = os.path.join(pkg, 'worlds', 'models')  # world models
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    parts = [gz_share, gz_world_models]
    if existing:
        parts.append(existing)
    combined_resource = ':'.join(parts)

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock',
        ),

        # Set resource path for Gazebo
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', combined_resource),

        # 1. Gazebo server (headless — avoids snap/glibc GUI crash)
        #    To open GUI separately: gz sim -g
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

        # 2. Robot state publisher (R2)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # 3. Spawn R2 into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'r2_game_field',
                '-name', 'r2_robot',
                '-topic', '/robot_description',
                '-x', '0.0',
                '-y', '3.5',
                '-z', '0.05',
            ],
            output='screen',
        ),

        # 4. ROS ↔ Gazebo bridge (uses config yaml + tf topic)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args', '-p', f'config_file:={bridge_config}',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
