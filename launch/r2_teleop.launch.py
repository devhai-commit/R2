"""
r2_teleop.launch.py — R2 in Gazebo, ready for keyboard teleop.

Starts:
  1. Gazebo with meihua_forest world
  2. robot_state_publisher (R2 URDF)
  3. ros_gz_sim create — spawns R2
  4. ros_gz_bridge

Then in a SEPARATE terminal run:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

    world_file = os.path.join(pkg, 'worlds', 'meihua_forest.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'r2_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    bridge_config = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')

    gz_share = os.path.dirname(pkg)
    gz_world_models = os.path.join(pkg, 'worlds', 'models')
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

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', combined_resource),

        # 1. Gazebo with GUI
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

        # 2. Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # 3. Spawn R2
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'r2_game_field',
                '-name', 'r2_robot',
                '-topic', '/robot_description',
                '-x', '0.0',
                '-y', '3.5',
                '-z', '0.0',
            ],
            output='screen',
        ),

        # 4. ROS ↔ Gazebo bridge
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
