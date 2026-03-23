"""
r2_rviz.launch.py — Visualize R2 robot model in RViz (no Gazebo).

Starts:
  1. robot_state_publisher — publish URDF to /robot_description
  2. joint_state_publisher_gui — interactive sliders for all joints
  3. rviz2 — with pre-configured RobotModel + TF display
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('r2_sim')

    xacro_file = os.path.join(pkg, 'urdf', 'r2_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_config = os.path.join(pkg, 'config', 'r2_robot.rviz')

    return LaunchDescription([
        # Publish URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Joint sliders to move lift, gripper, wheels, etc.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
