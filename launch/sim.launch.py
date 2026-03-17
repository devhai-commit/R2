import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('r2_sim')

    world_file = os.path.join(pkg, 'worlds', 'meihua_forest.sdf')

    xacro_file = os.path.join(pkg, 'urdf', 'r2_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot is defined inline in meihua_forest.sdf at the left R2 start/retry zone
    # (-1.34, 5.54, 0.075), yaw=π — no separate spawn step needed.

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-r'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
