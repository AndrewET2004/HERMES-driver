import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('hermes_driver_base')
    params_file = os.path.join(pkg_dir, 'config', 'driver_params.yaml')

    return LaunchDescription([
        Node(
            package='hermes_driver_base',
            executable='diff_drive_controller_node',
            name='diff_drive_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])