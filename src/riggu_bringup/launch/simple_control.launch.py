from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    drivelink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drivelink_interface'),
                'launch',
                'drivelink_interface_node.launch.py'  # Ensure the .py extension
            )
        )
    )

    joy_to_cmdvel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('joy_to_cmdvel'),
                'launch',
                'joy_to_cmdvel.launch.py'
            )
        )
    )

    return LaunchDescription([
        drivelink_launch,
        joy_to_cmdvel_launch,
    ])
