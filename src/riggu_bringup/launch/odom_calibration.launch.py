from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path to the riggu_bringup package for config files
    riggu_bringup_dir = get_package_share_directory('riggu_bringup')
    
    drivelink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drivelink_interface'),
                'launch',
                'drivelink_interface_node.launch.py'
            )
        ),
        launch_arguments={
            'node_config_path': os.path.join(riggu_bringup_dir, 'config', 'drivelink', 'node_config.yaml'),
            'controller_config_path': os.path.join(riggu_bringup_dir, 'config', 'drivelink', 'controller_config.json'),
        }.items()
    )

    joy_to_cmdvel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('joy_to_cmdvel'),
                'launch',
                'joy_to_cmdvel.launch.py'
            )
        ),
        launch_arguments={
            'config_file': os.path.join(riggu_bringup_dir, 'config', 'joy_to_cmdvel', 'joy_to_cmdvel_config.yaml'),
        }.items()
    )


    return LaunchDescription([
        drivelink_launch,
        joy_to_cmdvel_launch,
    ])
