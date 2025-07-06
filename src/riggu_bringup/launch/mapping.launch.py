from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path to the riggu_bringup package for config files
    riggu_bringup_dir = get_package_share_directory('riggu_bringup')

    # SLAM Toolbox launch for asynchronous online mapping
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                riggu_bringup_dir,
                'launch',
                'slam_toolbox',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(riggu_bringup_dir, 'config', 'slam_toolbox', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        slam_toolbox_launch,
    ])
