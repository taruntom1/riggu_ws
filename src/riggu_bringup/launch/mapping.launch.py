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

    # RPLidar C1 node with custom configuration
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[os.path.join(riggu_bringup_dir, 'config', 'lidar', 'rplidar_c1_config.yaml')],
        output='screen'
    )

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
        drivelink_launch,
        joy_to_cmdvel_launch,
        rplidar_node,
        slam_toolbox_launch,
    ])
