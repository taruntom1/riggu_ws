from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
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

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[os.path.join(riggu_bringup_dir, 'config', 'lidar', 'rplidar_c1_config.yaml')],
        output='screen'
    )

    return LaunchDescription([
        drivelink_launch,
        rplidar_node
    ])
