from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drivelink_interface',
            executable='drivelink_interface_node',  # replace with actual executable name
            name='drivelink_interface_node',
            output='screen',

        )
    ])
