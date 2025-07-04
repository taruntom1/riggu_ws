from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# This launch file starts the joy_to_cmdvel node, which converts joystick inputs to cmd_vel messages.
# Parameters can be adjusted in the config file.
# Optionally, you can also launch the joystick driver (joy_node) by uncommenting the relevant section.

def generate_launch_description():
    # Default config file path
    default_config_path = os.path.join(
        get_package_share_directory('joy_to_cmdvel'),
        'config',
        'joy_to_cmdvel_config.yaml'
    )
    
    # Declare launch argument for config file path
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the joy_to_cmdvel configuration file'
    )
    return LaunchDescription([
        config_file_arg,
        
        # Launch the joy node (joystick driver)
        # Uncomment the following block to launch the joystick driver together with joy_to_cmdvel
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Launch the joystick-to-cmd_vel converter node
        Node(
            package='joy_to_cmdvel',
            executable='joy_to_cmdvel_node',
            name='joy_to_cmdvel_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])
