from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Example launch file showing how to use custom configuration files.
    
    Usage:
      ros2 launch drivelink_interface drivelink_interface_custom_config.launch.py \
        node_config_path:=/path/to/custom_node_config.yaml \
        controller_config_path:=/path/to/custom_controller_config.json
    """
    
    # Declare launch arguments for custom configuration file paths
    node_config_path_arg = DeclareLaunchArgument(
        'node_config_path',
        default_value='/path/to/custom_node_config.yaml',
        description='Path to the custom node configuration YAML file'
    )
    
    controller_config_path_arg = DeclareLaunchArgument(
        'controller_config_path',
        default_value='/path/to/custom_controller_config.json',
        description='Path to the custom controller configuration JSON file'
    )

    # Create the node with custom parameters
    drivelink_interface_node = Node(
        package='drivelink_interface',
        executable='drivelink_interface_node',
        name='drivelink_interface_node',
        output='screen',
        parameters=[{
            'node_config_path': LaunchConfiguration('node_config_path'),
            'controller_config_path': LaunchConfiguration('controller_config_path'),
        }]
    )

    return LaunchDescription([
        node_config_path_arg,
        controller_config_path_arg,
        drivelink_interface_node,
    ])
