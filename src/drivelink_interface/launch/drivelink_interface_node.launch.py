from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments for configuration file paths
    node_config_path_arg = DeclareLaunchArgument(
        'node_config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('drivelink_interface'),
            'config',
            'node_config.yaml'
        ]),
        description='Path to the node configuration YAML file'
    )
    
    controller_config_path_arg = DeclareLaunchArgument(
        'controller_config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('drivelink_interface'),
            'config',
            'controller_config.json'
        ]),
        description='Path to the controller configuration JSON file'
    )

    # Create the node with parameters
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
