from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
import os
from ament_index_python.packages import get_package_share_directory


def set_twist_stamper_out_topic(context, *args, **kwargs):
    using_nav2 = LaunchConfiguration('using_nav2')
    if context.perform_substitution(using_nav2).lower() == 'false':
        return [SetLaunchConfiguration('twist_stamper_out_topic', '/cmd_vel_direct')]
    else:
        return [SetLaunchConfiguration('twist_stamper_out_topic', '/cmd_vel_joy')]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    using_nav2 = LaunchConfiguration('using_nav2')

    joy_params = os.path.join(
        get_package_share_directory('riggu_bringup'),
        'config',
        'joystick.yaml'
    )
    
    # Verify config file exists
    if not os.path.exists(joy_params):
        raise FileNotFoundError(f"Joy configuration file not found: {joy_params}")

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_joy_unstamped')]
    )

    twist_stamper_remappings = [
        ('/cmd_vel_in', '/cmd_vel_joy_unstamped'),
        ('/cmd_vel_out', LaunchConfiguration('twist_stamper_out_topic'))
    ]

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=twist_stamper_remappings
    )

    twist_mux_params_file = os.path.join(
        get_package_share_directory('riggu_bringup'),
        'config',
        'twist_mux_config.yaml'
    )
    
    # Verify config file exists
    if not os.path.exists(twist_mux_params_file):
        raise FileNotFoundError(f"Twist mux configuration file not found: {twist_mux_params_file}")

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params_file],
        remappings=[('cmd_vel_out', 'diff_drive/cmd_vel')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'using_nav2',
            default_value='true',
            description='If true, output of twist_stamper goes to /cmd_vel_joy, else to diff_drive/cmd_vel'
        ),
        OpaqueFunction(function=set_twist_stamper_out_topic),
        joy_node,
        teleop_node,
        twist_stamper,
        twist_mux_node
    ])