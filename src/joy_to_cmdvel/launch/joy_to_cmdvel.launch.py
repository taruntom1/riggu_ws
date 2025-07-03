from launch import LaunchDescription
from launch_ros.actions import Node

# This launch file starts the joy_to_cmdvel node, which converts joystick inputs to cmd_vel messages.
# Parameters can be adjusted below to match your joystick configuration.
# Optionally, you can also launch the joystick driver (joy_node) by uncommenting the relevant section.

def generate_launch_description():
    return LaunchDescription([
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
            parameters=[{
                'linear_axis': 1,       # Index of joystick axis for linear velocity (default: 1)
                'angular_axis': 3,      # Index of joystick axis for angular velocity (default: 2)
                'linear_scale': 0.05,    # Scaling factor for linear velocity
                'angular_scale': 0.03    # Scaling factor for angular velocity
            }]
        )
    ])
