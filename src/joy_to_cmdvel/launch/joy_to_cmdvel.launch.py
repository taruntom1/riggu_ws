from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joy node (joystick driver)
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     output='screen'
        # ),

        # Launch your joystick-to-cmd_vel converter node
        Node(
            package='joy_to_cmdvel',
            executable='joy_to_cmdvel_node',
            name='joy_to_cmdvel_node',
            output='screen',
            parameters=[{
                'linear_axis': 1,       # Left stick vertical
                'angular_axis': 2,      # Left stick horizontal
                'linear_scale': 1.0,
                'angular_scale': 1.0
            }]
        )
    ])
