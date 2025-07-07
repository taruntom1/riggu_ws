import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Set package names for robot description and bringup
    bringup_package_name = 'riggu_bringup'

    # Set the default world file for simulation
    default_world = os.path.join(
        get_package_share_directory(bringup_package_name),
        'worlds',
        'something.sdf'
    )
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Launch Gazebo simulator with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn the robot entity in Gazebo using the robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Bridge parameters for ROS <-> Gazebo communication
    bridge_params = os.path.join(
        get_package_share_directory(bringup_package_name),
        'config',
        'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Optional: Bridge for camera images between Gazebo and ROS
    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"]
    # )

    # Return the complete launch description
    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        # ros_gz_image_bridge
    ])
