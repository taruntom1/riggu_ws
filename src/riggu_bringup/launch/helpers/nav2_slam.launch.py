from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Whether to launch Nav2 (true/false)'
    )
    
    # Static params file path
    params_file = os.path.join(
        get_package_share_directory('riggu_bringup'),
        'config',
        'nav2_params.yaml'
    )

    # Paths to included launch files and configs
    slam_launch_file = os.path.join(
        get_package_share_directory('riggu_bringup'),
        'launch',
        'slam_toolbox',
        'online_sync_launch.py'
    )
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    # LaunchDescription actions
    actions = [
        use_sim_time_arg,
        launch_nav2_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params_file
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_nav2'))
        )
    ]

    return LaunchDescription(actions)
