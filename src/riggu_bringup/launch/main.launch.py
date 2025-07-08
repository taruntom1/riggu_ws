from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) if true'
    )
    
    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Whether to launch Nav2 (true/false)'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI) when using simulation'
    )

    # Get package share directory
    riggu_bringup_dir = get_package_share_directory('riggu_bringup')

    # Include hardware launch file (only when NOT using simulation)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(riggu_bringup_dir, 'launch', 'helpers', 'hardware.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    # Include joy launch file
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(riggu_bringup_dir, 'launch', 'helpers', 'joy.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim'),
            'using_nav2': LaunchConfiguration('launch_nav2')
        }.items()
    )

    # Include nav2_slam launch file
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(riggu_bringup_dir, 'launch', 'helpers', 'nav2_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim'),
            'launch_nav2': LaunchConfiguration('launch_nav2')
        }.items()
    )

    # Include rsp launch file from riggu_description package
    riggu_description_dir = get_package_share_directory('riggu_description')
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(riggu_description_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim')
        }.items()
    )

    # Include Gazebo simulation launch file (only when use_sim is true)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(riggu_bringup_dir, 'launch', 'helpers', 'launch_sim.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([
        use_sim_arg,
        launch_nav2_arg,
        headless_arg,
        hardware_launch,
        joy_launch,
        nav2_slam_launch,
        rsp_launch,
        sim_launch
    ])
