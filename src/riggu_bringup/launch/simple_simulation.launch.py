from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ekf_config_path = os.path.join(
        FindPackageShare('riggu_bringup').find('riggu_bringup'),
        'config',
        'ekf.yaml'
    )

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('riggu_description'),
            'urdf',
            'robot.urdf.xacro'
        ]),
        ' use_ros2_control:=false',
        ' sim_mode:=false'
    ])

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],  # Changed from 'ign gazebo' to 'gz sim'
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'ROS1', '-topic', 'robot_description'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node', # Name must match the header in your YAML file
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}], # Load from file
            remappings=[
                ('/odometry/filtered', '/odometry/filtered'),
                ('/odom', '/odom'),
                ('/imu/data', '/imu/data')
            ]
        ),
    ])