# Copyright 2026 Tarun Tom
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Gazebo Harmonic + riggu: gz sim, spawn, ros_gz_bridge, bringup mode:=sim, RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description."""
    share = FindPackageShare('riggu_gazebo')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    server_flags = PythonExpression(
        ["'-s --headless-rendering' if '", headless, "' == 'true' else ''"])

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r ', server_flags, ' ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'riggu',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.01', '-Y', LaunchConfiguration('yaw')],
        parameters=[{'use_sim_time': use_sim_time}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': PathJoinSubstitution([share, 'config', 'bridge.yaml']),
                     'use_sim_time': use_sim_time}])

    bringup = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('riggu_bringup'), 'launch', 'bringup.launch.py']),
        launch_arguments={'mode': 'sim', 'use_sim_time': use_sim_time}.items())

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'sim.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('rviz'), "' == 'true' and '", headless, "' != 'true'"])))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [share, 'worlds', 'house.sdf']), description='SDF world file'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Server only, headless rendering (CI); no RViz'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='2.5', description='Spawn x (m)'),
        DeclareLaunchArgument('y', default_value='-1.5', description='Spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)'),
        gazebo, bridge, spawn, bringup, rviz,
    ])
