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

"""Show the riggu model: robot_state_publisher, joint_state_publisher_gui, RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description."""
    share = FindPackageShare('riggu_description')
    use_sim_time = LaunchConfiguration('use_sim_time')

    description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([share, 'urdf', 'riggu.urdf.xacro']),
            ' mode:=', LaunchConfiguration('mode'),
            ' robot_config:=', LaunchConfiguration('robot_config'),
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock',
                              description='ros2_control hardware: real|sim|mock'),
        DeclareLaunchArgument('robot_config',
                              default_value=PathJoinSubstitution([share, 'config', 'robot.yaml']),
                              description='Dimensions file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'display.rviz'])],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
