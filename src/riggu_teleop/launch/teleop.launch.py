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

"""
Manual override: twist_mux + stop_latch, and with joy:=true the pad nodes.

  /joy -> teleop_twist_joy -> /cmd_vel_joy ┐
                              /cmd_vel_nav ┴→ twist_mux -> cmd_vel_out (/cmd_vel_mux)
  /joy -> stop_latch -> /estop (lock) ─────┘
  /joy -> arm_switch -> controller_manager switch_controller   (arm:=true)

riggu_bringup/bringup.launch.py includes this; cmd_vel_out and nav_in are its
collision-monitor wiring.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description."""
    config = PathJoinSubstitution([FindPackageShare('riggu_teleop'), 'config'])
    joy_config = LaunchConfiguration('joy_config')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    joy = IfCondition(LaunchConfiguration('joy'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joy', default_value='true', choices=['true', 'false'],
                              description='start joy_node and teleop_twist_joy'),
        DeclareLaunchArgument('arm', default_value='false', choices=['true', 'false'],
                              description='start arm_switch (pad arms/disarms the drive)'),
        DeclareLaunchArgument('joy_config',
                              default_value=PathJoinSubstitution([config, 'joy_generic.yaml']),
                              description='pad mapping and scales'),
        DeclareLaunchArgument('mux_config',
                              default_value=PathJoinSubstitution([config, 'twist_mux.yaml'])),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel_mux',
                              description='twist_mux output topic'),
        DeclareLaunchArgument('nav_in', default_value='/cmd_vel_nav',
                              description='topic twist_mux reads as its nav input'),

        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[LaunchConfiguration('mux_config'), use_sim_time],
             remappings=[('cmd_vel_out', LaunchConfiguration('cmd_vel_out')),
                         ('/cmd_vel_nav', LaunchConfiguration('nav_in'))],
             output='screen'),
        Node(package='riggu_teleop', executable='stop_latch', name='stop_latch',
             parameters=[joy_config, use_sim_time], output='screen'),
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joy_config, use_sim_time], condition=joy, output='screen'),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
             parameters=[joy_config, use_sim_time],
             remappings=[('cmd_vel', '/cmd_vel_joy')], condition=joy, output='screen'),
        Node(package='riggu_teleop', executable='arm_switch', name='arm_switch',
             parameters=[joy_config, use_sim_time],
             condition=IfCondition(LaunchConfiguration('arm')), output='screen'),
    ])
