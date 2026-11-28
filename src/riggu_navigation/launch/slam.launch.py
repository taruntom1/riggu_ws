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
slam_toolbox online async: /scan + odom TF -> /map and map -> odom.

Runs next to riggu_bringup (which owns odom -> base_footprint through the EKF).
Laser range limits come from riggu_description robot.yaml (lidar.range_min/max).
Lifecycle: nav2 lifecycle_manager_slam (autostart, bond), like the other Nav2 nodes.
Save the map with scripts/save_map.sh <name>.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def lidar_range(context):
    """Return (range_min, range_max) of the lidar from robot.yaml."""
    with open(LaunchConfiguration('robot_config').perform(context), encoding='utf-8') as f:
        lidar = yaml.safe_load(f)['lidar']
    return float(lidar['range_min']['value']), float(lidar['range_max']['value'])


def slam(context, use_sim_time):
    """slam_toolbox with the laser range from robot.yaml, and its lifecycle manager."""
    range_min, range_max = lidar_range(context)
    return [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[LaunchConfiguration('slam_config'),
                         {'min_laser_range': range_min, 'max_laser_range': range_max,
                          'use_lifecycle_manager': True, 'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_slam', output='screen',
             parameters=[{'node_names': ['slam_toolbox'], 'autostart': True,
                          'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    """Build the launch description."""
    share = FindPackageShare('riggu_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_config', default_value=PathJoinSubstitution(
                [share, 'config', 'slam_toolbox.yaml'])),
        DeclareLaunchArgument(
            'robot_config', default_value=PathJoinSubstitution(
                [FindPackageShare('riggu_description'), 'config', 'robot.yaml']),
            description='Dimensions file (lidar range limits)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open rviz/mapping.rviz (dev machine only)'),
        OpaqueFunction(function=slam, args=[use_sim_time]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'mapping.rviz'])],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
