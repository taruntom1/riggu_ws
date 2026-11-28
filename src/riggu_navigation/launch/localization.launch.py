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
map_server + AMCL on a saved map: /map and map -> odom.

Runs next to riggu_bringup (which owns odom -> base_footprint through the EKF).
map:= is required (a saved map .yaml). AMCL waits for an initial pose: RViz
"2D Pose Estimate" or /initialpose. Laser range limits come from
riggu_description robot.yaml (lidar.range_min/max).
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


def localization(context, use_sim_time):
    """map_server and AMCL (laser range from robot.yaml), and their lifecycle manager."""
    params = LaunchConfiguration('params_file')
    range_min, range_max = lidar_range(context)
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params, {'yaml_filename': LaunchConfiguration('map'),
                                  'use_sim_time': use_sim_time}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'laser_min_range': range_min,
                                  'laser_max_range': range_max,
                                  'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'node_names': ['map_server', 'amcl'], 'autostart': True,
                          'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    """Build the launch description."""
    share = FindPackageShare('riggu_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Saved map .yaml (~/riggu_maps/<name>.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution(
                [share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument(
            'robot_config', default_value=PathJoinSubstitution(
                [FindPackageShare('riggu_description'), 'config', 'robot.yaml']),
            description='Dimensions file (lidar range limits)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open rviz/navigation.rviz (dev machine only)'),
        OpaqueFunction(function=localization, args=[use_sim_time]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'navigation.rviz'])],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
