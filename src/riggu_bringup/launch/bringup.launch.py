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
riggu base bring-up: robot_state_publisher, ros2_control, controller spawners.

mode:=real|sim|mock picks the ros2_control hardware (see riggu_description).
In sim, Gazebo runs the controller manager (gz_ros2_control), so no
ros2_control_node is started here; riggu_gazebo/sim.launch.py includes this.

real also starts the lidar driver and DriveLink diagnostics, and loads
diff_drive_controller *inactive*: activating it arms the DriveLink motors, so
that is always an explicit `ros2 control switch_controllers --activate`.

imu:=false drops the `imu` sensor from the expanded ros2_control block and the
imu_sensor_broadcaster (robot without an IMU; DriveLink refuses a missing instance).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro
import yaml

DRIVELINK_ARGS = ('agent', 'device', 'token_file')


def strip_imu(doc):
    """Remove <sensor name="imu"> from every <ros2_control> block of a URDF DOM."""
    for block in doc.getElementsByTagName('ros2_control'):
        for sensor in block.getElementsByTagName('sensor'):
            if sensor.getAttribute('name') == 'imu':
                block.removeChild(sensor)
    return doc


def robot_description(context, controllers):
    """Expand riggu.urdf.xacro for the launch args; without the IMU on imu:=false."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    mappings = {'mode': arg('mode'), 'robot_config': arg('robot_config'),
                'controllers_file': controllers.perform(context)}
    mappings.update({f'drivelink_{n}': arg(f'drivelink_{n}') for n in DRIVELINK_ARGS})
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('riggu_description'), 'urdf', 'riggu.urdf.xacro']).perform(context)
    doc = xacro.process_file(xacro_file, mappings=mappings)
    if arg('imu') != 'true':
        strip_imu(doc)
    return doc.toxml()


def launch_setup(context, controllers, use_sim_time):
    """RSP, IMU broadcaster and DriveLink diagnostics: the nodes that depend on imu:=."""
    imu = LaunchConfiguration('imu').perform(context) == 'true'
    mode = LaunchConfiguration('mode').perform(context)
    nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                  parameters=[{'robot_description': ParameterValue(
                      robot_description(context, controllers), value_type=str),
                      'use_sim_time': use_sim_time}],
                  output='screen')]
    if imu:
        nodes.append(spawner('imu_sensor_broadcaster', controllers, use_sim_time,
                             ['-r ~/imu:=/imu']))
    if mode == 'real':
        # /diagnostics + ack_fault/recover; recover re-activates the drive, i.e. arms it.
        recovery = ['joint_state_broadcaster', 'diff_drive_controller']
        if imu:
            recovery.insert(1, 'imu_sensor_broadcaster')
        drivelink = {n: LaunchConfiguration(f'drivelink_{n}').perform(context)
                     for n in DRIVELINK_ARGS}
        nodes.append(Node(package='drivelink_bringup', executable='diagnostics',
                          parameters=[drivelink, {'hardware': 'drivelink',
                                                  'recovery_controllers': recovery}],
                          output='screen'))
    return nodes


def drive_spawner(context, controllers, use_sim_time):
    """Spawn diff_drive_controller with wheel params and limits from robot.yaml."""
    mode = LaunchConfiguration('mode').perform(context)
    with open(LaunchConfiguration('robot_config').perform(context), encoding='utf-8') as f:
        robot = yaml.safe_load(f)
    wheel, limits = robot['wheel'], robot['limits']
    params = {
        'wheel_radius': wheel['radius']['value'],
        'wheel_separation': wheel['separation']['value'],
        'linear.x.max_velocity': limits['max_linear_velocity']['value'],
        'linear.x.min_velocity': -limits['max_linear_velocity']['value'],
        'linear.x.max_acceleration': limits['max_linear_accel']['value'],
        'linear.x.min_acceleration': -limits['max_linear_accel']['value'],
        'angular.z.max_velocity': limits['max_angular_velocity']['value'],
        'angular.z.min_velocity': -limits['max_angular_velocity']['value'],
        'angular.z.max_acceleration': limits['max_angular_accel']['value'],
        'angular.z.min_acceleration': -limits['max_angular_accel']['value'],
    }
    ros_args = ['-r ~/cmd_vel:=/cmd_vel_safe', '-r ~/odom:=/odom']
    ros_args += [f'-p {name}:={float(value)}' for name, value in params.items()]
    return [spawner('diff_drive_controller', controllers, use_sim_time, ros_args,
                    inactive=mode == 'real')]


def spawner(name, controllers, use_sim_time, ros_args=(), inactive=False):
    """Return a spawner node for one controller (loaded inactive if asked)."""
    args = [name, '--param-file', controllers, '--controller-manager-timeout', '120']
    if inactive:
        args.append('--inactive')
    # '=' form: argparse would read a bare '-p ...' value as the spawner's own -p
    args += [f'--controller-ros-args={arg}' for arg in ros_args]
    return Node(package='controller_manager', executable='spawner', name=f'spawner_{name}',
                arguments=args, parameters=[{'use_sim_time': use_sim_time}],
                output='screen')


def generate_launch_description():
    """Build the launch description."""
    description_share = FindPackageShare('riggu_description')
    controllers = PathJoinSubstitution(
        [FindPackageShare('riggu_bringup'), 'config', 'controllers.yaml'])
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_sim = PythonExpression(["'", mode, "' == 'sim'"])
    is_real = PythonExpression(["'", mode, "' == 'real'"])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock',
                              choices=['real', 'sim', 'mock'],
                              description='ros2_control hardware'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu', default_value='true', choices=['true', 'false'],
                              description='false: no imu sensor / imu_sensor_broadcaster'),
        DeclareLaunchArgument(
            'robot_config',
            default_value=PathJoinSubstitution([description_share, 'config', 'robot.yaml']),
            description='Dimensions file (wheel params and limits for the controller)'),
        DeclareLaunchArgument('drivelink_agent', default_value='tcp://127.0.0.1:47620',
                              description='real: DriveLink agent URL'),
        DeclareLaunchArgument('drivelink_device', default_value='base',
                              description='real: device alias in /etc/drivelink/agent.json'),
        DeclareLaunchArgument('drivelink_token_file', default_value='/etc/drivelink/token',
                              description='real: agent token (user in the drivelink group)'),
        DeclareLaunchArgument(
            'lidar_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('riggu_bringup'), 'config', 'lidar.yaml']),
            description='real: sllidar_node params'),

        OpaqueFunction(function=launch_setup, args=[controllers, use_sim_time]),

        # real and mock; in sim the controller manager lives in Gazebo
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers, {'use_sim_time': use_sim_time}],
             remappings=[('~/robot_description', '/robot_description')],
             condition=UnlessCondition(is_sim),
             output='screen'),

        # real only: RPLIDAR C1
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             parameters=[LaunchConfiguration('lidar_config')],
             respawn=True, respawn_delay=2.0,
             condition=IfCondition(is_real), output='screen'),

        spawner('joint_state_broadcaster', controllers, use_sim_time),
        OpaqueFunction(function=drive_spawner, args=[controllers, use_sim_time]),
    ])
