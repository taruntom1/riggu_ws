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

robot_localization ekf_node (every mode) owns odom -> base_footprint and publishes
/odometry/filtered from /wheel/twist and /imu (config/ekf.yaml). wheel_twist turns
/joint_states into /wheel/twist, armed or not (diff_drive_controller's /odom stops
while it is inactive). use_imu_orientation:=false
(the default) fuses the gyro only (config/ekf_gyro_only.yaml); imu:=false fuses the wheels
only. imu_scale corrects the BNO055 gyro scale (robot.yaml imu.gyro_z_scale, real only):
/imu -> /imu/corrected, the EKF's IMU input.

cmd_vel chain (riggu_teleop/teleop.launch.py + nav2 collision_monitor):
  /cmd_vel_joy, /cmd_vel_nav, /estop -> twist_mux -> /cmd_vel_mux -> collision_monitor
    -> /cmd_vel_safe -> diff_drive_controller
collision_check_joy:=false moves collision_monitor onto the nav input only:
  /cmd_vel_nav -> collision_monitor -> /cmd_vel_nav_checked -> twist_mux -> /cmd_vel_safe
joy/arm:=auto start the pad nodes and the pad arm switch on mode:=real only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro
import yaml

DRIVELINK_ARGS = ('agent', 'device', 'token_file')
DRIVELINK_LABELS = ('left_label', 'right_label')


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
    mappings.update({f'drivelink_{n}': arg(f'drivelink_{n}')
                     for n in DRIVELINK_ARGS + DRIVELINK_LABELS})
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
        # /diagnostics + ack_fault/recover. recover restores the hardware and the
        # broadcasters only: like launch, it never re-activates (arms) the drive.
        recovery = ['joint_state_broadcaster'] + (['imu_sensor_broadcaster'] if imu else [])
        drivelink = {n: LaunchConfiguration(f'drivelink_{n}').perform(context)
                     for n in DRIVELINK_ARGS}
        nodes.append(Node(package='drivelink_bringup', executable='diagnostics',
                          parameters=[drivelink, {'hardware': 'drivelink',
                                                  'recovery_controllers': recovery}],
                          output='screen'))
    return nodes


def ekf_params(context):
    """ekf.yaml params, with the gyro-only overrides and without the IMU as the args ask."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def load(path):
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f)['ekf_filter_node']['ros__parameters']
    params = load(arg('ekf_config'))
    if arg('use_imu_orientation') != 'true':
        params.update(load(arg('ekf_gyro_only_config')))
    if arg('imu') != 'true':
        params = {k: v for k, v in params.items() if not k.startswith('imu0')}
    return params


def ekf(context, use_sim_time):
    """wheel_twist (/joint_states -> /wheel/twist) and the robot_localization ekf_node."""
    with open(LaunchConfiguration('robot_config').perform(context), encoding='utf-8') as f:
        wheel = yaml.safe_load(f)['wheel']
    wheel_params = {'wheel_radius': float(wheel['radius']['value']),
                    'wheel_separation': float(wheel['separation']['value']),
                    'use_sim_time': use_sim_time}
    config = PathJoinSubstitution(
        [FindPackageShare('riggu_bringup'), 'config', 'wheel_twist.yaml'])
    nodes = [Node(package='riggu_bringup', executable='wheel_twist', name='wheel_twist',
                  parameters=[config, wheel_params], output='screen')]
    if LaunchConfiguration('imu').perform(context) == 'true':
        nodes.append(Node(package='riggu_bringup', executable='imu_scale', name='imu_scale',
                          parameters=[{'gyro_z_scale': gyro_z_scale(context),
                                       'use_sim_time': use_sim_time}],
                          output='screen'))
    return nodes + [
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[ekf_params(context), {'use_sim_time': use_sim_time}],
             output='screen'),
    ]


def gyro_z_scale(context):
    """Return the real BNO055's gyro scale from robot.yaml; 1.0 for sim and mock."""
    if LaunchConfiguration('mode').perform(context) != 'real':
        return 1.0
    with open(LaunchConfiguration('robot_config').perform(context), encoding='utf-8') as f:
        return float(yaml.safe_load(f)['imu']['gyro_z_scale']['value'])


def footprint(robot, margin):
    """Footprint polygon (base_footprint, axle at x=0) around chassis and wheels."""
    def v(group, key):
        return robot[group][key]['value']
    front = max(v('chassis', 'axle_from_front'), v('wheel', 'radius')) + margin
    rear = v('chassis', 'axle_from_front') - v('chassis', 'length') - margin
    half = max(v('chassis', 'width') / 2.0,
               v('wheel', 'separation') / 2.0 + v('wheel', 'width') / 2.0) + margin
    return [[front, half], [front, -half], [rear, -half], [rear, half]]


def cmd_vel_chain(context, use_sim_time):
    """twist_mux, stop latch, pad nodes (riggu_teleop) and collision_monitor."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    auto = {'auto': 'true' if mode == 'real' else 'false'}
    joy, arm = auto.get(arg('joy'), arg('joy')), auto.get(arg('arm'), arg('arm'))
    check_joy = arg('collision_check_joy') == 'true'

    config = arg('collision_monitor_config')
    with open(config, encoding='utf-8') as f:
        margin = yaml.safe_load(f)['riggu_footprint']['ros__parameters']['margin']
    with open(arg('robot_config'), encoding='utf-8') as f:
        robot = yaml.safe_load(f)
    points = ', '.join(f'[{x:.4f}, {y:.4f}]' for x, y in footprint(robot, margin))
    monitor = {
        'cmd_vel_in_topic': '/cmd_vel_mux' if check_joy else '/cmd_vel_nav',
        'cmd_vel_out_topic': '/cmd_vel_safe' if check_joy else '/cmd_vel_nav_checked',
        'FootprintApproach.points': f'[{points}]',
        'use_sim_time': use_sim_time,
    }
    if mode == 'mock':  # no lidar: don't treat the missing /scan as a stop
        monitor['scan.source_timeout'] = 0.0

    teleop = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('riggu_teleop'), 'launch', 'teleop.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time, 'joy': joy, 'arm': arm,
            'cmd_vel_out': '/cmd_vel_mux' if check_joy else '/cmd_vel_safe',
            'nav_in': '/cmd_vel_nav' if check_joy else '/cmd_vel_nav_checked',
        }.items())
    return [
        teleop,
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', parameters=[config, monitor], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_collision_monitor',
             parameters=[{'node_names': ['collision_monitor'], 'autostart': True,
                          'use_sim_time': use_sim_time}],
             output='screen'),
    ]


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
        DeclareLaunchArgument('drivelink_left_label', default_value='pid_left',
                              description='real: DriveLink controller label, left wheel'),
        DeclareLaunchArgument('drivelink_right_label', default_value='pid_right',
                              description='real: DriveLink controller label, right wheel'),
        DeclareLaunchArgument(
            'lidar_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('riggu_bringup'), 'config', 'lidar.yaml']),
            description='real: sllidar_node params'),
        DeclareLaunchArgument('use_imu_orientation', default_value='false',
                              choices=['true', 'false'],
                              description='EKF: also fuse BNO055 absolute (magnetometer) yaw'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('riggu_bringup'), 'config', 'ekf.yaml'])),
        DeclareLaunchArgument(
            'ekf_gyro_only_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('riggu_bringup'), 'config', 'ekf_gyro_only.yaml']),
            description='overrides on ekf_config for use_imu_orientation:=false'),
        DeclareLaunchArgument('joy', default_value='auto', choices=['auto', 'true', 'false'],
                              description='joy_node + teleop_twist_joy (auto: real only)'),
        DeclareLaunchArgument('arm', default_value='auto', choices=['auto', 'true', 'false'],
                              description='pad arm/disarm switch (auto: real only)'),
        DeclareLaunchArgument('collision_check_joy', default_value='true',
                              choices=['true', 'false'],
                              description='false: joystick bypasses the collision monitor'),
        DeclareLaunchArgument(
            'collision_monitor_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('riggu_bringup'), 'config', 'collision_monitor.yaml'])),

        OpaqueFunction(function=launch_setup, args=[controllers, use_sim_time]),
        OpaqueFunction(function=cmd_vel_chain, args=[use_sim_time]),
        OpaqueFunction(function=ekf, args=[use_sim_time]),

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
