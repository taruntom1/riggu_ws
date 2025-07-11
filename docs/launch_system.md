# Launch System Documentation

The Riggu robot uses a modular launch system that allows flexible configuration for different operational modes. This document provides detailed information about each launch file and how they work together.

## 🏗️ Launch Architecture

The launch system follows a hierarchical structure:

```
main.launch.py (Entry Point)
├── hardware.launch.py (Hardware Drivers)
├── joy.launch.py (Control System)
├── nav2_slam.launch.py (Navigation)
├── rsp.launch.py (Robot Description)
└── launch_sim.launch.py (Simulation)
```

## 📋 Launch File Reference

### 1. Main Launch File (`main.launch.py`)

**Purpose**: Central orchestrator for the entire robot system

**Launch Arguments**:
- `use_sim` (bool, default: false): Switch between real hardware and simulation
- `launch_nav2` (bool, default: true): Enable/disable navigation stack
- `headless` (bool, default: false): Run simulation without GUI

**Logic Flow**:
```python
if use_sim == false:
    launch hardware.launch.py
    
launch joy.launch.py
launch nav2_slam.launch.py
launch rsp.launch.py (from riggu_description)

if use_sim == true:
    launch launch_sim.launch.py
```

**Usage Examples**:
```bash
# Default: Real robot with navigation
ros2 launch riggu_bringup main.launch.py

# Simulation mode
ros2 launch riggu_bringup main.launch.py use_sim:=true

# Real robot without navigation
ros2 launch riggu_bringup main.launch.py launch_nav2:=false

# Headless simulation
ros2 launch riggu_bringup main.launch.py use_sim:=true headless:=true
```

### 2. Hardware Launch (`hardware.launch.py`)

**Purpose**: Initialize all physical hardware components

**Components**:
- **Drivelink Interface**: Motor control and odometry feedback
- **RPLidar Node**: LIDAR sensor driver

**Configuration Files Used**:
- `config/drivelink/node_config.yaml`
- `config/drivelink/controller_config.json`
- `config/lidar/rplidar_c1_config.yaml`

**Nodes Launched**:
```yaml
drivelink_interface_node:
  package: drivelink_interface
  executable: drivelink_interface_node
  config: drivelink/node_config.yaml

rplidar_node:
  package: rplidar_ros
  executable: rplidar_node
  config: lidar/rplidar_c1_config.yaml
```

### 3. Joy Control Launch (`joy.launch.py`)

**Purpose**: Set up joystick/gamepad control system with safety features

**Launch Arguments**:
- `use_sim_time` (bool): Use simulation time
- `using_nav2` (bool): Determines output topic routing

**Components**:
- **Joy Node**: Reads joystick input
- **Teleop Node**: Converts joystick to velocity commands
- **Twist Stamper**: Adds timestamps to velocity commands
- **Twist Mux**: Prioritizes different velocity sources

**Topic Flow**:
```
Joystick Input → /joy → teleop_node → /cmd_vel_joy_unstamped
                                           ↓
twist_stamper → /cmd_vel_joy (if using_nav2=true)
             → /cmd_vel_direct (if using_nav2=false)
                    ↓
twist_mux → /diff_drive/cmd_vel (final output)
```

**Dynamic Topic Configuration**:
The launch file uses an `OpaqueFunction` to dynamically set the twist_stamper output topic based on whether Nav2 is being used:

```python
def set_twist_stamper_out_topic(context, *args, **kwargs):
    using_nav2 = LaunchConfiguration('using_nav2')
    if context.perform_substitution(using_nav2).lower() == 'false':
        return [SetLaunchConfiguration('twist_stamper_out_topic', '/cmd_vel_direct')]
    else:
        return [SetLaunchConfiguration('twist_stamper_out_topic', '/cmd_vel_joy')]
```

### 4. Navigation and SLAM Launch (`nav2_slam.launch.py`)

**Purpose**: Initialize navigation and mapping capabilities

**Launch Arguments**:
- `use_sim_time` (bool, default: true): Use simulation time
- `launch_nav2` (bool, default: true): Enable Nav2 stack

**Components**:
- **SLAM Toolbox**: Real-time mapping and localization
- **Nav2 Stack**: Path planning and navigation (conditional)

**Configuration Files**:
- `config/nav2_params.yaml`: Navigation parameters
- `config/slam_toolbox/`: SLAM configuration

**Conditional Launch**:
Nav2 is only launched if `launch_nav2` is true, allowing for SLAM-only operation.

### 5. Simulation Launch (`launch_sim.launch.py`)

**Purpose**: Set up Gazebo simulation environment

**Launch Arguments**:
- `world` (string): Path to world file (default: something.sdf)
- `headless` (bool, default: false): Run without GUI

**Components**:
- **Gazebo Sim**: Physics simulation
- **Entity Spawner**: Places robot in simulation
- **ROS-Gazebo Bridge**: Topic/service bridging

**Gazebo Arguments**:
```python
gz_args = PythonExpression([
    "'-r -v4 -s ' + '", world, "' if '", headless, "' == 'true' else '-r -v4 ' + '", world, "'"
])
```

**Bridge Configuration**:
Uses `config/gz_bridge.yaml` to define topic mappings between ROS and Gazebo.

### 6. Simple Control Launch (`simple_control.launch.py`)

**Purpose**: Minimal setup for basic robot operation without navigation

**Components**:
- Drivelink interface
- Joy-to-cmdvel conversion (simplified)

**Use Cases**:
- Hardware testing
- Direct teleoperation
- Development and debugging

### 7. Specialized Launch Files

#### SLAM Toolbox Launches (`launch/slam_toolbox/`)

Multiple SLAM modes available:
- `online_sync_launch.py`: Real-time synchronous SLAM
- `online_async_launch.py`: Real-time asynchronous SLAM
- `offline_launch.py`: Post-processing SLAM
- `localization_launch.py`: Pure localization mode
- `merge_maps_kinematic_launch.py`: Map merging

#### RViz Launch (`launch_rviz.launch.py`)

Starts RViz with appropriate configuration for visualization.

#### Odometry Calibration (`odom_calibration.launch.py`)

Special mode for calibrating wheel odometry parameters.

## 🔧 Launch File Patterns

### Conditional Launching
```python
# Launch only if condition is met
hardware_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(hardware_launch_file),
    condition=UnlessCondition(LaunchConfiguration('use_sim'))
)
```

### Argument Passing
```python
# Pass arguments to included launch files
nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_launch_file),
    launch_arguments={
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': params_file
    }.items()
)
```

### Dynamic Configuration
```python
# Use OpaqueFunction for runtime decisions
return LaunchDescription([
    OpaqueFunction(function=set_twist_stamper_out_topic),
    # ... other actions
])
```

## 🎛️ Launch Argument Reference

### Global Arguments (main.launch.py)
| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| use_sim | bool | false | Enable simulation mode |
| launch_nav2 | bool | true | Launch navigation stack |
| headless | bool | false | Run simulation without GUI |

### Component-Specific Arguments
| Component | Argument | Type | Default | Description |
|-----------|----------|------|---------|-------------|
| joy.launch.py | use_sim_time | bool | false | Use simulation time |
| joy.launch.py | using_nav2 | bool | true | Route commands for Nav2 |
| launch_sim.launch.py | world | string | something.sdf | World file to load |

## 📊 Launch Dependencies

### Package Dependencies
```
riggu_bringup depends on:
├── drivelink_interface (hardware driver)
├── riggu_description (robot model)
├── rplidar_ros (LIDAR driver)
├── nav2_bringup (navigation)
├── slam_toolbox (mapping)
├── joy (joystick input)
├── teleop_twist_joy (joystick control)
├── twist_stamper (utility)
├── twist_mux (command prioritization)
├── ros_gz_sim (simulation)
└── ros_gz_bridge (sim integration)
```

### Launch File Dependencies
```
main.launch.py includes:
├── hardware.launch.py
│   ├── drivelink_interface_node.launch.py
│   └── (rplidar_node direct)
├── joy.launch.py
│   └── (multiple nodes direct)
├── nav2_slam.launch.py
│   ├── online_sync_launch.py (SLAM)
│   └── navigation_launch.py (Nav2)
├── rsp.launch.py (from riggu_description)
└── launch_sim.launch.py
    └── gz_sim.launch.py (from ros_gz_sim)
```

## 🚀 Custom Launch File Creation

### Template Structure
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    my_arg = DeclareLaunchArgument(
        'my_argument',
        default_value='default_value',
        description='Description of argument'
    )
    
    # Create nodes
    my_node = Node(
        package='my_package',
        executable='my_executable',
        parameters=[LaunchConfiguration('my_argument')]
    )
    
    return LaunchDescription([
        my_arg,
        my_node
    ])
```

### Best Practices
1. **Always provide default values** for launch arguments
2. **Include descriptive help text** for arguments
3. **Use consistent naming** across related launch files
4. **Validate file paths** before using them
5. **Use conditions** for optional components
6. **Pass sim_time consistently** throughout the system

## 🐛 Launch Debugging

### Common Issues
1. **Missing launch arguments**: Check argument names and defaults
2. **File not found errors**: Verify package installation and file paths
3. **Node startup failures**: Check dependencies and configurations
4. **Parameter loading issues**: Validate YAML syntax and file paths

### Debug Commands
```bash
# Validate launch file syntax
ros2 launch --debug riggu_bringup main.launch.py

# List available launch arguments
ros2 launch riggu_bringup main.launch.py --show-args

# Dry run (show what would be launched)
ros2 launch --debug --dry-run riggu_bringup main.launch.py

# Monitor active nodes
ros2 node list

# Check parameter loading
ros2 param list
```
