# Riggu Bringup Package Documentation

The `riggu_bringup` package is the main orchestration package for the Riggu differential drive robot. It provides launch files, configuration parameters, and coordination between all robot subsystems.

## 📦 Package Overview

- **Package Name**: riggu_bringup
- **Package Type**: Python (ament_python)
- **Dependencies**: rclpy, launch, launch_ros
- **Maintainer**: tarun (taruntom1@gmail.com)

## 🎯 Purpose

This package serves as the central hub for:
- **System Integration**: Coordinates hardware drivers, navigation, and control
- **Configuration Management**: Centralizes parameter files for all subsystems
- **Launch Orchestration**: Provides flexible launch options for different scenarios
- **Environment Adaptation**: Supports both real hardware and simulation modes

## 📁 Package Structure

```
riggu_bringup/
├── config/                         # Configuration files
│   ├── drivelink/                  # Hardware interface configs
│   ├── joy_to_cmdvel/             # Joystick configuration
│   ├── lidar/                     # LIDAR sensor configs
│   ├── slam_toolbox/              # SLAM configuration
│   ├── joystick.yaml              # Joystick control parameters
│   ├── twist_mux_config.yaml      # Command velocity multiplexing
│   ├── nav2_params.yaml           # Navigation parameters
│   └── gz_bridge.yaml             # Gazebo-ROS bridge config
├── launch/                        # Launch files
│   ├── helpers/                   # Modular launch components
│   │   ├── hardware.launch.py     # Hardware driver launches
│   │   ├── joy.launch.py          # Joystick control setup
│   │   ├── nav2_slam.launch.py    # Navigation and SLAM
│   │   └── launch_sim.launch.py   # Gazebo simulation
│   ├── slam_toolbox/              # SLAM-specific launches
│   ├── main.launch.py             # Main system launcher
│   ├── simple_control.launch.py   # Basic control without nav
│   ├── launch_rviz.launch.py      # Visualization
│   └── odom_calibration.launch.py # Odometry calibration
├── worlds/                        # Simulation worlds
│   ├── empty.world                # Basic empty world
│   └── something.sdf              # Custom simulation environment
└── resource/                      # Package resources
```

## 🚀 Launch Files

### Main Launch File: `main.launch.py`

The primary entry point for the entire robot system.

**Arguments:**
- `use_sim` (default: false): Enable simulation mode
- `launch_nav2` (default: true): Enable navigation stack
- `headless` (default: false): Run simulation without GUI

**Components Launched:**
- Hardware drivers (real robot only)
- Robot description (URDF)
- Joystick control system
- Navigation and SLAM (optional)
- Gazebo simulation (simulation mode only)

**Usage Examples:**
```bash
# Real robot with navigation
ros2 launch riggu_bringup main.launch.py

# Simulation with navigation
ros2 launch riggu_bringup main.launch.py use_sim:=true

# Real robot without navigation
ros2 launch riggu_bringup main.launch.py launch_nav2:=false

# Headless simulation
ros2 launch riggu_bringup main.launch.py use_sim:=true headless:=true
```

### Helper Launch Files

#### `hardware.launch.py`
Manages physical hardware components:
- **Drivelink Interface**: Motor control and odometry
- **RPLidar Node**: LIDAR sensor driver

#### `joy.launch.py`
Configures joystick/gamepad control:
- **Joy Node**: Joystick input processing
- **Teleop Node**: Converts joystick to velocity commands
- **Twist Stamper**: Adds timestamps to velocity commands
- **Twist Mux**: Prioritizes different velocity sources

#### `nav2_slam.launch.py`
Navigation and mapping components:
- **SLAM Toolbox**: Simultaneous localization and mapping
- **Nav2 Stack**: Path planning and navigation (optional)

#### `launch_sim.launch.py`
Gazebo simulation setup:
- **Gazebo Sim**: Physics simulation environment
- **Robot Spawning**: Places robot in simulation
- **ROS-Gazebo Bridge**: Communication between ROS and Gazebo

### Specialized Launch Files

#### `simple_control.launch.py`
Minimal setup for basic robot control without navigation:
- Drivelink interface
- Joy-to-cmdvel conversion

#### `launch_rviz.launch.py`
Visualization setup for development and debugging

#### `odom_calibration.launch.py`
Special mode for calibrating odometry parameters

## ⚙️ Configuration Management

### Key Configuration Files

#### Joystick Configuration (`joystick.yaml`)
```yaml
joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05
    autorepeat_rate: 20.0

teleop_node:
  ros__parameters:
    enable_button: 4        # Safety enable button
    enable_turbo_button: 5  # Turbo mode button
    axis_linear: {x: 1}     # Forward/backward stick
    axis_angular: {yaw: 3}  # Rotation stick
    scale_linear: {x: 0.1}  # Normal speed
    scale_angular: {yaw: 0.3}
```

#### Twist Mux Configuration (`twist_mux_config.yaml`)
```yaml
twist_mux:
  ros__parameters:
    topics:
      navigation:           # Nav2 commands
        topic: cmd_vel
        priority: 10
      joystick:            # Manual control
        topic: cmd_vel_joy
        priority: 100      # Higher priority than nav
```

#### Hardware Configuration (`drivelink/node_config.yaml`)
```yaml
serial_config:
  baud: 576000
  port: drivelink

diff_drive_config:
  wheel_radius: 0.06675   # 66.75mm wheels
  wheel_base: 0.39        # 390mm between wheels
```

#### LIDAR Configuration (`lidar/rplidar_c1_config.yaml`)
```yaml
rplidar_node:
  ros__parameters:
    serial_port: "/dev/lidar"
    serial_baudrate: 460800
    frame_id: "laser_frame"
    scan_mode: "Standard"
```

## 🔄 System Flow

### Real Hardware Mode
```
Joystick → Teleop → Twist Stamper → Twist Mux → Drivelink → Motors
                                        ↑
Nav2 Goals → Navigation Stack → cmd_vel ┘
```

### Simulation Mode
```
Joystick → Teleop → Twist Stamper → Twist Mux → Gazebo Bridge → Sim Robot
                                        ↑
Nav2 Goals → Navigation Stack → cmd_vel ┘
```

## 🛡️ Safety Features

1. **Twist Mux Priority**: Joystick commands override navigation
2. **Enable Button**: Joystick requires safety button press
3. **Timeout Protection**: Commands expire if not refreshed
   
## 🔧 Customization

### Adding New Control Sources
1. Add topic configuration to `twist_mux_config.yaml`
2. Set appropriate priority level
3. Configure timeout values

### Modifying Robot Parameters
1. Update `drivelink/node_config.yaml` for physical dimensions
2. Adjust `joystick.yaml` for control sensitivity
3. Modify `nav2_params.yaml` for navigation behavior

### Custom Simulation Worlds
1. Add SDF files to `worlds/` directory
2. Update `launch_sim.launch.py` default world
3. Configure appropriate spawn positions

## 📊 Topics and Services

### Key Topics
- `/cmd_vel` - Final velocity commands to robot
- `/cmd_vel_joy` - Joystick velocity commands
- `/scan` - LIDAR scan data
- `/odom` - Odometry information
- `/tf` - Transform tree

### Important Services
- `/slam_toolbox/*` - SLAM operations
- `/navigate_to_pose` - Navigation goals
- `/clear_costmap` - Reset navigation maps

## 🐛 Troubleshooting

### Common Issues

1. **No Joystick Response**
   - Check device_id in joystick.yaml
   - Verify joystick is connected: `ls /dev/input/js*`
   - Test with: `ros2 topic echo /joy`

2. **Hardware Not Responding**
   - Check serial port permissions
   - Verify drivelink device: `ls -l /dev/drivelink`
   - Monitor: `ros2 topic echo /drivelink/status`

3. **Navigation Issues**
   - Ensure static transforms are published
   - Check LIDAR data: `ros2 topic echo /scan`
   - Verify map is being built: `rviz2`

### Debug Commands
```bash
# Check active topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check transform tree
ros2 run tf2_tools view_frames

# SLAM status
ros2 service list | grep slam
```
