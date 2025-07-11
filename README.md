# Riggu ROS2 Differential Drive Robot

This is a comprehensive ROS2 workspace for controlling and operating a differential drive robot called "Riggu". The workspace supports both real hardware operation and Gazebo simulation with full navigation capabilities.

## 🚀 Quick Start

### Prerequisites
- ROS2 Kilted Kingsnake
- Ubuntu 24.04 LTS
- Gazebo Sim (for simulation)
- Nav2 stack
- SLAM Toolbox
- rmw_zenoh_cpp (Zenoh middleware)

### Build the Workspace
```bash
source /opt/ros/kilted/setup.bash
colcon build
source install/setup.bash
```

### Launch Options

#### Real Hardware with Navigation
```bash
ros2 launch riggu_bringup main.launch.py use_sim:=false launch_nav2:=true
```

#### Simulation with Navigation
```bash
ros2 launch riggu_bringup main.launch.py use_sim:=true launch_nav2:=true
```

#### Simple Control (No Navigation)
```bash
ros2 launch riggu_bringup simple_control.launch.py
```

## 📁 Workspace Structure

```
riggu_ws/
├── src/
│   ├── drivelink_ros_interface/     # Low-level hardware interface
│   ├── riggu_bringup/              # Main launch and configuration package
│   ├── riggu_description/          # Robot URDF and description
│   ├── rplidar_ros/                # LIDAR driver
│   └── twist_stamper/              # Velocity command stamping utility
├── build/                          # Build artifacts
├── install/                        # Installation files
├── log/                           # Build and runtime logs
└── maps/                          # Saved maps for navigation
```

## 🤖 Robot Specifications

- **Drive Type**: Differential Drive
- **Wheel Radius**: 0.06675 m (66.75 mm)
- **Wheel Base**: 0.39 m (390 mm)
- **LIDAR**: RPLidar C1
- **Control Interface**: Serial communication via drivelink
- **Middleware**: rmw_zenoh_cpp (Zenoh for distributed communication)

## 🌐 Network Communication (Zenoh Middleware)

The Riggu robot uses Zenoh middleware for efficient distributed communication between robot and control stations.

### Automatic Setup (Recommended)
The Zenoh router automatically starts on the robot when booted. For development containers, adjust the `start_zenoh.sh` script to point to the Riggu's IP address, and the Zenoh router will start automatically.

### Manual Setup
If automatic setup is not available, use these commands:

#### Start Zenoh Router (on PC)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

#### Connect to Remote Zenoh Router (replace IP with Riggu's IP)
```bash
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/100.107.192.97:7447"]'
```

**Note**: Replace `100.107.192.97` with your Riggu robot's actual IP address.

## 📚 Documentation

For detailed information about each component:

- [Riggu Bringup Package](docs/riggu_bringup.md) - Main launch and configuration
- [Launch System](docs/launch_system.md) - Detailed launch file documentation
- [Configuration Guide](docs/configuration.md) - Parameter configuration
- [Hardware Setup](docs/hardware_setup.md) - Physical robot setup
- [Simulation Guide](docs/simulation.md) - Using Gazebo simulation
- [Navigation Guide](docs/navigation.md) - Nav2 and SLAM usage
- [Zenoh Configuration](docs/zenoh_configuration.md) - Network middleware setup

## 🎮 Control Methods

1. **Joystick/Gamepad Control**: Direct teleoperation with safety features
2. **Navigation Goals**: Autonomous navigation using Nav2
3. **Direct Command Velocity**: Programmatic control via `/cmd_vel` topics

## 🛠️ Development Tasks

Available VS Code tasks:
- `build` - Build the drivelink_interface package
- `Build and Run drivelink_interface` - Build and launch the main system
- `Clean Workspace` - Remove build artifacts

## 📞 Support

- **Maintainer**: tarun (taruntom1@gmail.com)
- **License**: TODO: License declaration
- **ROS2 Distribution**: Kilted Kingsnake
