# Riggu ROS2 Differential Drive Robot

This is a comprehensive ROS2 workspace for controlling and operating a differential drive robot called "Riggu". The workspace supports both real hardware operation and Gazebo simulation with full navigation capabilities.

## 🚀 Quick Start

### Prerequisites

#### Option 1: Dev Container (Recommended)
This repository includes a fully configured dev container that provides all necessary dependencies. To use it:

1. **Install Prerequisites**:
   - Docker Desktop or Docker Engine
   - Visual Studio Code
   - [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

2. **Open in Dev Container**:
   - Clone this repository
   - Open the folder in VS Code
   - When prompted, click "Reopen in Container" or use `Ctrl+Shift+P` → "Dev Containers: Reopen in Container"
   - The container will automatically build and configure the environment

3. **Features Included**:
   - ROS2 Kilted Kaiju
   - Ubuntu 24.04 LTS base
   - Gazebo Sim (for simulation)
   - Nav2 stack
   - SLAM Toolbox
   - rmw_zenoh_cpp (Zenoh middleware)
   - All development tools (GDB, Clang, Valgrind, etc.)
   - Automatic Zenoh router startup
   - GPU acceleration support
   - Hardware device access (USB, video)

#### Option 2: Local Installation
If you prefer to install dependencies locally:
- ROS2 Kilted Kaiju
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

**Note**: In the dev container, the ROS environment is automatically sourced in your shell.

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

### Dev Container Setup (Automatic)
When using the dev container, Zenoh is automatically configured and started:
- The container includes a `start_zenoh.sh` script that runs on startup
- Default configuration connects to Riggu's IP: `100.107.192.97:7447`
- To change the robot IP, edit `.devcontainer/start_zenoh.sh` and rebuild the container

### Manual Setup (Non-Dev Container)
If not using the dev container, use these commands:

#### Connect to Remote Zenoh Router (replace IP with Riggu's IP)
```bash
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/100.107.192.97:7447"]'
```

**Note**: Replace `100.107.192.97` with Riggu robot's actual IP address.

#### Start Zenoh Router (on PC)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```


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

## 🛠️ Development Environment

### Dev Container Features
This repository includes a comprehensive dev container configuration that provides:

- **Base Environment**: Ubuntu 24.04 LTS with ROS2 Kilted Kaiju
- **Hardware Access**: USB devices (`/dev/ttyUSB0`) and cameras (`/dev/video0`)
- **GPU Support**: NVIDIA GPU acceleration for Gazebo simulation
- **Networking**: Host network access for robot communication
- **Development Tools**:
  - GDB debugger and GDB server
  - Clang compiler and formatter
  - Valgrind memory analyzer
  - Code coverage tools (lcov, gcovr)
  - Static analysis (cppcheck)
  - Build acceleration (ccache)
- **VS Code Extensions**:
  - C++ tools and IntelliCode
  - Python development suite
  - ROS Development Environment (RDE) pack
  - Markdown tools

### Available VS Code Tasks
- `Clean Workspace` - Remove build artifacts

### Getting Started with Dev Container
1. Open this repository in VS Code
2. Install the Dev Containers extension if not already installed
3. Click "Reopen in Container" when prompted
4. Wait for the container to build (first time only)
5. The environment will be ready with all dependencies pre-installed

### Troubleshooting Dev Container
- **USB Device Access**: Ensure your host system has the required USB devices connected before starting container
- **GPU Issues**: Run `/.devcontainer/test_gpu.sh` to verify GPU acceleration
- **Network Connectivity**: Check that the Zenoh endpoint in `start_zenoh.sh` matches your robot's IP

## ⚙️ Container Configuration

### Customizing the Dev Container
The dev container can be customized by modifying files in `.devcontainer/`:

- **`devcontainer.json`**: Main configuration file
  - Modify VS Code extensions
  - Change container runtime arguments
  - Add additional mounts or environment variables

- **`Dockerfile`**: Container image definition
  - Add system packages
  - Install additional ROS packages
  - Configure build tools

- **`start_zenoh.sh`**: Zenoh router startup script
  - Change robot IP address (`ZENOH_ENDPOINT` variable)
  - Modify connection retry logic
  - Add custom networking setup

### Example: Changing Robot IP
To connect to a different robot, edit `.devcontainer/start_zenoh.sh`:
```bash
ZENOH_ENDPOINT="tcp/YOUR_ROBOT_IP:7447"
```
Then rebuild the container: `Ctrl+Shift+P` → "Dev Containers: Rebuild Container"

## 📞 Support

- **Maintainer**: tarun (taruntom1@gmail.com)
- **License**: TODO: License declaration
- **ROS2 Distribution**: Kilted Kaiju
