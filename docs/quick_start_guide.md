# Riggu Quick Start Guide

This guide will get you up and running with the Riggu differential drive robot in just a few steps. Follow this guide to power on the robot, establish communication, and start the main control system.

## 🔋 Prerequisites

Before starting, ensure you have:
- Riggu robot hardware powered on and accessible
- Development machine with ROS2 Kilted Kaiju installed, or the devcontainer running
- Robot IP address (default: `100.107.192.97`)

## ⚡ Quick Start Steps

### Step 1: Power On the Robot

1. **Connect Power Supply**: Plug in the robot's battery
2. **Power On**: Press the main power button on the raspberry pi
3. **Wait for Boot**: Allow 30-60 seconds for the robot to fully boot up
4. **Network Connectivity**: To connect the robot to your network, set up a WiFi Access Point (AP) with the following credentials:
   - **SSID**: Riggu
   - **Password**: getconnected
   - The AP can be either 5GHz or 2.4GHz.
   - Once the robot boots, it will automatically connect to this AP.
5. **Verify Status**: Check that status LEDs indicate normal operation

### Step 2: Establish Network Connection (on your PC)

#### Option A: Using Dev Container (Recommended)
If you're using the provided dev container, Zenoh middleware is automatically configured:

**Before starting the container:**
1. **Set Robot IP**: Edit `.devcontainer/start_zenoh.sh` and update the `ZENOH_ENDPOINT` variable with your robot's IP address:
   ```bash
   # Example: Change from default to your robot's IP
   ZENOH_ENDPOINT="tcp/YOUR_ROBOT_IP:7447"
   ```

2. **Start/Rebuild Container**: 
   ```bash
   # If container is not running, start it normally
   # If container is already running, rebuild it to apply IP changes:
   # Ctrl+Shift+P → "Dev Containers: Rebuild Container"
   ```

Once the container is running with the correct IP:
```bash
# The dev container automatically starts Zenoh with the correct configuration
# No additional setup required - proceed to Step 3
```

#### Option B: Manual Zenoh Setup
If you're not using the dev container, configure Zenoh manually:

```bash
# Set Zenoh as the RMW implementation
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Start the Zenoh router on your PC (or robot, if not already running)
ros2 run rmw_zenoh_cpp rmw_zenohd

# Connect to the robot's Zenoh router (replace IP with robot's actual IP)
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/100.107.192.97:7447"]'

# Verify connection (optional)
ros2 node list
```

### Step 3: Launch the Robot System (on the robot)

Now you're ready to launch the main robot control system:

#### For Hardware with Navigation
```bash
ros2 launch riggu_bringup main.launch.py launch_nav2:=true
```

#### For Simple Control (No Navigation)
```bash
ros2 launch riggu_bringup simple_control.launch.py launch_nav2:=false
```

### Step 4: Verify Operation

After launching, verify that the system is working:

```bash
# Check that nodes are running
ros2 node list

# Verify topic communication
ros2 topic list

# Test robot movement (if safe to do so)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

## 🎮 Control Options

Once the robot is running, you can control it using:

### Joystick Control
If you have a joystick/gamepad connected:
```bash
# The joystick should automatically work with the main launch
# Use the left stick for linear movement, right stick for rotation
```

### Navigation Goals (if Nav2 is launched)
To use the launch file below, the workspace must be built and installed on your PC:
```bash
# Launch RViz for visualization and goal setting
ros2 launch riggu_bringup launch_rviz.launch.py
```
If the workspace is not built/installed locally, you can run RViz directly with a config file:
```bash
rviz2 -d path/to/your/rviz_config.rviz
```

## 🔧 Troubleshooting

### Connection Issues

**Problem**: Cannot connect to robot
```bash
# Check network connectivity
ping <YOUR_ROBOT_IP>

# Verify Zenoh configuration
echo $ZENOH_CONFIG_OVERRIDE

# Test with different endpoint
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/YOUR_ROBOT_IP:7447"]'
```

**Problem**: No topics visible
```bash
# Check RMW implementation
echo $RMW_IMPLEMENTATION

# Should output: rmw_zenoh_cpp
# If not, set it:
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

### Launch Issues

**Problem**: Build errors
```bash
# Clean and rebuild
colcon build --packages-select riggu_bringup --cmake-clean-cache
```

**Problem**: Node startup failures
```bash
# Check dependencies
rosdep check riggu_bringup

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Hardware Issues

**Problem**: Robot doesn't respond to commands
```bash
# Check drivelink interface
ros2 topic echo /drivelink/status

# Check hardware connection
ros2 service call /drivelink/get_status std_srvs/srv/Empty
```

**Problem**: LIDAR not working
```bash
# Check LIDAR topics
ros2 topic echo /scan

# Verify LIDAR node
ros2 node info /rplidar_composition
```

## 📊 Monitoring and Diagnostics

### System Status
```bash
# Check all nodes
ros2 node list

# Monitor system diagnostics
ros2 topic echo /diagnostics

# View robot state
ros2 topic echo /robot_state_publisher/robot_description
```

### Performance Monitoring
```bash
# Check CPU usage
top

# Monitor network traffic
iftop

# ROS2 performance
ros2 doctor
```

## 🛑 Safe Shutdown

To safely shut down the robot:

1. **Stop all movement**:


2. **Terminate launch processes**:
   ```bash
   # Press Ctrl+C in the terminal running the launch file
   ```

3. **Power down robot**:
   - Wait for all processes to terminate
   - Run the following command for a safe shutdown:
     ```bash
     sudo shutdown now
     ```

## 📚 Next Steps

Once you have the robot running:

- **Navigation**: See [Navigation Guide](navigation.md) for autonomous navigation
- **Configuration**: Check [Configuration Guide](configuration.md) for parameter tuning
- **Development**: Read [Launch System](launch_system.md) for advanced launch options
- **Simulation**: Try [Simulation Guide](simulation.md) for testing without hardware

## 🆘 Emergency Procedures

### Emergency Stop
```bash
# Immediate stop command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

Alternatively, you can immediately stop all robot activity by pressing Ctrl+C in the terminal window running the launch file.

### System Recovery
```bash
# If system becomes unresponsive:
# 1. Ctrl+C to stop all processes
# 2. Power cycle the robot
# 3. Restart from Step 1 of this guide
```
---

*This guide covers the essential steps to get Riggu operational. For advanced configuration and troubleshooting, consult the detailed documentation in the `docs/` folder.*
