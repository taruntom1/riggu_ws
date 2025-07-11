# Hardware Setup Guide

This guide covers the physical setup, wiring, and configuration of the Riggu differential drive robot.

## 🤖 Robot Overview

The Riggu robot is a differential drive platform designed for autonomous navigation and teleoperation. It consists of:

- **Differential Drive Base**: Two independently controlled wheels
- **LIDAR Sensor**: RPLidar C1 for environment sensing
- **Control Board**: Hardware interface running drivelink protocol
- **Power System**: Battery management and distribution
- **Communication**: Serial interfaces for sensor and control

## 📐 Physical Specifications

### Mechanical Parameters
- **Wheel Radius**: 66.75 mm (0.06675 m)
- **Wheel Base**: 390 mm (0.39 m) - center-to-center distance
- **Robot Footprint**: Approximately 400mm × 300mm
- **Ground Clearance**: Variable based on wheel size
- **Weight**: TBD based on specific build

### Performance Specifications
- **Maximum Linear Speed**: 0.2 m/s (configurable)
- **Maximum Angular Speed**: 0.6 rad/s (configurable)
- **Payload Capacity**: TBD
- **Operating Time**: Depends on battery configuration

## 🔌 Hardware Components

### 1. Drive System
- **Motors**: DC gear motors with encoders
- **Wheels**: 66.75mm radius wheels
- **Motor Driver**: Compatible with drivelink interface
- **Encoders**: Quadrature encoders for odometry

### 2. Sensing System
- **LIDAR**: RPLidar C1
  - Range: 0.15-12m
  - Resolution: 0.33-0.5°
  - Scan Rate: 10Hz
  - Interface: USB/Serial

### 3. Control System
- **Main Controller**: Board running drivelink firmware
- **Communication**: Serial interface at 576000 baud
- **I/O**: GPIO for additional sensors/actuators

### 4. Power System
- **Battery**: Li-Po/Li-Ion battery pack
- **Voltage**: 12V nominal (adjust based on motor requirements)
- **Power Distribution**: 5V/3.3V regulators for electronics
- **Protection**: Fuses, reverse polarity protection

## 🔧 Assembly Instructions

### Step 1: Base Platform Assembly
1. **Mount Drive Motors**
   - Secure motors to chassis frame
   - Ensure proper alignment for differential drive
   - Allow clearance for wheel rotation

2. **Install Wheels**
   - Mount wheels to motor shafts
   - Verify wheel radius matches configuration (66.75mm)
   - Check that wheel base distance is 390mm

3. **Mount Control Electronics**
   - Install main control board
   - Ensure access to serial/USB ports
   - Provide proper ventilation

### Step 2: LIDAR Installation
1. **Mounting Position**
   - Mount RPLidar C1 at robot center (if possible)
   - Height: Above robot base for clear 360° view
   - Orientation: Ensure 0° angle points forward

2. **Wiring**
   - Connect USB cable to host computer
   - Secure cable to prevent disconnection during movement
   - Route away from moving parts

### Step 3: Power System Setup
1. **Battery Installation**
   - Secure battery pack to robot base
   - Ensure weight distribution for stability
   - Provide easy access for charging/replacement

2. **Power Distribution**
   - Wire main power switch
   - Connect motor drivers to battery voltage
   - Install voltage regulators for electronics
   - Add fuses for protection

### Step 4: Control Wiring
1. **Serial Connection**
   - Connect control board to host computer
   - Use USB-to-serial adapter if needed
   - Secure cable connections

2. **Motor Wiring**
   - Connect motors to motor drivers
   - Wire encoder signals to control board
   - Ensure proper motor direction

## 🖥️ Software Setup

### 1. Ubuntu/ROS2 Host Setup

**Install ROS2 Kilted:**
```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2
sudo apt update
sudo apt install ros-kilted-desktop

# Install additional packages
sudo apt install ros-kilted-nav2-bringup ros-kilted-slam-toolbox
sudo apt install ros-kilted-joy ros-kilted-teleop-twist-joy
sudo apt install ros-kilted-twist-mux ros-kilted-rmw-zenoh-cpp
```

**Install Gazebo (for simulation):**
```bash
sudo apt install gz-sim
sudo apt install ros-kilted-ros-gz-sim ros-kilted-ros-gz-bridge
```

### 2. Device Permission Setup

**Create udev rules for hardware devices:**

```bash
# Create drivelink device rule
sudo nano /etc/udev/rules.d/99-riggu-hardware.rules
```

Add the following content:
```bash
# Drivelink interface (adjust vendor/product IDs as needed)
SUBSYSTEM=="tty", ATTRS{idVendor}=="YOUR_VENDOR_ID", ATTRS{idProduct}=="YOUR_PRODUCT_ID", SYMLINK+="drivelink", MODE="666", GROUP="dialout"

# RPLidar C1
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="666", GROUP="dialout"

# Add user to dialout group for serial access
```

```bash
# Apply udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Logout and login for group changes to take effect
```

### 3. Workspace Setup

**Clone and build the workspace:**
```bash
# Create workspace
mkdir -p ~/riggu_ws/src
cd ~/riggu_ws

# Clone repositories (adjust URLs as needed)
git clone https://github.com/taruntom1/riggu_ws --recurse-submodules

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
source /opt/ros/kilted/setup.bash
colcon build

# Source workspace
source install/setup.bash
echo "source ~/riggu_ws/install/setup.bash" >> ~/.bashrc
```

### 4. Zenoh Middleware Setup

**Configure Zenoh middleware:**
```bash
# Set Zenoh as default RMW implementation
echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc
source ~/.bashrc

# The Zenoh router automatically starts on the robot when booted
# For development machines, connect to robot's Zenoh router:
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/ROBOT_IP:7447"]'

# Replace ROBOT_IP with actual robot IP address (e.g., 100.107.192.97)
```

**For detailed Zenoh configuration, see the [Zenoh Configuration Guide](zenoh_configuration.md).**

## 🔧 Hardware Calibration

### 1. Motor Direction and Encoder Setup

**Test motor directions:**
```bash
# Launch minimal control system
ros2 launch riggu_bringup simple_control.launch.py

# Send test commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}' --once

# Verify:
# - Positive linear.x should move robot forward
# - Positive angular.z should rotate robot counter-clockwise
```

**Fix motor directions if needed:**
- Swap motor wires for incorrect direction
- Or configure in drivelink firmware/software

### 2. Wheel Odometry Calibration

**Measure actual wheel parameters:**
```bash
# Mark wheels and measure circumference
# Calculate actual wheel radius
actual_radius = measured_circumference / (2 * π)

# Update config/drivelink/node_config.yaml
wheel_radius: <actual_radius>
```

**Calibrate wheel base:**
```bash
# Launch odometry calibration
ros2 launch riggu_bringup odom_calibration.launch.py

# Command robot to rotate 360 degrees multiple times
# Measure actual rotation vs. commanded rotation
# Adjust wheel_base parameter accordingly
```

### 3. LIDAR Calibration

**Verify LIDAR mounting:**
```bash
# Test LIDAR functionality
ros2 launch rplidar_ros rplidar.launch.py

# Check scan data
ros2 topic echo /scan

# Verify in RViz
ros2 run rviz2 rviz2
# Add LaserScan display, set topic to /scan
```

**Frame alignment:**
- Ensure LIDAR 0° points toward robot front
- Adjust mounting or frame configuration if needed
- Update robot URDF if necessary

## 🔍 Hardware Testing

### 1. Pre-Operational Checks

**Power System:**
```bash
# Check battery voltage
# Verify all systems power on
# Test emergency stop functionality
```

**Communication:**
```bash
# Verify serial devices exist
ls -l /dev/drivelink /dev/lidar

# Test communication
ros2 launch riggu_bringup hardware.launch.py

# Check topics are publishing
ros2 topic list
ros2 topic hz /scan /odom
```

### 2. Movement Testing

**Manual Control Test:**
```bash
# Connect joystick
# Launch control system
ros2 launch riggu_bringup main.launch.py launch_nav2:=false

# Test all movement directions
# Verify safety stops work
# Check for mechanical issues
```

**Autonomous Movement Test:**
```bash
# Launch full system
ros2 launch riggu_bringup main.launch.py

# Build a small map
# Set navigation goals
# Verify autonomous operation
```

### 3. Sensor Validation

**LIDAR Testing:**
```bash
# Check scan quality
ros2 topic echo /scan | grep ranges

# Verify range accuracy with known objects
# Test in different lighting conditions
# Check for interference issues
```

**Odometry Testing:**
```bash
# Drive known distances
# Compare odometry to actual movement
# Test on different surfaces
# Verify angular accuracy
```


### Troubleshooting Common Issues

**Robot Won't Move:**
1. Check battery voltage
2. Verify drivelink connection
3. Test motor drivers

**Poor Navigation:**
1. Verify LIDAR functionality
2. Check map quality
3. Recalibrate odometry
4. Adjust navigation parameters

**Communication Issues:**
1. Check serial connections
2. Verify device permissions
3. Test cables
4. Check USB hub power