# Configuration Guide

This document provides comprehensive information about configuring the Riggu robot system. All configuration files are located in the `config/` directory of the `riggu_bringup` package.

## 📁 Configuration Structure

```
config/
├── drivelink/                  # Hardware interface configuration
│   ├── node_config.yaml        # Serial and robot parameters
│   └── controller_config.json  # Motor controller settings
├── joy_to_cmdvel/             # Alternative joystick configuration
├── lidar/                     # LIDAR sensor configuration
│   └── rplidar_c1_config.yaml # RPLidar C1 settings
├── slam_toolbox/              # SLAM configuration files
├── joystick.yaml              # Joystick control parameters
├── twist_mux_config.yaml      # Command velocity prioritization
├── nav2_params.yaml           # Navigation stack parameters
├── gz_bridge.yaml             # Gazebo-ROS bridge configuration
├── config.rviz                # RViz visualization config
└── mapping.rviz               # RViz config for mapping
```

## 🎮 Joystick Configuration

### File: `joystick.yaml`

Controls joystick input processing and teleoperation behavior.

```yaml
joy_node:
  ros__parameters:
    device_id: 0              # Joystick device number (0 = first joystick)
    deadzone: 0.05            # Ignore inputs below this threshold
    autorepeat_rate: 20.0     # Publishing frequency (Hz)

teleop_node:
  ros__parameters:
    # Linear movement configuration
    axis_linear:
      x: 1                    # Left stick vertical (forward/backward)
    scale_linear:
      x: 0.1                  # Normal speed multiplier (m/s)
    scale_linear_turbo:
      x: 0.2                  # Turbo speed multiplier (m/s)

    # Angular movement configuration  
    axis_angular:
      yaw: 3                  # Right stick horizontal (rotation)
    scale_angular:
      yaw: 0.3                # Normal rotation speed (rad/s)
    scale_angular_turbo:
      yaw: 0.6                # Turbo rotation speed (rad/s)

    # Safety buttons
    enable_button: 4          # Must hold for any movement (L1/LB)
    enable_turbo_button: 5    # Hold for turbo mode (R1/RB)
    require_enable_button: true
```

### Joystick Button Mapping (Standard Gamepad)
| Button | Number | Function |
|--------|---------|----------|
| A/Cross | 0 | - |
| B/Circle | 1 | - |
| X/Square | 2 | - |
| Y/Triangle | 3 | - |
| L1/LB | 4 | **Enable Button** |
| R1/RB | 5 | **Turbo Button** |
| Select/Back | 6 | - |
| Start/Menu | 7 | - |

### Axis Mapping
| Axis | Number | Function |
|------|---------|----------|
| Left Stick X | 0 | - |
| Left Stick Y | 1 | **Forward/Backward** |
| Right Stick X | 3 | **Rotation** |
| Right Stick Y | 4 | - |

### Customization Examples

**Increase Speed:**
```yaml
scale_linear:
  x: 0.2                    # Increase from 0.1 to 0.2 m/s
scale_angular:
  yaw: 0.5                  # Increase from 0.3 to 0.5 rad/s
```

**Change Control Layout:**
```yaml
axis_linear:
  x: 0                      # Use left stick horizontal
axis_angular:
  yaw: 1                    # Use left stick vertical for rotation
```

**Disable Safety Button:**
```yaml
require_enable_button: false
```

## 🤖 Hardware Configuration

### File: `drivelink/node_config.yaml`

Core robot hardware parameters.

```yaml
serial_config:
  baud: 576000              # Serial communication speed
  port: drivelink           # Serial device name (/dev/drivelink)

diff_drive_config:
  wheel_radius: 0.06675     # Wheel radius in meters (66.75mm)
  wheel_base: 0.39          # Distance between wheels in meters (390mm)
```

### Hardware Calibration

**Measuring Wheel Radius:**
1. Mark a point on the wheel
2. Measure circumference after one full rotation
3. Calculate: radius = circumference / (2 * π)

**Measuring Wheel Base:**
1. Measure center-to-center distance between wheels
2. Alternative: Drive robot in a circle and measure

**Serial Port Setup:**
The robot expects the hardware to be available at `/dev/drivelink`. Set up udev rules:

```bash
# Create udev rule
sudo nano /etc/udev/rules.d/99-drivelink.rules

# Add content:
SUBSYSTEM=="tty", ATTRS{idVendor}=="YOUR_VENDOR_ID", ATTRS{idProduct}=="YOUR_PRODUCT_ID", SYMLINK+="drivelink"

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### File: `drivelink/controller_config.json`

Motor controller-specific parameters (content depends on hardware).

## 📡 LIDAR Configuration

### File: `lidar/rplidar_c1_config.yaml`

Configuration for RPLidar C1 sensor.

```yaml
rplidar_node:
  ros__parameters:
    channel_type: "serial"      # Communication type
    serial_port: "/dev/lidar"   # Device path
    serial_baudrate: 460800     # Communication speed
    frame_id: "laser_frame"     # TF frame name
    inverted: false             # Invert scan direction
    angle_compensate: true      # Enable angle compensation
    scan_mode: "Standard"       # Scan mode (Standard/Express)
```

### LIDAR Troubleshooting

**Setup Device Permissions:**
```bash
# Create udev rule for LIDAR
sudo nano /etc/udev/rules.d/99-rplidar.rules

# Add content:
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="666"

# Reload and apply
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Test LIDAR Connection:**
```bash
# Check device exists
ls -l /dev/lidar

# Test communication
sudo chmod 666 /dev/lidar
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/lidar
```

## 🚦 Command Velocity Multiplexing

### File: `twist_mux_config.yaml`

Manages priority between different velocity command sources.

```yaml
twist_mux:
  ros__parameters:
    use_stamped: true         # Use TwistStamped messages
    topics:
      navigation:             # Nav2 navigation commands
        topic: cmd_vel        # Topic name
        timeout: 0.5          # Timeout in seconds
        priority: 10          # Lower number = lower priority
      joystick:              # Manual joystick control
        topic: cmd_vel_joy    # Topic name  
        timeout: 0.5          # Timeout in seconds
        priority: 100         # Higher number = higher priority
```

### Priority System

The twist_mux node selects commands based on:
1. **Priority Level**: Higher numbers override lower numbers
2. **Timeout**: Commands expire if not refreshed
3. **Message Presence**: Must have recent messages to be active

**Example Priority Hierarchy:**
- Joystick: Priority 100 (highest - safety override)
- Navigation: Priority 10 (lower - autonomous operation)

### Adding New Command Sources

```yaml
topics:
  emergency_stop:
    topic: cmd_vel_emergency
    timeout: 0.1
    priority: 1000          # Highest priority
  
  my_custom_controller:
    topic: cmd_vel_custom
    timeout: 1.0
    priority: 50            # Medium priority
```

## 🗺️ Navigation Configuration

### File: `nav2_params.yaml`

Comprehensive Nav2 stack configuration (content varies based on setup).

Key sections typically include:
- **Controller Server**: Path following behavior
- **Planner Server**: Path planning algorithms  
- **Costmap Parameters**: Obstacle avoidance
- **AMCL**: Localization settings
- **BT Navigator**: Behavior tree configuration

### Common Parameter Adjustments

**Robot Footprint:**
```yaml
footprint: [[-0.2, -0.15], [-0.2, 0.15], [0.2, 0.15], [0.2, -0.15]]
```

**Maximum Speeds:**
```yaml
max_vel_x: 0.5
max_vel_theta: 1.0
```

**Planning Frequency:**
```yaml
planner_frequency: 1.0
controller_frequency: 10.0
```

## 🌉 Gazebo Bridge Configuration

### File: `gz_bridge.yaml`

Maps topics between ROS and Gazebo simulation.

Example structure:
```yaml
- topic: /scan
  ros_type: sensor_msgs/msg/LaserScan
  gz_type: gz.msgs.LaserScan

- topic: /cmd_vel
  ros_type: geometry_msgs/msg/Twist
  gz_type: gz.msgs.Twist

- topic: /odom
  ros_type: nav_msgs/msg/Odometry
  gz_type: gz.msgs.Odometry
```

## 🎯 SLAM Configuration

### SLAM Toolbox Parameters

Located in `slam_toolbox/` directory, typically includes:
- **Mapping parameters**: Resolution, update rates
- **Loop closure**: Detection and correction
- **Sensor parameters**: LIDAR-specific settings

## 🔧 Parameter Management

### Loading Parameters in Launch Files

```python
# Load from YAML file
parameters=[os.path.join(package_dir, 'config', 'my_config.yaml')]

# Override specific parameters
parameters=[
    os.path.join(package_dir, 'config', 'my_config.yaml'),
    {'param_name': 'param_value'}
]
```

### Runtime Parameter Changes

```bash
# List all parameters for a node
ros2 param list /node_name

# Get current value
ros2 param get /node_name parameter_name

# Set new value
ros2 param set /node_name parameter_name new_value

# Save current parameters
ros2 param dump /node_name > my_params.yaml
```

## 📊 Configuration Validation

### Pre-launch Checks

Create a validation script:
```bash
#!/bin/bash
# validate_config.sh

echo "Checking configuration files..."

# Check file existence
config_files=(
    "config/joystick.yaml"
    "config/twist_mux_config.yaml" 
    "config/drivelink/node_config.yaml"
    "config/lidar/rplidar_c1_config.yaml"
)

for file in "${config_files[@]}"; do
    if [ ! -f "$file" ]; then
        echo "ERROR: Missing config file: $file"
        exit 1
    fi
done

# Check YAML syntax
for file in config/*.yaml config/*/*.yaml; do
    python3 -c "import yaml; yaml.safe_load(open('$file'))" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "ERROR: Invalid YAML syntax in $file"
        exit 1
    fi
done

echo "All configuration files validated successfully!"
```

### Common Configuration Errors

1. **YAML Syntax Errors**: Indentation, missing colons
2. **Wrong Parameter Names**: Typos in parameter keys
3. **Invalid Values**: Out-of-range numbers, wrong types
4. **Missing Files**: Referenced files don't exist
5. **Permission Issues**: Cannot read configuration files

### Debug Configuration Loading

```bash
# Check if parameters are loaded correctly
ros2 param list /node_name

# Verify parameter values
ros2 param get /node_name param_name

# Monitor parameter changes
ros2 param get /node_name param_name --spin

# Launch with parameter debugging
ros2 launch --debug riggu_bringup main.launch.py
```
