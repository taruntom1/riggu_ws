# Simulation Guide

This guide covers using the Riggu robot in Gazebo simulation environment for development, testing, and demonstration purposes.

## 🎯 Overview

The Riggu simulation provides a complete virtual environment for:
- **Algorithm Development**: Test navigation and control algorithms safely
- **System Integration**: Validate complete system behavior
- **Training**: Learn robot operation without hardware risks
- **Demonstration**: Show capabilities without physical robot

## 🚀 Quick Start

### Launch Simulation
```bash
# Basic simulation with navigation
ros2 launch riggu_bringup main.launch.py use_sim:=true

# Headless simulation (no GUI)
ros2 launch riggu_bringup main.launch.py use_sim:=true headless:=true

# Simulation without navigation (for basic testing)
ros2 launch riggu_bringup main.launch.py use_sim:=true launch_nav2:=false
```

### Control the Robot
```bash
# Using joystick (if connected)
# Just use the gamepad as with real hardware

# Using keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Using RViz navigation
# Set 2D Nav Goal in RViz interface
```

## 🌍 Simulation Environment

### Default World
The simulation uses `worlds/something.sdf` as the default environment. This world includes:
- Indoor environment layout
- Static obstacles
- Textured surfaces for realistic LIDAR simulation
- Appropriate lighting conditions

### Available Worlds
```
worlds/
├── empty.world          # Minimal empty environment
└── something.sdf        # Default environment with obstacles
```

### Custom World Creation
Create custom SDF worlds for specific testing scenarios:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="custom_world">
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Add obstacles, walls, etc. -->
    
  </world>
</sdf>
```

## 🤖 Robot Model in Simulation

### Gazebo Integration
The robot model is loaded from the `riggu_description` package with:
- **Physical Properties**: Mass, inertia, friction
- **Sensor Models**: LIDAR simulation
- **Actuator Models**: Differential drive plugin
- **Visual Models**: 3D appearance

### Sensor Simulation

#### LIDAR Simulation
The RPLidar C1 is simulated with realistic properties:
- **Range**: 0.15-12m (matching real sensor)
- **Resolution**: Configurable angular resolution
- **Noise**: Realistic measurement noise
- **Ray Tracing**: Accurate intersection calculation

#### Odometry Simulation
Wheel odometry is simulated including:
- **Wheel Slip**: Configurable slip parameters
- **Noise**: Position and velocity noise
- **Drift**: Accumulated error over time

### Physics Configuration
Key physics parameters for realistic simulation:
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## 🔌 ROS-Gazebo Bridge

### Bridge Configuration
The `config/gz_bridge.yaml` file defines topic mappings:

```yaml
# Essential topics for robot operation
- topic: /cmd_vel
  ros_type: geometry_msgs/msg/Twist
  gz_type: gz.msgs.Twist
  
- topic: /odom
  ros_type: nav_msgs/msg/Odometry
  gz_type: gz.msgs.Odometry
  
- topic: /scan
  ros_type: sensor_msgs/msg/LaserScan
  gz_type: gz.msgs.LaserScan

# Transform information
- topic: /tf
  ros_type: tf2_msgs/msg/TFMessage
  gz_type: gz.msgs.Pose_V

# Clock synchronization
- topic: /clock
  ros_type: rosgraph_msgs/msg/Clock
  gz_type: gz.msgs.Clock
```

### Adding New Sensors
To add additional sensors to the simulation:

1. **Update Robot URDF** (in riggu_description):
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

2. **Update Bridge Configuration**:
```yaml
- topic: /camera/image_raw
  ros_type: sensor_msgs/msg/Image
  gz_type: gz.msgs.Image
```

## 🎮 Simulation Control

### Joystick Control
Joystick control works identically to real hardware:
- Connect USB gamepad
- Launch simulation with joystick support
- Use safety button + movement controls

### Programmatic Control
Send velocity commands directly:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{}'
```

### Python Control Example
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def control_loop(self):
        msg = Twist()
        # Add your control logic here
        msg.linear.x = 0.1  # Move forward slowly
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = SimulationController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🗺️ Navigation in Simulation

### SLAM in Simulation
Build maps using simulated LIDAR data:

```bash
# Launch simulation with SLAM
ros2 launch riggu_bringup main.launch.py use_sim:=true

# Drive robot around to build map
# Use joystick or keyboard teleop

# Save map when complete
ros2 run nav2_map_server map_saver_cli -f my_simulation_map
```

### Navigation Testing
Test autonomous navigation:

```bash
# Launch with pre-built map
ros2 launch riggu_bringup main.launch.py use_sim:=true

# Use RViz to set navigation goals
# Monitor navigation performance
# Test different scenarios
```

### Scenario Testing
Create specific test scenarios:

1. **Obstacle Avoidance**:
   - Add dynamic obstacles
   - Test recovery behaviors
   - Validate safety features

2. **Path Planning**:
   - Complex environments
   - Multiple goal sequences
   - Different map types

3. **Localization**:
   - Symmetric environments
   - Poor LIDAR conditions
   - Recovery from lost localization

## 📊 Simulation Monitoring

### Performance Metrics
Monitor simulation performance:

```bash
# Check simulation real-time factor
gz stats

# Monitor topic frequencies
ros2 topic hz /scan /odom /cmd_vel

# Check computational load
htop
```

### Data Recording
Record simulation data for analysis:

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /odom /cmd_vel /tf

# Play back recorded data
ros2 bag play <bag_file>
```

## 🔧 Simulation Configuration

### Performance Tuning

**For Real-time Performance**:
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

**For Faster-than-real-time**:
```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size>
  <real_time_factor>0</real_time_factor>  <!-- Uncapped -->
  <real_time_update_rate>100</real_time_update_rate>
</physics>
```

### Graphics Settings
For headless operation or reduced graphics:
```bash
# Headless (no GUI)
ros2 launch riggu_bringup main.launch.py use_sim:=true headless:=true

# Reduced graphics in GUI mode
export GAZEBO_GUI_PARAMS="--gui-params=minimal"
```

## 🐛 Simulation Troubleshooting

### Common Issues

**Simulation Runs Slowly**:
- Reduce physics update rate
- Simplify world model
- Use headless mode
- Close unnecessary applications

**Robot Doesn't Move**:
- Check cmd_vel topic is being published
- Verify bridge configuration
- Confirm robot model has differential drive plugin
- Check for collision issues

**LIDAR Data Issues**:
- Verify laser sensor in robot model
- Check bridge topic mappings
- Confirm scan topic is publishing
- Test with simpler environment

**Navigation Problems**:
- Verify transforms are published
- Check map quality
- Ensure LIDAR data is clean
- Validate navigation parameters

### Debug Commands

```bash
# List Gazebo topics
gz topic -l

# Check Gazebo model info
gz model -m riggu_robot -i

# Monitor bridge status
ros2 topic list | grep -E "(scan|odom|cmd_vel)"

# Check transform tree
ros2 run tf2_tools view_frames

# Verify simulation time
ros2 topic echo /clock
```

## 📈 Simulation vs Reality

### Key Differences
- **Perfect Sensors**: No real-world noise/drift
- **Ideal Physics**: Simplified collision/friction models
- **Consistent Environment**: No lighting/weather changes
- **Computational Limits**: May not run real-time

### Validation Strategy
1. **Develop in Simulation**: Rapid prototyping
2. **Test Core Algorithms**: Validate basic functionality
3. **Real-world Testing**: Verify performance on hardware
4. **Iterative Refinement**: Update based on real results

### Simulation Fidelity
Improve realism by:
- Adding sensor noise models
- Including environmental dynamics
- Modeling hardware limitations
- Using realistic physics parameters

## 🔄 Simulation Workflows

### Development Workflow
```bash
# 1. Start simulation
ros2 launch riggu_bringup main.launch.py use_sim:=true

# 2. Develop/test algorithm
# Edit code, test in simulation

# 3. Record test data
ros2 bag record -a

# 4. Analyze results
# Play back data, analyze performance

# 5. Deploy to hardware
# Transfer working code to real robot
```

### Testing Workflow
```bash
# 1. Create test scenario
# Design specific world/obstacles

# 2. Run automated tests
# Scripted navigation sequences

# 3. Collect metrics
# Success rates, path efficiency

# 4. Generate reports
# Performance analysis
```

