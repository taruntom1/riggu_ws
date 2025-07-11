# Navigation Guide

This comprehensive guide covers the navigation capabilities of the Riggu robot, including SLAM (Simultaneous Localization and Mapping), autonomous navigation using Nav2, and advanced navigation features.

## 🎯 Overview

The Riggu robot navigation system provides:
- **SLAM**: Real-time mapping and localization
- **Path Planning**: Global and local path planning algorithms
- **Obstacle Avoidance**: Dynamic obstacle detection and avoidance
- **Autonomous Navigation**: Goal-based autonomous movement
- **Recovery Behaviors**: Automatic recovery from navigation failures

## 🗺️ SLAM (Simultaneous Localization and Mapping)

### SLAM Overview
The system uses SLAM Toolbox for creating maps while simultaneously tracking the robot's position.

**Key Features**:
- Real-time mapping using LIDAR data
- Loop closure detection for map consistency
- Online and offline mapping modes
- Map saving and loading capabilities

### Starting SLAM

**With Full System**:
```bash
# Launch complete system with SLAM
ros2 launch riggu_bringup main.launch.py

# SLAM is included by default
```

**SLAM Only**:
```bash
# Launch just SLAM components
ros2 launch riggu_bringup nav2_slam.launch.py launch_nav2:=false
```

### Building a Map

1. **Start the System**:
```bash
ros2 launch riggu_bringup main.launch.py
```

2. **Open RViz for Visualization**:
```bash
ros2 run rviz2 rviz2 -d src/riggu_bringup/config/mapping.rviz
```

3. **Drive the Robot**:
   - Use joystick control to move around
   - Drive slowly for better mapping quality
   - Ensure good LIDAR visibility of walls/obstacles
   - Cover all areas you want mapped

4. **Monitor Map Quality**:
   - Watch map building in RViz
   - Ensure loop closures are detected
   - Check for map consistency

5. **Save the Map**:
```bash
# Save current map
ros2 run nav2_map_server map_saver_cli -f my_map

# This creates:
# - my_map.pgm (image file)
# - my_map.yaml (metadata)
```

### SLAM Configuration

Key SLAM parameters in `config/slam_toolbox/`:

```yaml
# Resolution
resolution: 0.05                    # Map resolution (m/pixel)

# Scan processing
minimum_travel_distance: 0.5        # Min distance before processing
minimum_travel_heading: 0.5         # Min rotation before processing

# Loop closure
loop_search_maximum_distance: 3.0   # Max distance for loop search
loop_match_minimum_chain_size: 10   # Min chain size for loop closure

# Map saving
map_file_name: /tmp/slam_map        # Auto-save location
map_start_at_dock: true             # Start map at robot position
```

## 🧭 Autonomous Navigation (Nav2)

### Navigation Stack Overview
The Nav2 stack provides complete autonomous navigation capabilities:

- **Global Planner**: Computes optimal path to goal
- **Local Planner**: Follows path while avoiding obstacles  
- **Costmaps**: Represent obstacle information
- **Recovery Behaviors**: Handle navigation failures
- **Behavior Trees**: Coordinate navigation actions

### Navigation Modes

#### 1. Navigation with Known Map
```bash
# Launch with pre-saved map
ros2 launch riggu_bringup main.launch.py map:=/path/to/map.yaml

# Set navigation goals using RViz 2D Nav Goal tool
```

#### 2. Navigation with SLAM
```bash
# Launch with simultaneous mapping
ros2 launch riggu_bringup main.launch.py

# Build map while navigating
```

### Setting Navigation Goals

#### Using RViz
1. **Open RViz**:
```bash
ros2 run rviz2 rviz2 -d src/riggu_bringup/config/config.rviz
```

2. **Set Goals**:
   - Click "2D Nav Goal" button
   - Click and drag on map to set goal position and orientation
   - Robot will automatically plan and execute path

#### Using Command Line
```bash
# Send navigation goal programmatically
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

#### Using Python
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1] 
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    client = NavigationClient()
    
    # Send goal to position (2.0, 1.0) with 90 degree rotation
    future = client.send_goal(2.0, 1.0, 1.57)
    rclpy.spin_until_future_complete(client, future)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ⚙️ Navigation Configuration

### Key Configuration Files

The main navigation configuration is in `config/nav2_params.yaml`:

#### Global Costmap
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      track_unknown_space: true
      
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

#### Local Costmap
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
```

#### Controller Configuration
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # DWB parameters
      max_vel_x: 0.3
      min_vel_x: -0.3
      max_vel_y: 0.0
      min_vel_y: 0.0
      max_vel_theta: 1.0
      min_vel_theta: -1.0
```

### Robot-Specific Parameters

#### Robot Footprint
Define the robot's physical outline:
```yaml
footprint: [[-0.2, -0.15], [-0.2, 0.15], [0.2, 0.15], [0.2, -0.15]]
# Or use radius for circular robot:
# robot_radius: 0.25
```

#### Speed Limits
```yaml
max_vel_x: 0.3          # Maximum forward speed (m/s)
max_vel_theta: 1.0      # Maximum rotation speed (rad/s)
acc_lim_x: 2.5          # Linear acceleration limit
acc_lim_theta: 3.2      # Angular acceleration limit
```

## 🛡️ Safety Features

### Obstacle Avoidance
- **Dynamic Obstacles**: Real-time detection and avoidance
- **Static Map Obstacles**: Pre-mapped permanent obstacles
- **Inflation Layers**: Safety margins around obstacles
- **Emergency Stops**: Immediate stopping for critical situations

### Recovery Behaviors
Automatic recovery when navigation fails:

1. **Clear Costmap**: Reset obstacle information
2. **Spin Recovery**: Rotate to clear unknown space
3. **Back Up**: Move backward to escape tight spaces
4. **Wait**: Pause for dynamic obstacles to clear

### Safety Configuration
```yaml
# Emergency stop distances
emergency_stop_distance: 0.1    # Stop if obstacle within 10cm

# Recovery behavior timeouts
spin_timeout: 10.0              # Max time for spin recovery
backup_timeout: 5.0             # Max time for backup recovery

# Costmap clearing
clear_costmap_timeout: 2.0      # How often to clear costmaps
```

## 📊 Navigation Monitoring

### Key Topics
Monitor navigation performance:

```bash
# Robot position
ros2 topic echo /amcl_pose

# Global path plan
ros2 topic echo /plan

# Local path plan  
ros2 topic echo /local_plan

# Costmap data
ros2 topic echo /global_costmap/costmap
ros2 topic echo /local_costmap/costmap

# Navigation status
ros2 action client /navigate_to_pose nav2_msgs/action/NavigateToPose
```

### Performance Metrics
```bash
# Path planning frequency
ros2 topic hz /plan

# Control output frequency
ros2 topic hz /cmd_vel

# Localization update rate
ros2 topic hz /amcl_pose

# Scan processing rate
ros2 topic hz /scan
```

## 🔧 Advanced Navigation Features

### Waypoint Navigation
Navigate through multiple points:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class WaypointNavigator:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        
    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
        
    def navigate_waypoints(self, waypoints):
        # Convert waypoints to poses
        poses = []
        for x, y, yaw in waypoints:
            poses.append(self.create_pose(x, y, yaw))
            
        # Execute waypoint navigation
        self.navigator.followWaypoints(poses)
        
        # Wait for completion
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.1)

def main():
    waypoint_nav = WaypointNavigator()
    
    # Define waypoints: (x, y, yaw)
    waypoints = [
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 1.57),
        (0.0, 1.0, 3.14),
        (0.0, 0.0, -1.57)
    ]
    
    waypoint_nav.navigate_waypoints(waypoints)

if __name__ == '__main__':
    main()
```

### Navigation with Constraints
```yaml
# Path constraints
use_path_constraints: true
path_constraints:
  max_deviation: 0.5        # Max deviation from planned path
  lookahead_distance: 1.0   # How far ahead to look

# Speed constraints in different areas
speed_constraint_zones:
  - name: "slow_zone"
    polygon: [[0, 0], [1, 0], [1, 1], [0, 1]]
    max_linear_vel: 0.1
    max_angular_vel: 0.2
```

## 🎛️ Navigation Tuning

### Path Planning Tuning
```yaml
# Global planner (A*)
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    use_astar: true
    allow_unknown: true
    
    NavfnPlanner:
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### Local Planner Tuning
```yaml
# DWB local planner
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  
  # Trajectory generation
  vx_samples: 20
  vy_samples: 5
  vtheta_samples: 20
  
  # Trajectory evaluation
  path_distance_bias: 64.0      # Stay close to global path
  goal_distance_bias: 24.0      # Move toward goal
  occdist_scale: 0.02          # Avoid obstacles
  
  # Forward simulation
  sim_time: 1.7                # Simulation horizon
  discretize_by_time: false    # Discretize by distance
```

### Costmap Tuning
```yaml
# Inflation parameters
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0    # How quickly cost increases
  inflation_radius: 0.55      # Distance to inflate around obstacles

# Obstacle layer
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  max_obstacle_height: 2.0
  combination_method: 1       # Maximum of all layers
  
  scan:
    topic: /scan
    obstacle_max_range: 2.5   # Max range to consider obstacles
    obstacle_min_range: 0.0   # Min range to consider obstacles
    raytrace_max_range: 3.0   # Max range for raytracing
    raytrace_min_range: 0.0   # Min range for raytracing
```

## 🐛 Navigation Troubleshooting

### Common Issues

#### Robot Won't Plan Path
**Symptoms**: No global path appears in RViz
**Solutions**:
```bash
# Check if goal is reachable
ros2 topic echo /global_costmap/costmap

# Verify map is loaded
ros2 topic echo /map

# Check planner status
ros2 node info /planner_server
```

#### Robot Gets Stuck
**Symptoms**: Robot oscillates or stops moving
**Solutions**:
- Increase inflation radius
- Adjust local costmap size
- Tune DWB parameters
- Check for sensor issues

#### Poor Localization
**Symptoms**: Robot position jumps or drifts
**Solutions**:
```bash
# Check AMCL parameters
ros2 param list /amcl

# Increase particle count
ros2 param set /amcl max_particles 5000

# Verify scan quality
ros2 topic echo /scan
```

### Debug Tools

#### Navigation Visualization
```bash
# Launch RViz with navigation config
ros2 run rviz2 rviz2 -d src/riggu_bringup/config/config.rviz

# Key displays to enable:
# - Map
# - Laser Scan
# - Local/Global Costmaps
# - Global/Local Plans
# - Robot Footprint
```

#### Navigation Debugging Commands
```bash
# Check navigation status
ros2 action client /navigate_to_pose nav2_msgs/action/NavigateToPose

# Clear costmaps manually
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntirelyGlobalCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntirelyLocalCostmap

# Get robot pose
ros2 topic echo /amcl_pose --once

# Check transform tree
ros2 run tf2_tools view_frames
```

## 📈 Performance Optimization

### Computational Optimization
```yaml
# Reduce update frequencies for better performance
global_costmap:
  update_frequency: 1.0     # Reduce from default 5.0
  
local_costmap:
  update_frequency: 5.0     # Reduce from default 10.0
  
controller_server:
  controller_frequency: 20.0 # Reduce from default 50.0
```

### Memory Optimization
```yaml
# Reduce costmap sizes
local_costmap:
  width: 3                  # Reduce from larger values
  height: 3
  
# Reduce trajectory samples
FollowPath:
  vx_samples: 15           # Reduce from 20
  vtheta_samples: 15       # Reduce from 20
```