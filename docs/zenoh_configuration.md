# Zenoh Middleware Configuration

This guide covers the setup and configuration of Zenoh middleware for distributed communication in the Riggu robot system.

## 🌐 Overview

Zenoh is a next-generation middleware that provides efficient, scalable communication for robotics applications. The Riggu robot uses `rmw_zenoh_cpp` as the ROS2 middleware implementation for:

- **Distributed Communication**: Connect robot and control stations over network
- **Efficient Data Transfer**: Optimized for robotics data patterns
- **Scalable Architecture**: Support multiple robots and control stations
- **Network Resilience**: Automatic reconnection and discovery

## 🔧 Installation

### Install Zenoh Middleware
```bash
# Install rmw_zenoh_cpp
sudo apt update
sudo apt install ros-kilted-rmw-zenoh-cpp

# Set Zenoh as default middleware
echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation
```bash
# Check if Zenoh is available
ros2 doctor --report

# List available RMW implementations
ros2 run rmw_implementation_cmake check_rmw_implementation.py
```

## 🚀 Quick Setup

### Scenario 1: Robot with Auto-Start (Recommended)
The Zenoh router automatically starts on the robot when booted.

**On Development Machine**:
```bash
# Connect to robot's Zenoh router (replace IP)
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/ROBOT_IP:7447"]'

# Launch your ROS2 nodes
ros2 launch riggu_bringup main.launch.py
```

### Scenario 2: Manual Router Setup

**On Robot**:
```bash
# Start Zenoh router
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**On Control Station**:
```bash
# Connect to robot's router
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/ROBOT_IP:7447"]'

# Or start local router and connect
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## ⚙️ Configuration Details

### Environment Variables

#### RMW_IMPLEMENTATION
```bash
# Set Zenoh as ROS2 middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

#### ZENOH_CONFIG_OVERRIDE
```bash
# Connect to specific router
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.1.100:7447"]'

# Multiple endpoints
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.1.100:7447","tcp/192.168.1.101:7447"]'

# Custom port
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.1.100:8080"]'
```

#### ZENOH_LOG_LEVEL
```bash
# Set logging level (error, warn, info, debug, trace)
export ZENOH_LOG_LEVEL=info
```

### Configuration File
Create `/tmp/zenoh.json5` for advanced configuration:

```json5
{
  // Network configuration
  listen: {
    endpoints: ["tcp/0.0.0.0:7447"]
  },
  
  // Connection configuration
  connect: {
    endpoints: ["tcp/192.168.1.100:7447"]
  },
  
  // Scouting configuration
  scouting: {
    multicast: {
      enabled: true,
      interfaces: ["wlan0", "eth0"]
    }
  },
  
  // Transport configuration
  transport: {
    unicast: {
      qos: {
        reliability: "reliable"
      }
    }
  }
}
```

Use configuration file:
```bash
export ZENOH_CONFIG_FILE=/tmp/zenoh.json5
```

## 🔗 Network Setup

### Network Discovery
Zenoh supports automatic peer discovery:

```bash
# Enable multicast discovery
export ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled:true'

# Disable for manual connections only
export ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled:false'
```

### Firewall Configuration
Ensure required ports are open:

```bash
# Default Zenoh port
sudo ufw allow 7447/tcp

# Custom port (if configured)
sudo ufw allow 8080/tcp

# Multicast discovery (UDP)
sudo ufw allow 7446/udp
```

### Router Configuration

#### Basic Router
```bash
# Start router on default port
ros2 run rmw_zenoh_cpp rmw_zenohd

# Start router on custom port
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:8080"]' ros2 run rmw_zenoh_cpp rmw_zenohd
```

#### Bridge Router
Connect multiple network segments:
```bash
# Router bridging two networks
export ZENOH_CONFIG_OVERRIDE='{
  listen: {endpoints: ["tcp/0.0.0.0:7447"]},
  connect: {endpoints: ["tcp/192.168.2.100:7447"]}
}'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## 🛠️ Development Container Setup

### Auto-Start Script
The `start_zenoh.sh` script automatically starts a Zenoh router in a tmux session for development containers. It provides robust session management and error handling.

#### Script Features
- **Tmux Session Management**: Runs Zenoh router in a persistent background session
- **Environment Variable Configuration**: Configurable endpoint, session name, and ROS setup
- **Error Handling**: Comprehensive validation and logging
- **Interactive Session Management**: Handles existing sessions gracefully

#### Configuration Variables
```bash
# Environment variables (with defaults)
ZENOH_SESSION_NAME="${ZENOH_SESSION_NAME:-zenoh_router}"
ZENOH_ENDPOINT="${ZENOH_ENDPOINT:-tcp/100.107.192.97:7447}"
ROS_SETUP_SCRIPT="${ROS_SETUP_SCRIPT:-/opt/ros/kilted/setup.bash}"
```

#### Usage
```bash
# Basic usage - start with default configuration
./start_zenoh.sh

# Custom endpoint
ZENOH_ENDPOINT="tcp/192.168.1.100:7447" ./start_zenoh.sh

# Custom session name
ZENOH_SESSION_NAME="my_zenoh_session" ./start_zenoh.sh

# Custom ROS setup script
ROS_SETUP_SCRIPT="/opt/ros/humble/setup.bash" ./start_zenoh.sh
```

#### Session Management
```bash
# Attach to the running Zenoh session
tmux attach-session -t zenoh_router

# List all tmux sessions
tmux list-sessions

# Kill the Zenoh session
tmux kill-session -t zenoh_router

# Check if session is running
tmux has-session -t zenoh_router && echo "Running" || echo "Not running"
```

### Docker Integration
Add to Dockerfile or docker-compose.yml:

```dockerfile
# Dockerfile
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
COPY .devcontainer/start_zenoh.sh /opt/ros/
RUN chmod +x /opt/ros/start_zenoh.sh
```

```yaml
# docker-compose.yml
services:
  riggu_dev:
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_ENDPOINT=tcp/100.107.192.97:7447
      - ZENOH_SESSION_NAME=zenoh_router
    volumes:
      - ./.devcontainer/start_zenoh.sh:/opt/ros/start_zenoh.sh:ro
```

### Troubleshooting the Start Script

#### Common Issues

**Script fails with "tmux not found"**:
```bash
# Install tmux
sudo apt update && sudo apt install tmux
```

**Script fails with "ROS setup script not found"**:
```bash
# Check ROS installation
ls -la /opt/ros/*/setup.bash

# Set custom ROS setup script
export ROS_SETUP_SCRIPT="/opt/ros/humble/setup.bash"
```

**Session already exists**:
- Script will prompt to kill existing session
- Or manually kill: `tmux kill-session -t zenoh_router`

**Cannot connect to endpoint**:
```bash
# Test network connectivity
ping 100.107.192.97

# Check if port is accessible
telnet 100.107.192.97 7447
```

## 🔍 Debugging and Monitoring

### Check Zenoh Status
```bash
# List Zenoh peers
export ZENOH_LOG_LEVEL=info
ros2 run rmw_zenoh_cpp rmw_zenohd

# Check connection status
ros2 topic list  # Should show topics from all connected peers

# Monitor data flow
ros2 topic hz /scan /odom /cmd_vel
```

### Network Diagnostics
```bash
# Test network connectivity
ping ROBOT_IP

# Check port accessibility
telnet ROBOT_IP 7447
nc -zv ROBOT_IP 7447

# Monitor network traffic
sudo tcpdump -i any port 7447
```

### Zenoh-Specific Tools
```bash
# Install zenoh tools (if available)
cargo install zenoh --features="default"

# List zenoh resources
zenoh ls

# Monitor zenoh traffic
zenoh monitor
```

## 🚨 Troubleshooting

### Common Issues

#### Cannot Connect to Router
**Symptoms**: ROS2 nodes don't see each other
**Solutions**:
```bash
# Check IP address is correct
ping ROBOT_IP

# Verify port is open
telnet ROBOT_IP 7447

# Check firewall rules
sudo ufw status

# Try direct connection
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/ROBOT_IP:7447"]'
```

#### Slow Communication
**Symptoms**: High latency or packet loss
**Solutions**:
```bash
# Check network quality
ping -c 10 ROBOT_IP

# Monitor bandwidth
iftop -i wlan0

# Adjust QoS settings
export ZENOH_CONFIG_OVERRIDE='transport/unicast/qos/reliability:"best_effort"'
```

#### Discovery Issues
**Symptoms**: Nodes intermittently disappear
**Solutions**:
```bash
# Enable multicast discovery
export ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled:true'

# Increase discovery timeout
export ZENOH_CONFIG_OVERRIDE='scouting/timeout:5000'

# Use explicit connections instead
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/ROBOT_IP:7447"]'
```

### Debug Commands
```bash
# Check if Zenoh session is running
tmux has-session -t zenoh_router && echo "Session active" || echo "Session not found"

# View Zenoh router logs
tmux capture-pane -t zenoh_router -p

# Attach to Zenoh session for live monitoring
tmux attach-session -t zenoh_router

# Verbose Zenoh logging
export ZENOH_LOG_LEVEL=debug

# Check RMW implementation
echo $RMW_IMPLEMENTATION

# List available topics across network
ros2 topic list

# Check node discovery
ros2 node list

# Monitor specific topics
ros2 topic echo /scan --once

# Test Zenoh connectivity
ZENOH_ENDPOINT="tcp/100.107.192.97:7447" ./start_zenoh.sh
```

## 🔧 Advanced Configuration

### Multi-Robot Setup
Configure multiple robots with unique identifiers:

```bash
# Robot 1
export ZENOH_CONFIG_OVERRIDE='mode:"peer",id:"robot1"'

# Robot 2  
export ZENOH_CONFIG_OVERRIDE='mode:"peer",id:"robot2"'

# Control station
export ZENOH_CONFIG_OVERRIDE='mode:"client"'
```

### Performance Tuning
```bash
# High-performance settings
export ZENOH_CONFIG_OVERRIDE='{
  transport: {
    unicast: {
      qos: {reliability: "reliable"},
      lowlatency: true
    }
  }
}'

# Low-bandwidth settings
export ZENOH_CONFIG_OVERRIDE='{
  transport: {
    unicast: {
      qos: {reliability: "best_effort"},
      compression: true
    }
  }
}'
```

### Security Configuration
```bash
# Enable authentication (if supported)
export ZENOH_CONFIG_OVERRIDE='{
  transport: {
    auth: {
      usrpwd: {
        user: "riggu",
        password: "secure_password"
      }
    }
  }
}'
```

## 📋 Best Practices

### Network Planning
1. **Use Static IPs**: Assign fixed IP addresses to robots
2. **Plan Port Usage**: Avoid conflicts with other services
3. **Network Segmentation**: Isolate robot network if needed
4. **Backup Connections**: Configure multiple network interfaces

### Performance Optimization
1. **Minimize Hops**: Direct connections when possible
2. **QoS Settings**: Match reliability to data criticality
3. **Bandwidth Management**: Monitor network usage
4. **Connection Pooling**: Reuse connections efficiently

### Debugging Strategy
1. **Start Simple**: Test basic connectivity first
2. **Layer by Layer**: Verify network, then Zenoh, then ROS2
3. **Monitor Continuously**: Use logging and monitoring tools
4. **Document Configuration**: Keep track of working setups

## 🔧 Start Zenoh Script Reference

### Script Location
The Zenoh startup script is located at:
```
/riggu_ws/.devcontainer/start_zenoh.sh
```

### Script Architecture
The script implements the following features:

1. **Robust Error Handling**: Uses `set -euo pipefail` for strict error handling
2. **Environment Configuration**: Configurable via environment variables
3. **Dependency Validation**: Checks for tmux and ROS setup script
4. **Session Management**: Handles existing tmux sessions gracefully
5. **Colored Logging**: Provides clear status messages with color coding
6. **Background Execution**: Runs Zenoh router in detached tmux session

### Internal Configuration
```bash
# Default values used by the script
ZENOH_SESSION_NAME="${ZENOH_SESSION_NAME:-zenoh_router}"
ZENOH_ENDPOINT="${ZENOH_ENDPOINT:-tcp/100.107.192.97:7447}"
ROS_SETUP_SCRIPT="${ROS_SETUP_SCRIPT:-/opt/ros/kilted/setup.bash}"
```

### Logging Functions
The script provides three logging levels:
- `log_info()`: Green colored informational messages
- `log_warn()`: Yellow colored warning messages  
- `log_error()`: Red colored error messages (to stderr)

### Error Conditions
The script will exit with error code 1 if:
- tmux is not installed or not in PATH
- ROS setup script doesn't exist
- Failed to create tmux session
- User chooses not to replace existing session

### Session Verification
After starting, the script verifies the session was created and provides:
- Success confirmation
- Instructions to attach to session
- Instructions to kill session
