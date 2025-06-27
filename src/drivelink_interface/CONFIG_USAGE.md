# Configuration Management

The drivelink_interface package now supports ROS2-style configuration management with configurable file paths.

## Configuration Files

The package uses two main configuration files:

1. **Node Configuration (YAML)**: Contains serial and differential drive configuration
   - Default: `config/node_config.yaml`
   - Parameter name: `node_config_path`

2. **Controller Configuration (JSON)**: Contains controller properties and wheel data
   - Default: `config/controller_config.json`
   - Parameter name: `controller_config_path`

## Usage

### Default Configuration
Launch with default configuration files:
```bash
ros2 launch drivelink_interface drivelink_interface_node.launch.py
```

### Custom Configuration Paths
Launch with custom configuration file paths:
```bash
ros2 launch drivelink_interface drivelink_interface_node.launch.py \
  node_config_path:=/path/to/your/custom_node_config.yaml \
  controller_config_path:=/path/to/your/custom_controller_config.json
```

### Using Parameters Directly
Set parameters directly when running the node:
```bash
ros2 run drivelink_interface drivelink_interface_node \
  --ros-args \
  -p node_config_path:=/path/to/your/custom_node_config.yaml \
  -p controller_config_path:=/path/to/your/custom_controller_config.json
```

### Example with Custom Launch File
Use the provided example launch file for custom configurations:
```bash
ros2 launch drivelink_interface drivelink_interface_custom_config.launch.py \
  node_config_path:=/home/user/my_node_config.yaml \
  controller_config_path:=/home/user/my_controller_config.json
```

## Parameter Files
You can also create parameter files (YAML format) to specify configuration paths:

```yaml
# parameters.yaml
drivelink_interface_node:
  ros__parameters:
    node_config_path: "/path/to/custom_node_config.yaml"
    controller_config_path: "/path/to/custom_controller_config.json"
```

Then launch with:
```bash
ros2 launch drivelink_interface drivelink_interface_node.launch.py \
  --ros-args --params-file parameters.yaml
```

## Configuration File Formats

### Node Configuration (YAML)
```yaml
serial_config:
  port: "/dev/ttyUSB0"
  baud: 115200

diff_drive_config:
  wheel_radius: 0.05
  wheel_base: 0.3
```

### Controller Configuration (JSON)
The JSON configuration contains controller properties and wheel data structures as defined by the `StructSerialiser` class.

## Benefits of ROS2-Style Configuration

1. **Parameter Integration**: Configuration paths are now ROS2 parameters that can be set via launch files, command line, or parameter files
2. **Runtime Flexibility**: Easy to switch between different configuration sets without code changes
3. **Launch File Support**: Full integration with ROS2 launch system
4. **Debugging**: Configuration paths are logged using ROS2 logging system
5. **Fallback Support**: Graceful fallback to default paths when parameters are not set
