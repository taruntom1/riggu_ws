# riggu_description

URDF/xacro model of riggu. *Planned — Phase 2.*

| File | Purpose |
|---|---|
| `urdf/riggu.urdf.xacro` | Top level. Arg: `mode:=real\|sim\|mock` |
| `urdf/properties.xacro` | All measured dimensions, masses, sensor poses (single source of truth) |
| `urdf/ros2_control.xacro` | `real`: DriveLink macro · `sim`: `gz_ros2_control/GazeboSimSystem` · `mock`: `mock_components/GenericSystem` |
| `urdf/sensors.xacro` | Lidar and IMU links, plus gz sensor definitions for `sim` |
| `launch/display.launch.py` | robot_state_publisher, joint_state_publisher_gui, RViz |

**Frames:** `base_footprint → base_link → {left_wheel, right_wheel, caster*, laser, imu_link}`

**ros2_control:**

| Item | Name | Interfaces |
|---|---|---|
| Wheel joint | `left_wheel_joint` | command: `velocity` · state: `position`, `velocity` |
| Wheel joint | `right_wheel_joint` | command: `velocity` · state: `position`, `velocity` |
| Sensor | `imu` (on `imu_link`) | state: `orientation.*`, `angular_velocity.*`, `linear_acceleration.*` |

```bash
xacro urdf/riggu.urdf.xacro mode:=mock > /tmp/riggu.urdf && check_urdf /tmp/riggu.urdf
ros2 launch riggu_description display.launch.py
```
