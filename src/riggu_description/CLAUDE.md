# riggu_description

- **Dimensions:** every dimension, mass, sensor pose and limit lives in `config/robot.yaml` as `{value, source, note}`. `source` is one of `estimate | old_config | datasheet | user | measured | calibrated`. Never invent a value without tagging it `estimate`. Never change a `measured`/`calibrated` value without the user.
- **Datums, not offsets:** YAML values are things a tape measure reaches (floor, chassis edges, centreline, axle, deck). `urdf/properties.xacro` derives the frame offsets; keep raw numbers out of the other xacro files.
- **Hardware selection:** `urdf/ros2_control.xacro` picks the hardware from `mode`; an unknown mode fails xacro.
  - `real`: DriveLink's installed `drivelink.ros2_control.xacro` macro (`$(find drivelink_bringup)`). Don't copy the plugin class name into this repo.
  - `sim`: `gz_ros2_control/GazeboSimSystem`; params via the `controllers_file` arg.
  - `mock`: `mock_components/GenericSystem` with `calculate_dynamics`.
- **Joints and interfaces must match DriveLink and `riggu_bringup/config/controllers.yaml`:**
  - Joint names: `left_wheel_joint`, `right_wheel_joint`; both rotate about +y, so +velocity is forward.
  - Command interface: velocity. State interfaces: position and velocity.
  - IMU sensor: `imu` on link `imu_link`.
- **Frames:** `base_footprint` is the root on the floor; `base_link` sits above it at wheel-axle height. Lidar frame is `laser`.
- **Dependencies:** no GUI or Gazebo packages in `package.xml` (they would reach the Pi). `drivelink_bringup` is skipped by rosdep.
- **Tests:** `test/test_urdf.py` expands all three modes, runs `check_urdf`, checks interfaces, frames, the YAML schema and that the parts fit. Keep it green after every YAML edit.
