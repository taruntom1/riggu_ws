# riggu_description

- **Dimensions:** every dimension and mass lives in `urdf/properties.xacro`. Values come from the user and are never invented; scaffold placeholders are marked `TODO(measure)`.
- **Hardware selection:** `urdf/ros2_control.xacro` picks the hardware from `mode`:
  - `real`: DriveLink's installed `drivelink.ros2_control.xacro` macro. Locate it with `ros2 pkg prefix --share drivelink_bringup`. Don't copy the plugin class name into this repo.
  - `sim`: `gz_ros2_control/GazeboSimSystem`.
  - `mock`: `mock_components/GenericSystem`.
- **Joints and interfaces must match DriveLink and `riggu_bringup/config/controllers.yaml`:**
  - Joint names: `left_wheel_joint`, `right_wheel_joint`.
  - Command interface: velocity.
  - State interfaces: position and velocity.
  - IMU sensor: `imu` on link `imu_link`.
- **Frames:** `base_footprint` is the root, and `base_link` sits above it at wheel-axle height.
- **Tests:** xacro expands for all three modes and `check_urdf` passes.
