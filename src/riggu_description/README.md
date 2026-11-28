# riggu_description

URDF/xacro model of riggu, generated from one dimensions file.

| File | Purpose |
|---|---|
| `config/robot.yaml` | **All dimensions, masses, sensor poses and limits.** Each value is `{value, source, note}`. `riggu_bringup` reads `wheel.radius`, `wheel.separation` and `limits.*` for `diff_drive_controller` |
| `urdf/riggu.urdf.xacro` | Top level. Args below |
| `urdf/properties.xacro` | Loads `robot.yaml`, derives every frame offset (no raw numbers) |
| `urdf/base.xacro` | `base_footprint`, `base_link` (chassis box), wheels, rear ball caster. In `sim` the wheel collision is a sphere (cylinder edge contacts in DART turn ~7% too fast) |
| `urdf/sensors.xacro` | `laser` (RPLIDAR C1), `imu_link` (BNO055); gz `gpu_lidar` + `imu` (noise and bias from `imu.*_stddev`) in `sim` |
| `urdf/ros2_control.xacro` | `real`: DriveLink macro · `sim`: `gz_ros2_control/GazeboSimSystem` · `mock`: `mock_components/GenericSystem` |
| `urdf/inertial.xacro` | Box / cylinder / sphere inertia macros |
| `launch/display.launch.py` | robot_state_publisher, joint_state_publisher_gui, RViz |
| `test/test_urdf.py` | xacro per mode, `check_urdf`, interfaces, frames, YAML schema, geometry fit |

**xacro args:** `mode:=real|sim|mock` (default `mock`) · `robot_config:=<yaml>` · `controllers_file:=<yaml>` (sim: gz_ros2_control params) · `drivelink_agent` (`tcp://127.0.0.1:47620`), `drivelink_device` (`base`), `drivelink_token_file` (`/etc/drivelink/token`), `drivelink_token`, `drivelink_{left,right,imu}_label` (`left`, `right`, `imu`).

**Frames:** `base_footprint` (floor) → `base_link` (axle midpoint, `wheel.radius` up) → `left_wheel`, `right_wheel`, `caster`, `laser` (optical plane, x = scan 0°), `imu_link`.

**ros2_control** (identical in every mode):

| Item | Name | Interfaces |
|---|---|---|
| Wheel joint | `left_wheel_joint`, `right_wheel_joint` | command `velocity` · state `position`, `velocity`; +velocity = forward |
| Sensor | `imu` (frame `imu_link`) | `orientation.*`, `angular_velocity.*`, `linear_acceleration.*` |

## Correcting the model

Values are measured from tape-friendly datums (floor, chassis front/rear edge, centreline, axle, deck top), not frame offsets. To correct one: edit `value`, set `source` (`estimate` → `measured`/`calibrated`, date in `note`), then

```bash
colcon build --symlink-install --packages-select riggu_description
colcon test --packages-select riggu_description && colcon test-result --verbose
ros2 launch riggu_description display.launch.py
```

The geometry tests fail if the parts stop fitting (wheel or caster off the floor, wheels inside the chassis, scan plane below the deck or wheel tops, IMU outside the chassis). Phase 4 writes calibrated `wheel.radius` / `wheel.separation` here; `diff_drive_controller` reads the same file.

Current state: wheel radius and separation from the old DriveLink config; chassis, caster and IMU are estimates; C1 from its datasheet. Sensor poses are verified in Phase 4.

## Notes
- `display.launch.py` needs `rviz2` and `joint_state_publisher_gui` (dev profile / Docker image). They are deliberately not in `package.xml`, so they stay off the Pi.
- `drivelink_bringup` comes from the DriveLink apt repo; rosdep calls pass `--skip-keys drivelink_bringup`.
