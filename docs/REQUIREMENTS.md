# riggu — Software Requirements

Differential-drive autonomous robot: ROS 2 Jazzy, rmw_zenoh, Raspberry Pi 5, DriveLink (ESP32-S3), RPLIDAR C1, BNO055 IMU.

## Platform

| Item | Decision |
|---|---|
| ROS distro | Jazzy everywhere (Pi + dev + sim) |
| Middleware | `rmw_zenoh_cpp`; `rmw_zenohd` router runs on the Pi as a systemd service; laptop/Docker connect as zenoh clients over Tailscale (Pi in a dedicated tailnet, shared to users; offline fallback via a Windows hotspot port-forward); sim runs its own local router |
| Robot computer | Raspberry Pi 5, Ubuntu 24.04 server (arm64) |
| Dev machine | Jazzy in Docker (host is WSL2 Ubuntu 26.04); GUI via WSLg/X11 |
| Workspace | Monorepo `riggu_ws`, packages prefixed `riggu_` |

## Hardware interface

- DriveLink agent on the Pi; ESP32-S3 connected over **USB** (udev rule → stable `/dev/drivelink`).
- `ros2_control` with the DriveLink hardware plugin; `diff_drive_controller` in **velocity** mode (ESP-side PID).
- `joint_state_broadcaster` + `imu_sensor_broadcaster` for wheel and IMU state.
- RPLIDAR C1 via `rplidar_ros` (`sllidar_ros2`), udev rule → `/dev/rplidar`.

## State estimation

- `robot_localization` EKF (`odom → base_link`) fusing:
  - diff_drive_controller wheel odometry: vx, yaw rate
  - BNO055: gyro z, **absolute orientation (yaw)**
- Orientation fusion is a config switch so it can be turned off if magnetic interference near motors spoils yaw.
- diff_drive_controller does **not** publish TF (EKF owns it).

## Mapping, localisation, navigation

- Mapping: `slam_toolbox` (online async), map save/serialize.
- Localisation: Nav2 AMCL on the saved map.
- Nav2 local controller: **MPPI**, tuned for Pi 5.
- Extras: **collision monitor** (lidar stop/slow zones before the motors), **waypoint following**.
- All SLAM/Nav2 nodes run **on the Pi**; laptop runs only RViz/monitoring.

## Manual override

- Generic Bluetooth gamepad paired to the **Pi**; `joy` + `teleop_twist_joy`.
- Configurable button/axis mapping file plus a guide for finding indices with `ros2 topic echo /joy`.
- `twist_mux`: joystick (deadman held) > Nav2; latching stop button; lock topic for e-stop.
- Command chain: `twist_mux → collision_monitor → diff_drive_controller`.

## Simulation

- Gazebo Harmonic with `gz_ros2_control`; same URDF/xacro and controller config as the real robot, only the hardware plugin differs (`use_sim` arg).
- Simulated RPLIDAR C1 (gpu_lidar) and IMU matching real specs.
- Default world: small indoor house/office.
- Same launch files for sim and real (`sim:=true|false`).

## Tooling & deployment

- **Docker**: Jazzy + Gazebo Harmonic image, docker compose; run script auto-detects WSLg / X11 / Wayland and NVIDIA or `/dev/dri`, falls back to software rendering.
- **Setup scripts** (native):
  - `robot` profile — Pi 5: ROS base, DriveLink agent + hardware, lidar, Nav2, zenoh router service, udev rules, Bluetooth; no Gazebo.
  - `dev` profile — Ubuntu 24.04 desktop: desktop-full, Gazebo Harmonic, RViz, Nav2.
- Pi deploy: native colcon build; systemd unit starts base bringup on boot (agent, ros2_control, lidar, EKF, teleop/twist_mux, zenoh router). SLAM / Nav2 launched on demand.
- **CI**: GitHub Actions builds in the Jazzy image and runs `colcon test` (URDF check, launch tests, ament lint).

## Planned packages

| Package | Contents |
|---|---|
| `riggu_description` | URDF/xacro, meshes, ros2_control tags (real + sim) |
| `riggu_bringup` | Top-level launch, controllers.yaml, EKF config, systemd units |
| `riggu_gazebo` | Worlds, Gazebo launch, bridges |
| `riggu_navigation` | slam_toolbox, AMCL, Nav2 (MPPI, collision monitor, waypoints), maps |
| `riggu_teleop` | joy, teleop_twist_joy mapping, twist_mux config |
| `docker/`, `scripts/` | Image, compose, run and setup scripts, udev rules, zenoh configs |

## Needed from you before the URDF step

- Wheel radius, wheel separation (track width), wheel width
- Chassis shape and dimensions, height of base above ground
- Caster type and position(s)
- Lidar mount position (x, y, z from base centre) and its yaw/orientation
- IMU (BNO055) mount position and axis orientation
- Masses (total or per part, for sim inertia)
- Max wheel speed (rad/s) and desired max linear/angular velocity and acceleration
- Encoder counts/rev and gear ratio (if not already set in DriveLink)
- DriveLink device alias and joint labels (`left`, `right`, `imu` defaults)
