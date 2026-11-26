# riggu_ws

ROS 2 Jazzy workspace for **riggu**, a differential-drive robot:

| Area | Components |
|---|---|
| Compute | Raspberry Pi 5, Ubuntu 24.04 |
| Motors, encoders, IMU | [DriveLink](https://taruntom1.github.io/drivelink_v2/) ESP32-S3 (ros2_control), BNO055 |
| Lidar | RPLIDAR C1 |
| Middleware | rmw_zenoh |
| Simulator | Gazebo Harmonic |

**Stack**

| Function | Implementation |
|---|---|
| Odometry | `diff_drive_controller` (velocity mode) fused by a `robot_localization` EKF (wheel + BNO055 gyro and yaw) |
| Mapping | `slam_toolbox` |
| Localisation | AMCL |
| Navigation | Nav2 (Smac 2D planner, MPPI controller), with collision monitor and waypoint following |
| Manual control | Bluetooth gamepad on the Pi through `twist_mux`, with deadman and latching e-stop |

Progress: [docs/STATUS.md](docs/STATUS.md). Plan: [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md).

## Layout
| Path | Contents |
|---|---|
| `src/riggu_description` | URDF/xacro; `mode:=real\|sim\|mock` hardware selection |
| `src/riggu_bringup` | Top-level launch, controllers, EKF, collision monitor, systemd |
| `src/riggu_gazebo` | Worlds, sim launch, bridges |
| `src/riggu_navigation` | SLAM, AMCL, Nav2, maps, waypoints |
| `src/riggu_teleop` | Joystick, twist_mux, stop latch |
| `docker/` | Jazzy + Gazebo dev/sim image, compose |
| `scripts/` | Setup (robot/dev), env, deploy, docker run |
| `config/` | zenoh profiles, udev rules |

## Quickstart
Available after Phase 1.

```bash
scripts/docker_run.sh                       # dev/sim container (GPU + WSLg/X11 auto-detected)
scripts/setup_local.sh --profile dev|robot  # native Ubuntu 24.04 install
source scripts/env.sh sim|remote|robot      # zenoh profile
```
