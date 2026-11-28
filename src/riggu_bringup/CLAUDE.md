# riggu_bringup

- **Controllers:** `config/controllers.yaml` is shared by `real`, `sim`, and `mock`. Mode differences belong in the launch file only.
- **Odometry TF:** the EKF (`ekf_filter_node`, every mode) owns `odom → base_footprint`; `diff_drive_controller` keeps `enable_odom_tf: false`. Never let two nodes publish it.
- **Wheel params and limits** come from `riggu_description/config/robot.yaml`, passed by `bringup.launch.py` as spawner `--controller-ros-args=-p ...` (use the `=` form; a bare `-p` value is parsed as the spawner's `--param-file`). Never copy them into `controllers.yaml`.
- **Velocity commands** are `geometry_msgs/TwistStamped` (Jazzy `diff_drive_controller`).
- **Arming:** activating `diff_drive_controller` claims DriveLink command interfaces, which **arms the motors**. Deactivating it disarms them. `mode:=real` therefore spawns it `--inactive`; never make real-mode launch activate it. Treat activation on the real robot as a motion-capable action. `recovery_controllers` for `drivelink_diagnostics/recover` excludes the drive for the same reason. The one automated exception is `riggu_teleop/arm_switch` (pad combo, a deliberate human action), started by `arm:=auto` on `real`.
- **IMU optional:** `imu:=false` removes `<sensor name="imu">` from the xacro output in `bringup.launch.py` (DriveLink refuses a configured-but-missing instance, and its macro always emits the sensor). Don't copy the DriveLink block to avoid it; drop the strip once DriveLink's macro can omit the IMU.
- **real-only nodes:** `sllidar_node` (`config/lidar.yaml`) and `drivelink_bringup/diagnostics`; their DriveLink connection args are shared with the xacro through `drivelink_*` launch args.
- **cmd_vel chain:** `twist_mux` → `/cmd_vel_mux` → `collision_monitor` → `/cmd_vel_safe` → `diff_drive_controller`. Nothing else publishes to the controller; manual tools publish to `/cmd_vel_joy` or `/cmd_vel_nav`. `collision_check_joy:=false` moves the monitor onto the nav input (`/cmd_vel_nav_checked`) and twist_mux publishes `/cmd_vel_safe`.
- **Collision monitor:** lifecycle node under its own `lifecycle_manager_collision_monitor`. The footprint comes from `robot.yaml` (`footprint()` in the launch) plus `riggu_footprint.margin`; don't hard-code points in the yaml. A stale `/scan` stops the robot (`source_timeout`); only `mode:=mock` disables that.
- **EKF:**
  - `config/ekf.yaml` fuses wheel vx, vy (= 0) and vyaw plus IMU vyaw and absolute yaw.
  - The wheel input is `/wheel/twist` from `wheel_twist` (`/joint_states`), **not** `/odom`: `diff_drive_controller` is inactive while disarmed and then publishes nothing, and an EKF without a velocity measurement diverges (seen on the robot: hundreds of metres, then jumps on arming). Never fuse `/odom` instead.
  - `config/ekf_gyro_only.yaml` drops absolute yaw.
  - `config/ekf_gyro_only.yaml` holds only the overriding keys, applied on top of `ekf.yaml` by `ekf_params()` in the launch; `imu:=false` drops every `imu0*` key.
  - The `use_imu_orientation` launch arg chooses between them.
  - Input covariances belong to the sources (`covariance_diagonal` in `wheel_twist.yaml`, kept equal to `diff_drive_controller`'s `twist_covariance_diagonal`; `static_covariance_*` in `controllers.yaml`), not the EKF file.
- **systemd:** units in `systemd/` are installed by `scripts/setup_local.sh --profile robot`, which replaces `@USER@` and `@WS@`. `riggu-base.service` starts after `rmw-zenohd`, `drivelink-agent` and `bluetooth`, and is only enabled (deploy builds, then restarts it). Its launch args come from `RIGGU_BRINGUP_ARGS` (`/etc/default/riggu-base`).
