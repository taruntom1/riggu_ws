# riggu_bringup

- **Controllers:** `config/controllers.yaml` is shared by `real`, `sim`, and `mock`. Mode differences belong in the launch file only.
- **Odometry TF:** the EKF owns `odom → base_footprint` from Phase 6, with `diff_drive_controller` `enable_odom_tf: false`. Until then (Phase 3–5) it is `true`.
- **Wheel params and limits** come from `riggu_description/config/robot.yaml`, passed by `bringup.launch.py` as spawner `--controller-ros-args=-p ...` (use the `=` form; a bare `-p` value is parsed as the spawner's `--param-file`). Never copy them into `controllers.yaml`.
- **Velocity commands** are `geometry_msgs/TwistStamped` (Jazzy `diff_drive_controller`).
- **Arming:** activating `diff_drive_controller` claims DriveLink command interfaces, which **arms the motors**. Deactivating it disarms them. `mode:=real` therefore spawns it `--inactive`; never make real-mode launch activate it. Treat activation (and `drivelink_diagnostics/recover`) on the real robot as a motion-capable action.
- **IMU optional:** `imu:=false` removes `<sensor name="imu">` from the xacro output in `bringup.launch.py` (DriveLink refuses a configured-but-missing instance, and its macro always emits the sensor). Don't copy the DriveLink block to avoid it; drop the strip once DriveLink's macro can omit the IMU.
- **real-only nodes:** `sllidar_node` (`config/lidar.yaml`) and `drivelink_bringup/diagnostics`; their DriveLink connection args are shared with the xacro through `drivelink_*` launch args.
- **cmd_vel chain:** `twist_mux` → `/cmd_vel_mux` → `collision_monitor` → `/cmd_vel_safe` → `diff_drive_controller`. Nothing else publishes to the controller.
- **EKF:**
  - `config/ekf.yaml` fuses wheel vx and vyaw plus IMU vyaw and absolute yaw.
  - `config/ekf_gyro_only.yaml` drops absolute yaw.
  - The `use_imu_orientation` launch arg chooses between them.
- **systemd:** units in `systemd/` are installed by `scripts/setup_local.sh --profile robot`, which replaces `@USER@` and `@WS@`. `riggu-base.service` starts after `rmw-zenohd` and `drivelink-agent`.
