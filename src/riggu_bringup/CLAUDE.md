# riggu_bringup

- **Controllers:** `config/controllers.yaml` is shared by `real`, `sim`, and `mock`. Mode differences belong in the launch file only.
- **Odometry TF:** set `diff_drive_controller` `enable_odom_tf: false`. The EKF owns `odom → base_footprint`.
- **Arming:** activating `diff_drive_controller` claims DriveLink command interfaces, which **arms the motors**. Deactivating it disarms them. Treat controller activation on the real robot as a motion-capable action.
- **cmd_vel chain:** `twist_mux` → `/cmd_vel_mux` → `collision_monitor` → `/cmd_vel_safe` → `diff_drive_controller`. Nothing else publishes to the controller.
- **EKF:**
  - `config/ekf.yaml` fuses wheel vx and vyaw plus IMU vyaw and absolute yaw.
  - `config/ekf_gyro_only.yaml` drops absolute yaw.
  - The `use_imu_orientation` launch arg chooses between them.
- **systemd:** units in `systemd/` are installed by `scripts/setup_local.sh --profile robot`. `riggu-base.service` starts after `rmw-zenohd` and `drivelink-agent`.
