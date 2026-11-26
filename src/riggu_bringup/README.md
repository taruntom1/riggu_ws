# riggu_bringup

Top-level bring-up for the real robot, the simulation, and mock hardware. *Planned — Phases 3–6.*

| File | Purpose |
|---|---|
| `launch/bringup.launch.py` | RSP + ros2_control + spawners + lidar (real) + EKF + teleop + collision monitor. Args: `mode:=real\|sim\|mock`, `use_sim_time`, `use_imu_orientation` |
| `config/controllers.yaml` | `joint_state_broadcaster`, `imu_sensor_broadcaster`, `diff_drive_controller` (velocity, `enable_odom_tf: false`). Shared by all modes |
| `config/ekf.yaml` | EKF fusing wheel vx/vyaw and IMU vyaw + yaw, `two_d_mode`, 30 Hz |
| `config/ekf_gyro_only.yaml` | Same, without IMU absolute yaw |
| `config/collision_monitor.yaml` | Stop and slowdown polygons on `/scan` |
| `systemd/riggu-base.service` | Starts base bring-up on boot (Pi); after `rmw-zenohd`, `drivelink-agent` |

**cmd_vel chain**

```
/cmd_vel_joy ┐
/cmd_vel_nav ┴→ twist_mux → /cmd_vel_mux → collision_monitor → /cmd_vel_safe → diff_drive_controller
/estop (lock) ┘
```

**TF:** the EKF publishes `odom → base_footprint` from `/odom` and `/imu`, and writes `/odometry/filtered`.

Activating `diff_drive_controller` on `real` arms the DriveLink motors.
