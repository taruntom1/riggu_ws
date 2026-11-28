# riggu_bringup

Top-level bring-up for the real robot, the simulation, and mock hardware. Phase 3: RSP, ros2_control, controllers. EKF, teleop, collision monitor and lidar come in Phases 4–6.

| File | Purpose |
|---|---|
| `launch/bringup.launch.py` | RSP + `ros2_control_node` (real/mock; in sim gz runs it) + spawners. Args: `mode:=real\|sim\|mock` (default `mock`), `use_sim_time`, `robot_config` |
| `config/controllers.yaml` | `joint_state_broadcaster`, `imu_sensor_broadcaster`, `diff_drive_controller` (velocity, 50 Hz, `cmd_vel_timeout` 0.5 s). Shared by all modes |
| `test/test_bringup_mock.py` | launch_test, `mode:=mock`: 3 controllers active, `/joint_states`, `/imu`, `/cmd_vel_safe` → `/odom`, timeout brakes. `ROS_DOMAIN_ID=87` |
| `systemd/rmw-zenohd.service` | Zenoh router on the Pi (rmw_zenoh default config, `tcp/[::]:7447`). Installed and enabled by `setup_local.sh --profile robot` |

Planned: `config/ekf.yaml`, `config/ekf_gyro_only.yaml` (Phase 6), `config/collision_monitor.yaml` (Phase 5), `systemd/riggu-base.service` (Phase 5), lidar driver in `mode:=real` (Phase 4).

**Controller wiring** (spawner args in the launch file, not the yaml):
- `wheel_radius`, `wheel_separation` and the linear/angular velocity and acceleration limits come from `riggu_description/config/robot.yaml` (`wheel.*`, `limits.*`).
- `diff_drive_controller`: `~/cmd_vel` ← `/cmd_vel_safe` (`TwistStamped`), `~/odom` → `/odom`. `imu_sensor_broadcaster`: `~/imu` → `/imu`.
- **TF:** until the Phase 6 EKF, `diff_drive_controller` publishes `odom → base_footprint` (`enable_odom_tf: true`).

**cmd_vel chain** (from Phase 5; until then teleop publishes straight to `/cmd_vel_safe`)

```
/cmd_vel_joy ┐
/cmd_vel_nav ┴→ twist_mux → /cmd_vel_mux → collision_monitor → /cmd_vel_safe → diff_drive_controller
/estop (lock) ┘
```

```bash
ros2 launch riggu_bringup bringup.launch.py mode:=mock
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -r cmd_vel:=/cmd_vel_safe
```

Activating `diff_drive_controller` on `real` arms the DriveLink motors: `mode:=real` arms them as soon as the spawner finishes.
