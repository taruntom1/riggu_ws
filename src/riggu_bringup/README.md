# riggu_bringup

Top-level bring-up for the real robot, the simulation, and mock hardware: RSP, ros2_control, controllers; on `real` also the C1 lidar and DriveLink diagnostics. EKF, teleop and collision monitor come in Phases 5–6.

| File | Purpose |
|---|---|
| `launch/bringup.launch.py` | RSP + `ros2_control_node` (real/mock; in sim gz runs it) + spawners. `real`: + `sllidar_node` (respawn), `drivelink_bringup/diagnostics`, `diff_drive_controller` loaded **inactive**. Args: `mode:=real\|sim\|mock` (default `mock`), `imu:=true\|false` (false: the `imu` sensor is stripped from the expanded ros2_control block and `imu_sensor_broadcaster` is not spawned), `use_sim_time`, `robot_config`, `lidar_config`, `drivelink_agent` (`tcp://127.0.0.1:47620`), `drivelink_device` (`base`), `drivelink_token_file` (`/etc/drivelink/token`) |
| `config/controllers.yaml` | `joint_state_broadcaster`, `imu_sensor_broadcaster`, `diff_drive_controller` (velocity, 50 Hz, `cmd_vel_timeout` 0.5 s). Shared by all modes |
| `config/lidar.yaml` | `sllidar_node`: C1 on `/dev/rplidar`, 460800 baud, frame `laser`, `Standard` mode |
| `scripts/calibrate_wheels.py` | `ros2 run riggu_bringup calibrate_wheels straight <m>\|turns <n>`: wheel angles from `/joint_states` between two Enters → odom error and calibrated `wheel.radius` / `wheel.separation`. Prints only |
| `test/test_bringup_launch.py`, `test/test_calibrate_wheels.py` | pytest: drive spawner `--inactive` only on `real`; `imu:=false` description; calibration maths |
| `test/test_bringup_mock.py` | launch_test, `mode:=mock`: 3 controllers active, `/joint_states`, `/imu`, `/cmd_vel_safe` → `/odom`, timeout brakes. `ROS_DOMAIN_ID=87` |
| `systemd/rmw-zenohd.service` | Zenoh router on the Pi (rmw_zenoh default config, `tcp/[::]:7447`). Installed and enabled by `setup_local.sh --profile robot` |

Planned: `config/ekf.yaml`, `config/ekf_gyro_only.yaml` (Phase 6), `config/collision_monitor.yaml` (Phase 5), `systemd/riggu-base.service` (Phase 5).

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

## Real robot

`mode:=real` loads `diff_drive_controller` inactive: the board stays disarmed until you activate it. Activating arms the motors; `/drivelink_diagnostics/recover` re-activates it too.

```bash
# Pi (source scripts/env.sh robot)
ros2 launch riggu_bringup bringup.launch.py mode:=real imu:=false   # until the BNO055 is fitted
ros2 control switch_controllers --activate diff_drive_controller --strict    # ARMS
ros2 control switch_controllers --deactivate diff_drive_controller --strict  # disarms
ros2 topic echo /diagnostics --once        # online, lease holder, arm state, faults
ros2 service call /drivelink_diagnostics/ack_fault std_srvs/srv/Trigger '{}'

# Laptop, in the container (source scripts/env.sh remote)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p speed:=0.1 -p turn:=0.5 -r cmd_vel:=/cmd_vel_safe
rviz2 -d "$(ros2 pkg prefix riggu_gazebo)/share/riggu_gazebo/rviz/sim.rviz"
```

**Calibration** (wheels on the floor, slow teleop, `wheel.radius` first):
1. `calibrate_wheels straight 2.0`: Enter, drive straight to a tape-measured 2 m mark, Enter.
2. `calibrate_wheels turns 10`: Enter, 10 full turns in place against a floor mark, Enter.
3. Write the printed values to `riggu_description/config/robot.yaml` (`source: calibrated`, date in `note`), rebuild, restart bringup, repeat both: the odometry error must be under 2 %.
