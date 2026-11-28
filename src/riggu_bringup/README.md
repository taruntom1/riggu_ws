# riggu_bringup

Top-level bring-up for the real robot, the simulation, and mock hardware: RSP, ros2_control, controllers, the cmd_vel chain (riggu_teleop + collision monitor); the `robot_localization` EKF; on `real` also the C1 lidar, DriveLink diagnostics and the pad.

| File | Purpose |
|---|---|
| `launch/bringup.launch.py` | RSP + `ros2_control_node` (real/mock; in sim gz runs it) + spawners. `real`: + `sllidar_node` (respawn), `drivelink_bringup/diagnostics`, `diff_drive_controller` loaded **inactive**. Args: `mode:=real\|sim\|mock` (default `mock`), `imu:=true\|false` (false: the `imu` sensor is stripped from the expanded ros2_control block, `imu_sensor_broadcaster` is not spawned and the EKF drops `imu0`), `use_imu_orientation:=false\|true` (default false: EKF fuses the gyro only; true adds the BNO055 absolute yaw), `ekf_config`, `ekf_gyro_only_config`, `use_sim_time`, `robot_config`, `lidar_config`, `drivelink_agent` (`tcp://127.0.0.1:47620`), `drivelink_device` (`base`), `drivelink_token_file` (`/etc/drivelink/token`), `drivelink_{left,right}_label` (`pid_left`, `pid_right`); chain: `joy`, `arm` (`auto`: on for `real` only), `collision_check_joy` (`true`; `false`: joystick bypasses the monitor), `collision_monitor_config` |
| `config/controllers.yaml` | `joint_state_broadcaster`, `imu_sensor_broadcaster`, `diff_drive_controller` (velocity, 50 Hz, `cmd_vel_timeout` 0.5 s). Shared by all modes |
| `config/ekf.yaml` | `ekf_filter_node` (30 Hz, `two_d_mode`): `/wheel/twist` vx, vy (= 0), vyaw + `/imu/corrected` yaw (`imu0_relative`: odom starts at yaw 0) and vyaw → `/odometry/filtered`, TF `odom → base_footprint`. Shared by all modes |
| `config/ekf_gyro_only.yaml` | Overrides on `ekf.yaml` for `use_imu_orientation:=false` (**default**): `imu0_config` vyaw only. Floor tests: the magnetometer heading drifts 3–6° on straights |
| `scripts/imu_scale.py` | Node: `/imu` → `/imu/corrected`, yaw rate ÷ `gyro_z_scale` (`robot.yaml` `imu.gyro_z_scale` on `real`, 1.0 in sim/mock). Stop-gap until drivelink_v2#136 |
| `config/collision_monitor.yaml` | nav2 `collision_monitor`: one `approach` polygon (1 s ahead) on `/scan`; stale scan (> 1 s) = stop. Footprint (`FootprintApproach.points`) = chassis + wheels from `robot.yaml` + `riggu_footprint.margin`, set by the launch. `mode:=mock`: `scan.source_timeout` 0 (no lidar) |
| `config/lidar.yaml` | `sllidar_node`: C1 on `/dev/rplidar`, 460800 baud, frame `laser`, `Standard` mode |
| `scripts/wheel_twist.py`, `config/wheel_twist.yaml` | Node: `/joint_states` encoder **position** differences → `/wheel/twist` (`TwistWithCovarianceStamped`, `base_footprint`), armed or not; the EKF's wheel input. Radius/separation from `robot.yaml` via the launch; covariance as `diff_drive_controller`'s |
| `scripts/odom_check.py` | `ros2 run riggu_bringup odom_check`: Enter at a floor mark, drive a closed path (square, N spins) back onto it, Enter → per source (`/odom`, `/odometry/filtered`, `/imu` yaw) distance from start and yaw residual from whole turns, plus the max IMU-minus-wheel yaw drift during the run (magnetic interference) and the raw gyro integral with the `gyro_z_scale` it implies (after N spins on a mark) |
| `scripts/calibrate_wheels.py` | `ros2 run riggu_bringup calibrate_wheels straight <m>\|turns <n>`: wheel angles from `/joint_states` between two Enters → odom error and calibrated `wheel.radius` / `wheel.separation`. Prints only |
| `test/test_bringup_launch.py`, `test/test_calibrate_wheels.py` | pytest: drive spawner `--inactive` only on `real`; `imu:=false` description; chain wiring and bypass; footprint; calibration maths; EKF params (gyro-only, `imu:=false`), only the EKF publishes odom TF |
| `test/test_odom_check.py`, `test/test_wheel_twist.py`, `test/test_imu_scale.py` | pytest: yaw unwrapping, turn residuals, report, gyro scale; diff-drive kinematics from positions; scale correction |
| `test/test_bringup_mock.py` | launch_test, `mode:=mock joy:=true arm:=true`: 3 controllers active, `/joint_states`, `/imu`; `/cmd_vel_nav` → chain → `/odom`, timeout brakes; `/odometry/filtered` and TF `odom → base_footprint` follow it, EKF vx variance stays small while disarmed; fake `/joy`: deadman, joy beats nav, stop latch, pad disarm/arm. `ROS_DOMAIN_ID=87` |
| `systemd/rmw-zenohd.service` | Zenoh router on the Pi (rmw_zenoh default config, `tcp/[::]:7447`). Installed and enabled by `setup_local.sh --profile robot` |
| `systemd/riggu-base.service` | `bringup.launch.py mode:=real $RIGGU_BRINGUP_ARGS` on boot (none by default; e.g. `imu:=false` or `use_imu_orientation:=false` in `/etc/default/riggu-base`), after `rmw-zenohd`, `drivelink-agent`, `bluetooth`. Comes up **disarmed**. Enabled by `setup_local.sh`, restarted by `deploy.sh` |

**Controller wiring** (spawner args in the launch file, not the yaml):
- `wheel_radius`, `wheel_separation` and the linear/angular velocity and acceleration limits come from `riggu_description/config/robot.yaml` (`wheel.*`, `limits.*`).
- `diff_drive_controller`: `~/cmd_vel` ← `/cmd_vel_safe` (`TwistStamped`), `~/odom` → `/odom`. `imu_sensor_broadcaster`: `~/imu` → `/imu`.
- **TF:** the EKF publishes `odom → base_footprint`; `diff_drive_controller` has `enable_odom_tf: false` and only publishes `/odom` (raw, and only while armed: an inactive controller doesn't update). The EKF's wheel input is `/wheel/twist`, which keeps flowing while disarmed.

**cmd_vel chain**

```
/cmd_vel_joy ┐
/cmd_vel_nav ┴→ twist_mux → /cmd_vel_mux → collision_monitor → /cmd_vel_safe → diff_drive_controller
/estop (lock) ┘
collision_check_joy:=false:  /cmd_vel_nav → collision_monitor → /cmd_vel_nav_checked → twist_mux → /cmd_vel_safe
```

- The approach polygon slows and stops only motion that would hit within 1 s; moving away stays free. An obstacle already *inside* the footprint blocks every direction: drive out with `collision_check_joy:=false`.
- Lidar data older than 1 s (C1 USB drop) stops the robot until scans return.

```bash
ros2 launch riggu_bringup bringup.launch.py mode:=mock
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -r cmd_vel:=/cmd_vel_joy
```

## Real robot

`mode:=real` loads `diff_drive_controller` inactive: the board stays disarmed until you activate it, from the pad (LB + Start for 1 s; Back disarms, see `riggu_teleop`) or with `switch_controllers`. Activating arms the motors. After a DriveLink link loss the hardware goes `unconfigured`; `/drivelink_diagnostics/recover` (or `ros2 control set_hardware_component_state drivelink active`) restores it and the broadcasters, still disarmed.

```bash
# Pi (source scripts/env.sh robot). On boot riggu-base.service runs the bringup:
sudo systemctl status riggu-base; journalctl -u riggu-base -f
ros2 launch riggu_bringup bringup.launch.py mode:=real   # by hand: stop riggu-base first
ros2 control switch_controllers --activate diff_drive_controller --strict    # ARMS
ros2 control switch_controllers --deactivate diff_drive_controller --strict  # disarms
ros2 topic echo /diagnostics --once        # online, lease holder, arm state, faults
ros2 service call /drivelink_diagnostics/ack_fault std_srvs/srv/Trigger '{}'
ros2 service call /drivelink_diagnostics/recover std_srvs/srv/Trigger '{}'  # after a link loss; stays disarmed

# Laptop, in the container (source scripts/env.sh remote)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p speed:=0.1 -p turn:=0.5 -r cmd_vel:=/cmd_vel_joy
rviz2 -d "$(ros2 pkg prefix riggu_gazebo)/share/riggu_gazebo/rviz/sim.rviz"
```

**Calibration** (wheels on the floor, slow teleop, `wheel.radius` first):
1. `calibrate_wheels straight 2.0`: Enter, drive straight to a tape-measured 2 m mark, Enter. Write `wheel.radius` to `riggu_description/config/robot.yaml` (`source: calibrated`, date in `note`) before step 2: the turns result uses it.
2. `calibrate_wheels turns 10`: Enter, 10 full turns in place against a floor mark, Enter. Write `wheel.separation`.
3. Restart bringup (it reads `robot.yaml` at launch) and repeat both: the odometry error must be under 2 %. 2026-11-28: radius 0.06374 m, separation 0.3853 m → 1 % straight, 0.2 % turns.
