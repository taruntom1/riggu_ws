# Status

Updated 2026-11-28. Next phase: **7** (in progress). Run a phase with `/implement-phase <n>`.

| # | Phase | State | Branch / PR |
|---|---|---|---|
| 0 | Host tooling and repo | done | `phase/0-host-tooling` · [PR #1](https://github.com/taruntom1/riggu_ws/pull/1) |
| 1 | Workspace, Docker, setup scripts, CI | done | `phase/1-workspace` · [PR #2](https://github.com/taruntom1/riggu_ws/pull/2) |
| 2 | Robot description (URDF) | done | `phase/2-description` · [PR #3](https://github.com/taruntom1/riggu_ws/pull/3) |
| 3 | Simulated drive | done | `phase/3-sim-drive` · [PR #4](https://github.com/taruntom1/riggu_ws/pull/4) |
| 4 | Real base bring-up | done | `phase/4-real-base` · [PR #5](https://github.com/taruntom1/riggu_ws/pull/5) |
| 5 | Joystick, twist_mux, safety | done | `phase/5-teleop-safety` · [PR #6](https://github.com/taruntom1/riggu_ws/pull/6) |
| 6 | EKF fusion | done | `phase/6-ekf` · [PR #8](https://github.com/taruntom1/riggu_ws/pull/8) |
| 7 | Mapping and localisation | in progress | `phase/7-mapping` |
| 8 | Navigation | todo | — |
| 9 | Hardening and docs | todo | — |

States: `todo` · `in progress` · `review` (PR open) · `done` (merged) · `blocked`

## Needs from user
- **Pi:** `/etc/default/riggu-base` (`imu:=true`) is redundant since Phase 6 and can be removed.
- **Phase 6, optional:** one fresh square + spin with `odom_check` to confirm the gyro scale out of sample (the replay's spin is in-sample).
- **USB EMI:** both the C1 and the DriveLink USB links dropped once under motor load; secure and shield the cables (ferrites, routing away from motor wiring).
- **Any time:** tape-measure the `source: estimate` values in `src/riggu_description/config/robot.yaml` (chassis, caster, IMU, wheel width) and weigh the robot.

## Known environment facts
- The dev host is WSL2 Ubuntu 26.04 (`resolute`) with ROS **Lyrical** installed, so build in the Jazzy container.
- RTX 4050 + WSLg; systemd running; WSL `networkingMode=Mirrored` (LAN 172.16.32.0/21).
- Docker Engine 29.10.1 with the `nvidia` runtime: `docker run --gpus all` works. The user is in the `docker` group.
- `sudo` is passwordless (sudo-rs).
- GitHub SSH port 22 is blocked. `~/.ssh/config` routes `github.com` through `ssh.github.com:443`.
- Repo: https://github.com/taruntom1/riggu_ws (public). Git author is `taruntom1`, set in the repo-local config.
- Name history: the old repo named `riggu_ws` was renamed to `riggu_ws_old`, and the new repo took the name.
- **Pi network:** user `riggu` on the laptop's Windows mobile hotspot (192.168.137.126). WSL can't reach the hotspot subnet directly.
- **Primary access:** Tailscale. The Pi is the only machine in a dedicated riggu tailnet (tag `tag:robot`, no key expiry) and is shared into the user's personal tailnet.
  - The Windows Tailscale client is visible to WSL through mirrored mode.
  - `~/.ssh/config` maps `riggu`/`riggu.local` to the Pi's tailnet name.
  - The personal tailnet has a stale offline machine also named `riggu`, so always use the full name.
- **Fallback:** `ssh riggu-local` and `tcp/127.0.0.1:17447`, through the Windows portproxy.
- Pi: Ubuntu 24.04.4 aarch64, avahi active, key SSH works, passwordless sudo, Tailscale 1.104.7.
- Pi already has ros-jazzy-ros-base and DriveLink 0.2.1 (agent, cli 0.2.0, bringup). DriveLink board: CH343 `1a86:55d3` → `/dev/ttyACM0`, `/dev/drivelink/5B5E080149`.
- Dev image `riggu_ws:jazzy-dev` (7.1 GB). On WSL, GL is Mesa D3D12 on the RTX 4050 (OpenGL 4.6).
- DriveLink board (2026-11-28): controllers `pid_left`/`pid_right` (velocity mode, cascade PID), encoders 7385 counts/wheel rev, `imu` (`imu.bno055`, NDOF, calibration saved). The BNO055 chip is yawed 180° to `base_link`; its gyro reads 3.2 % fast. Agent on `0.0.0.0:47620`.
- The laptop clock drifted 92 s behind the Pi once; Windows "Sync now" fixed it. Laptop teleop needs synced clocks (`cmd_vel_timeout` uses the stamp).
- Pad: USB "ShanWan" PlayStation-shape pad, enumerates as Xbox 360 (`045e:028e`, xpad) → `/dev/input/js0`. Mapping in `riggu_teleop/config/joy_generic.yaml`.
- DriveLink macro: `drivelink_diff_drive(name agent device token_file token left_joint left_label right_joint right_label imu_label)`.

## Decisions / deviations
- Phase 0: the baseline commit (docs, `.gitignore`) went straight to `main`. The empty git repo in `/home/tarun` was removed, at the user's request.
- Phase 0: host installs are in `scripts/setup_host_tools.sh`, which runs on any Ubuntu.
- **Phase 0, Tailscale:** the Pi is in its own tailnet and shared to people, so it can't reach anyone's tailnet. Collaborators also get shares, not membership.
  - The policy allows only 22, 7447, 47620 and ICMP to the robot.
  - The hotspot port-forward is kept as the offline fallback.
  - The `remote` zenoh profile is client mode.
- **Phase 0, Pi sudo:** passwordless, as the user chose.
- **Phase 1, zenoh:** no `config/zenoh/*.json5`. Profiles are rmw_zenoh defaults plus `ZENOH_CONFIG_OVERRIDE` in `scripts/env.sh` (rmw_zenoh 0.2.13 supports it).
- **Phase 1, lidar:** apt `rplidar_ros` 2.1.0 has no C1 support, so Slamtec `sllidar_ros2` is pinned in `riggu.repos` → `src/external/`.
- **Phase 1, compose:** a single `dev` service (no separate `sim` service); sim is a command in it.
- **Phase 1, CI:** image built with the GHA layer cache, nothing published (user's choice).
- **Phase 2, source data:** the old `riggu_description` URDF is the unmodified articubot_one template, so it is not used. Wheel radius 66.75 mm and separation 390 mm come from the old DriveLink config. Layout (axle at the front, rear ball caster, single deck, lidar over the axle) is from the user. Other sizes are estimates; total mass is an estimate of ~2.5 kg.
- **Phase 2, params:** dimensions live in `riggu_description/config/robot.yaml` (`{value, source}`), loaded by `properties.xacro`. From Phase 3, `diff_drive_controller` takes its wheel params from it too.
- **Phase 2, done-when:** checking the sensor axes against the real IMU and lidar moved to Phase 4 checks 3–4 (Pi unreachable; user's choice).
- **Phase 2, deps:** `drivelink_bringup` is in `package.xml` and rosdep skips it (`--skip-keys`). `rviz2`/`joint_state_publisher_gui` are dev-only (Dockerfile, `setup_local.sh --profile dev`), not in `package.xml`.
- **Phase 2, container shells:** `bash -lc` (docker_run.sh commands, CI) now sources `env.sh sim` through `~/.profile`. Before, it fell back to FastDDS, and RViz on WSLg got no TF.
- **Phase 3, odom TF:** `diff_drive_controller` publishes `odom → base_footprint` (`enable_odom_tf: true`) until the Phase 6 EKF (user's choice). Teleop publishes straight to `/cmd_vel_safe` until Phase 5.
- **Phase 3, world:** hand-written primitive `house.sdf` (10×8 m, 4 rooms), no Fuel models (user's choice). CI runs only the mock launch test; Gazebo is checked locally, headless (user's choice).
- **Phase 3, sim wheels:** sphere collisions in `sim` only. With cylinders, in-place turns ran ~7% faster than odometry.
- **Phase 3, params:** wheel radius/separation and velocity/acceleration limits (new `limits.max_*_accel`, estimates) go from `robot.yaml` to `diff_drive_controller` as spawner `--controller-ros-args`. Sim IMU noise/bias values added to `robot.yaml`.
- **Phase 4, arming:** `mode:=real` loads `diff_drive_controller` inactive (as DriveLink's example does); arming is an explicit `switch_controllers --activate` (user's choice). DriveLink `diagnostics` runs in real mode for fault ack/recover.
- **Phase 4, agent bind:** `0.0.0.0:47620` (token-protected), plus a portproxy `127.0.0.1:47620` rule for the offline path (user's choice). Config template `config/drivelink/agent.json`.
- **Phase 4, IMU:** BNO055 not fitted yet. `bringup.launch.py imu:=false` strips the `imu` sensor from the expanded DriveLink block and skips `imu_sensor_broadcaster`; [drivelink_v2#135](https://github.com/taruntom1/drivelink_v2/issues/135) asks for an optional IMU in the macro (user's choice: both). IMU check 3 waits for the IMU.
- **Phase 4, calibration:** wheel radius 0.06374 m, separation 0.3853 m (`calibrated`); verification 1 % straight, 0.2 % turns. Lidar yaw 0 verified (`measured`).
- **Phase 4, recovery:** `drivelink_diagnostics` `recovery_controllers` excludes `diff_drive_controller`, so `recover` never arms (same rule as launch).
- **Phase 4, IMU check 3:** done in Phase 6 (2026-11-28) by hand tilts: z up, chip yawed 180° (`imu.yaw` π).
- **Phase 5, arming:** the pad arms (deadman LB + Start held 1 s → `diff_drive_controller` active) and disarms (Back) through `riggu_teleop/arm_switch`; boot and launch stay disarmed (user's choice).
- **Phase 5, collision monitor:** baseline is one `approach` polygon (footprint from `robot.yaml` + 3 cm, 1 s horizon); a stale `/scan` stops. Joystick bypass is the launch arg `collision_check_joy:=false` (monitor moves onto `/cmd_vel_nav`). Mock disables the scan timeout.
- **Phase 5, `/estop`:** a 10 Hz heartbeat with a 0.5 s twist_mux lock timeout, so a dead `stop_latch` locks the output. `~/set` (SetBool) latches it from the laptop.
- **Phase 6, IMU heading:** default `use_imu_orientation:=false` (gyro-only). Floor tests with ground truth showed the magnetometer heading drifting 3–6° on straights; the BNO055 gyro reads 3.2 % fast, corrected by a new `imu_scale` node (user's choice), with a firmware fix requested in [drivelink_v2#136](https://github.com/taruntom1/drivelink_v2/issues/136) (Phase 9 to-do). Validated by replaying a rosbag, not a fresh run (user's choice): the spin result is in-sample for the scale.
- **Phase 6, IMU mount:** the chip is yawed 180° (`imu.yaw` π, measured); about 5° tilt at rest is ignored in 2D. `wheel_twist` differences encoder positions: DriveLink's velocity state integrates about 1 % off.
- **Phase 6, EKF inputs:** the wheel input is a new `wheel_twist` node (`/joint_states` → `/wheel/twist`), not `/odom`, which stops while disarmed and made the EKF diverge on the robot (user's choice). It also fuses vy (= 0), as robot_localization recommends for a diff drive; IMU yaw is `imu0_relative` (odom starts at yaw 0). The EKF runs in every mode, and `imu:=false` drops `imu0`. `riggu-base` now starts with the IMU on.
- **Phase 5, pad recovery:** the arm combo calls `/drivelink_diagnostics/recover` first when the hardware isn't active (user's choice), so a link loss is recoverable without a laptop. Seen once: link loss 52 s after a boot ("no fresh sample", no USB disconnect), when Wi-Fi and chrony came up; cause open.
