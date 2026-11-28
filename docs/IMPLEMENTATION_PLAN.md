# riggu — Implementation Plan

Companion to [REQUIREMENTS.md](REQUIREMENTS.md). Each phase is one feature branch and one PR into `main`, merged after CI is green and the acceptance checks pass. Run a phase with `/implement-phase <n>`; progress lives in [STATUS.md](STATUS.md).

## Working agreements

| Topic | Decision |
|---|---|
| Repo | Public GitHub `taruntom1/riggu_ws`; local `master` renamed to `main` |
| Git flow | `phase/<n>-<name>` branch → PR → CI green → you merge |
| Dev environment | Docker Engine inside WSL (docker-ce + nvidia-container-toolkit), RTX 4050 via WSLg; VS Code devcontainer on the same image |
| Pi deploy | Push → `scripts/deploy.sh` on the Pi (pull, rosdep, colcon build, restart services) |
| Pi access | Key-based SSH from this WSL machine to `riggu.local` |
| Pi address | **Primary: Tailscale.** The Pi (hostname and user `riggu`) is the only machine in a dedicated riggu tailnet, tagged `tag:robot`. It is *shared* to each person, and shared machines can't initiate connections into the recipient's tailnet. The policy is `config/tailscale/riggu-policy.hujson`. Each dev's `~/.ssh/config` maps `riggu.local` to the Pi's riggu-tailnet MagicDNS name. **Fallback (no internet):** the Pi is on the laptop's Windows hotspot; `scripts/windows/riggu_portproxy.ps1` forwards `127.0.0.1:2222 → Pi:22`, `127.0.0.1:17447 → Pi:7447` and `127.0.0.1:47620 → Pi:47620`; SSH alias `riggu-local`. Env: `RIGGU_HOST` (SSH host, default `riggu.local`), `RIGGU_ZENOH_ENDPOINT` (default `tcp/<HostName of RIGGU_HOST from ssh -G>:7447`) |
| Order | Interleaved: every capability is brought up in sim, then on the real robot, before moving on |

## Cross-cutting conventions

- **Frames (REP-105):** `map → odom → base_footprint → base_link → {left_wheel, right_wheel, caster*, laser, imu_link}`. `odom → base_footprint` belongs to the EKF; `map → odom` to slam_toolbox or AMCL.
- **Topics:** no namespace (single robot). `/scan`, `/imu`, `/joint_states`, `/odom` (raw wheel, armed only), `/wheel/twist` (EKF wheel input), `/odometry/filtered` (EKF), `/cmd_vel_joy`, `/cmd_vel_nav`, `/cmd_vel_mux`, `/cmd_vel_safe` → controller.
- **One config for sim and real:** shared `controllers.yaml`, EKF, and Nav2 params; only the ros2_control hardware block and `use_sim_time` differ. Launch arg `mode:=real|sim|mock`.
- **Zenoh profiles:** rmw_zenoh's default configs plus `ZENOH_CONFIG_OVERRIDE`, set by `scripts/env.sh` (no json5 files; see `config/README.md`):
  - `robot` (Pi): router listens on `tcp/[::]:7447`; local sessions connect to it.
  - `remote` (laptop to robot): **client-mode** session connecting to `${RIGGU_ZENOH_ENDPOINT}`. Default: the tailnet name from `ssh -G $RIGGU_HOST`, port 7447; the offline fallback is `tcp/127.0.0.1:17447`. Client mode routes everything through the Pi router, and the Pi never has to connect back to the laptop, which the Tailscale share forbids.
  - `sim` (container): local router only, isolated from the robot.
  - Selected by `source scripts/env.sh <profile>`, which sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and the `ZENOH_*_CONFIG_URI` variables.
- **DriveLink:** reuse the installed `drivelink.ros2_control.xacro` macro rather than copying the plugin block, so plugin names stay owned by DriveLink. Agent config `/etc/drivelink/agent.json`, device alias `base`, USB endpoint `serial:///dev/drivelink/<serial>` (from `drivelink-agent --list-ports`). ROS user in the `drivelink` group for the token.
- **Languages:** config-only packages use `ament_cmake`; the few small custom nodes (stop latch, waypoint runner) are Python.

---

## Phase 0 — Host tooling and repo (no ROS yet)

Tasks
- Rename `master` → `main`; add `.gitignore`, `README.md`, `LICENSE` (Apache-2.0 unless you choose otherwise).
- **On this WSL machine:** run `sudo scripts/setup_host_tools.sh`. It removes the broken Docker Desktop context and installs docker-ce, the compose plugin, nvidia-container-toolkit, and `gh`.
- **You:** create the public GitHub repo and `gh auth login`. I add `origin` and push.
- **You:** generate an SSH key here and copy it to the Pi; set the Pi's hostname to `riggu`.

Done when: `docker run --gpus all … nvidia-smi` works in WSL, `git push` works, and `ssh riggu.local` works.

## Phase 1 — Workspace skeleton, Docker, setup scripts, CI

Tasks
- `src/` packages with `package.xml` and build files only: `riggu_description`, `riggu_bringup`, `riggu_gazebo`, `riggu_navigation`, `riggu_teleop`.
- `riggu.repos` for source deps, imported into `src/external/` (git-ignored). Apt `rplidar_ros` 2.1.0 lacks C1 support, so it pins Slamtec `sllidar_ros2`.
- `docker/Dockerfile` (amd64) from `osrf/ros:jazzy-desktop-full`. Stages:
  - `base`: rmw_zenoh, Nav2, slam_toolbox, robot_localization, twist_mux, joy, teleop_twist_joy, ros_gz, gz_ros2_control, DriveLink apt repo + `ros-jazzy-drivelink-bringup`.
  - `dev`: tools, non-root user matching host UID.
- `docker/compose.yaml` + `scripts/docker_run.sh`:
  - Display: WSLg (`/mnt/wslg`, `WAYLAND_DISPLAY`), X11, or Wayland.
  - GPU: NVIDIA (`--gpus all`), `/dev/dri`, or `LIBGL_ALWAYS_SOFTWARE`.
  - Mounts the workspace.
- `.devcontainer/devcontainer.json` using the same image and run args.
- `scripts/setup_local.sh --profile robot|dev`. Idempotent; installs:
  - Both profiles: ROS apt, Jazzy, rosdep.
  - `robot`: ros-base, DriveLink agent/cli/bringup, Tailscale (`setup_tailscale.sh --install-only`), udev rules (`/dev/rplidar`), Bluetooth + joystick packages, chrony, avahi, zenoh router systemd unit, groups (`dialout`, `drivelink`, `input`).
  - `dev`: desktop-full, Gazebo Harmonic.
- `scripts/env.sh`, `scripts/deploy.sh`.
- `.github/workflows/ci.yml`: build image (cached), `colcon build`, `colcon test`, ament lint.

Done when: `scripts/docker_run.sh` opens a GPU-accelerated RViz on WSLg, `colcon build` is clean in the container, and CI is green on the PR.

## Phase 2 — Robot description (URDF)

Tasks
- `riggu_description/config/robot.yaml`: every dimension, mass, sensor pose and limit as `{value, source, note}`, measured from tape-friendly datums. `source` (`estimate`, `old_config`, `datasheet`, `user`, `measured`, `calibrated`) shows what still needs correcting.
- `riggu_description/urdf/`:
  - `riggu.urdf.xacro`, plus `properties.xacro`, which loads `robot.yaml` and derives every frame offset.
  - Inertia macros; wheels, caster, `laser`, `imu_link`.
  - Primitives.
- `ros2_control.xacro` selects the hardware by `mode`:
  - `real`: DriveLink macro with velocity command interfaces and IMU sensor.
  - `sim`: `gz_ros2_control/GazeboSimSystem` with the same joints and IMU sensor.
  - `mock`: `mock_components/GenericSystem`, for CI and desk tests.
- `display.launch.py` (robot_state_publisher, joint_state_publisher_gui, RViz).
- Test: xacro expands for all three modes and `check_urdf` passes; interfaces, frames, YAML schema and geometry fit are checked (colcon test).

Done when: the model in RViz matches the physical robot's layout (axle at the front, rear caster, lidar over the axle). The values start from the old DriveLink config plus estimates (no tape measurements yet). Checking the IMU and lidar axes against the real sensors moved to Phase 4 hardware checks 3–4.

## Phase 3 — Simulated drive

Tasks
- `riggu_gazebo`: indoor house world (hand-written primitive SDF, 10×8 m, 4 rooms), `sim.launch.py` (gz sim, spawn, `ros_gz_bridge` for `/clock` and `/scan`).
- Sensors:
  - Lidar: `gpu_lidar` matched to the C1 (10 Hz, 0.72°, 0.05–12 m, Gaussian noise).
  - IMU: through gz_ros2_control, with noise.
- `riggu_bringup/config/controllers.yaml` (shared): `joint_state_broadcaster`, `imu_sensor_broadcaster`, `diff_drive_controller` with velocity interface, limits, `cmd_vel_timeout`. Wheel radius and separation come from `riggu_description/config/robot.yaml` (set by the launch file), not a second copy.
- `riggu_bringup/launch/bringup.launch.py mode:=sim|real|mock`.
- Until Phase 6, `diff_drive_controller` publishes `odom → base_footprint` (`enable_odom_tf: true`). Until Phase 5, keyboard teleop publishes straight to `/cmd_vel_safe`; from Phase 5 to `/cmd_vel_joy`.

Done when: keyboard teleop drives the robot in Gazebo, `/scan`, `/imu`, and `/odom` are sane in RViz, and CI runs a `mock`-mode launch test.

## Phase 4 — Real base bring-up

Tasks
- Run `setup_local.sh --profile robot` on the Pi over SSH. It installs `/etc/drivelink/agent.json` from `config/drivelink/agent.json`: alias `base`, USB endpoint, bind `0.0.0.0:47620` (token-protected; the tailnet policy allows it, and the hotspot portproxy forwards it). Start the zenoh router service.
- `bringup.launch.py mode:=real` on the Pi: robot_state_publisher, `ros2_control_node` with the DriveLink plugin, spawners, lidar driver (`sllidar_ros2`) on `/dev/rplidar`, DriveLink `diagnostics` (fault ack/recover). `diff_drive_controller` is loaded **inactive**; arming is an explicit `ros2 control switch_controllers --activate`.
- Hardware checks (in this order):
  1. Wheel direction and encoder polarity, open loop in the DriveLink app first.
  2. Velocity-mode PID response.
  3. IMU axes against the URDF. *Deferred: the BNO055 isn't fitted yet; `bringup.launch.py imu:=false` until it is. Done when the IMU is fitted, before Phase 6.*
  4. Lidar orientation against the URDF.
- Calibrate wheel radius (2 m straight run) and separation (10 in-place turns) with `ros2 run riggu_bringup calibrate_wheels`; write the values back to `riggu_description/config/robot.yaml` (`source: calibrated`).
- Correct the sensor poses in `robot.yaml` from checks 3–4 (`source: measured`).
- Laptop: `source scripts/env.sh remote` → RViz over zenoh.

Done when: keyboard teleop from the laptop drives the real robot, odometry error is under about 2% on the straight-line and rotation tests, and RViz on the laptop shows the scan and TF live.

## Phase 5 — Joystick, twist_mux, safety chain

Tasks
- Pairing guide for a generic Bluetooth pad on the Pi; `config/joy_generic.yaml` for the mapping, plus a doc on finding indices with `ros2 topic echo /joy`.
- `joy` + `teleop_twist_joy` (deadman button, normal/turbo scale) → `/cmd_vel_joy`.
- `twist_mux`: joy (priority 100) > nav (10). The lock topic `/estop` (priority 255) zeroes output; it is a heartbeat, so a dead stop latch also locks.
- `riggu_teleop/stop_latch.py`: a stop button toggles `/estop` latched until it is pressed again.
- `riggu_teleop/arm_switch.py`: boot stays disarmed; holding deadman (LB) + Start for 1 s activates `diff_drive_controller` (arms), Back deactivates it.
- Chain: `twist_mux → collision_monitor → diff_drive_controller`. The collision monitor is added here so teleop is protected too: one footprint `approach` polygon from `robot.yaml` (Phase 8 tunes stop/slowdown zones). `collision_check_joy:=false` moves it onto the nav input only.
- Loss-of-link layering: joy timeout → twist_mux timeout → `cmd_vel_timeout` → DriveLink watchdog (200 ms).
- `riggu-base.service` (systemd) on the Pi: base bringup + teleop on boot, after the zenoh router and drivelink-agent.

Done when:
- The robot moves only while the deadman is held, and the joystick always beats a published `/cmd_vel_nav`.
- The stop latch holds until it is released.
- Turning the pad off stops the robot within about 0.5 s.
- The robot drives from the pad after a cold boot with no laptop.

## Phase 6 — Sensor fusion (EKF)

Tasks
- BNO055 calibration in the DriveLink app (all four levels at 3, saved), done with the robot in its final configuration.
- Set IMU covariances in the `imu_sensor_broadcaster` params.
- `robot_localization` `ekf_node` (`two_d_mode`, 30 Hz):
  - `twist0`: vx, vy (= 0, non-holonomic) and vyaw from `/wheel/twist` (`riggu_bringup/wheel_twist`, encoder position differences from `/joint_states`). Not `/odom`: `diff_drive_controller` publishes it only while armed.
  - `imu0`: vyaw from `/imu/corrected` (`riggu_bringup/imu_scale`: the BNO055 gyro reads 3.2 % fast; `robot.yaml` `imu.gyro_z_scale`).
  - `use_imu_orientation` (default `false`, chosen from the floor tests) adds the BNO055 absolute yaw.
- `diff_drive_controller` `enable_odom_tf: false` (true since Phase 3, while no EKF exists).
- Sim IMU bias and noise (added in Phase 3, values in `robot.yaml`) are checked against the tuned EKF.
- Magnetic interference check: compare IMU yaw with odometry yaw while spinning the motors in place. Record the result and choose the default.

Done when: after a 4 m square driven by joystick, the EKF ends closer to the start than raw wheel odometry, and the 360° spin error is under 3°.

## Phase 7 — Mapping and localisation

Tasks
- `riggu_navigation`:
  - `slam.launch.py` (slam_toolbox online async, 5 cm resolution, C1 range limits from `robot.yaml`).
  - `localization.launch.py` (map_server + AMCL, diff-drive motion model). Both under nav2 lifecycle managers.
- Map save helper `scripts/save_map.sh <name>`: map (pgm/yaml) plus the slam_toolbox pose graph. Maps go to `~/riggu_maps` on the Pi; the sim reference map is committed under `riggu_navigation/maps/`.
- RViz configs for mapping and navigation.

Done when: clean maps of the sim house and of a real room exist, and AMCL converges from a rough initial pose within one short drive.

## Phase 8 — Autonomous navigation

Tasks
- `nav2_params.yaml`:
  - Planner: Smac 2D. Controller: MPPI (DiffDrive, about 20 Hz, batch sized for the Pi 5).
  - Velocity smoother.
  - Costmaps from `/scan` with inflation sized to the footprint.
  - Recovery behaviours: default BT.
- Collision monitor polygons (stop, slowdown) tuned to the footprint and max speed.
- Waypoint following: the Nav2 RViz panel, plus `riggu_navigation/waypoints.py`, which runs a YAML list of poses (optionally looped).
- `navigation.launch.py` (localisation + Nav2) and an `slam:=true` variant that navigates while mapping.
- Pi profiling: CPU and latency under full stack; tune until the controller loop holds its rate.

Done when:
- Goal and waypoint runs succeed in sim and on the real robot.
- The collision monitor stops the robot for an obstacle thrown into its path.
- The joystick can override Nav2 at any moment.
- The Pi stays under about 70% CPU.

## Phase 9 — Hardening and docs

Tasks
- CI: headless Gazebo smoke test (spawn, drive, odometry moves).
- README quickstarts: Docker sim, native dev, Pi setup, daily robot use.
- Docs: tuning notes, troubleshooting (zenoh connectivity, WSL mDNS, Bluetooth re-pairing).
- **DriveLink gyro scale:** once [drivelink_v2#136](https://github.com/taruntom1/drivelink_v2/issues/136) lands, calibrate the scale on the board, drop `riggu_bringup/imu_scale` and `imu.gyro_z_scale`, and point the EKF's `imu0` back at `/imu`. Re-run the Phase 6 floor replay or square/spin to confirm.
- Tag `v1.0`.

---

## Risks and open checks

| Risk | Mitigation |
|---|---|
| `ros-jazzy-drivelink-*` arm64/amd64 availability, and the macro and plugin names | Phase 1: 0.2.1 on noble amd64 and arm64; macro `drivelink_diff_drive(name agent device token_file token left_joint left_label right_joint right_label imu_label)` in `drivelink_bringup/urdf/drivelink.ros2_control.xacro` |
| RPLIDAR C1 support in apt `rplidar_ros` | Phase 1: apt 2.1.0 has no C1 support; Slamtec `sllidar_ros2` pinned in `riggu.repos` |
| BNO055 yaw disturbed by motor magnets | Phase 6: wheels-up test stayed within 1.5°, but on the floor the heading drifted 3–6° on straights, so the default is gyro-only (`use_imu_orientation:=false`) |
| BNO055 gyro scale 3.2 % fast | `riggu_bringup/imu_scale` stop-gap; firmware fix in drivelink_v2#136 (Phase 9) |
| Robot reachability and isolation | Tailscale share from a dedicated tailnet (Pi can't reach sharees). Offline fallback: hotspot + Windows `netsh portproxy`; re-run it if the Pi's hotspot IP changes |
| Clock skew between Pi and laptop breaking RViz TF | chrony on the Pi; the laptop only visualises |
| USB drops under motor load (2026-11-28: C1 port "disabled by hub (EMI?)", DriveLink CH343 re-enumerated) | Short shielded cables / ferrites, motor power wiring away from USB; sllidar respawns and `/dev/rplidar` follows; DriveLink link loss → hardware `unconfigured`, `recover` restores it disarmed |
| Collision monitor `approach` polygon in doorways (Phase 7 sim): turning in place sweeps the footprint's rear corners (~0.35 m) into a 0.9 m door frame or a counter, and the monitor stops it | Drive straight through doors; Phase 8 sizes MPPI/costmap inflation and the monitor polygons for it |
| nav2 lifecycle bond (4 s) drops under host CPU starvation (Phase 7 sim, during a Docker Desktop start): the collision monitor shut down and `/cmd_vel_safe` stopped (fail-safe) | Watch in Phase 8 Pi profiling; `bond_timeout` only if it recurs on the Pi |
| Pi 5 CPU under SLAM + Nav2 | MPPI batch and rate tuning, costmap update rates, Phase 8 profiling |
