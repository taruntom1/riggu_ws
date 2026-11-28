# Status

Updated 2026-11-28. Next phase: **2** (in review). Run a phase with `/implement-phase <n>`.

| # | Phase | State | Branch / PR |
|---|---|---|---|
| 0 | Host tooling and repo | done | `phase/0-host-tooling` · [PR #1](https://github.com/taruntom1/riggu_ws/pull/1) |
| 1 | Workspace, Docker, setup scripts, CI | done | `phase/1-workspace` · [PR #2](https://github.com/taruntom1/riggu_ws/pull/2) |
| 2 | Robot description (URDF) | review | `phase/2-description` · PR_LINK |
| 3 | Simulated drive | todo | — |
| 4 | Real base bring-up | todo | — |
| 5 | Joystick, twist_mux, safety | todo | — |
| 6 | EKF fusion | todo | — |
| 7 | Mapping and localisation | todo | — |
| 8 | Navigation | todo | — |
| 9 | Hardening and docs | todo | — |

States: `todo` · `in progress` · `review` (PR open) · `done` (merged) · `blocked`

## Needs from user
- **Phase 2:** look at the model (`ros2 launch riggu_description display.launch.py`), then review and merge PR_REF.
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
