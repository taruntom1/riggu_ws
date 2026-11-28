# riggu_ws

ROS 2 **Jazzy** workspace for *riggu*, a differential-drive robot.

| Area | Components |
|---|---|
| Compute | Raspberry Pi 5, Ubuntu 24.04 |
| Motors and IMU | DriveLink ESP32-S3 board over USB, via ros2_control; BNO055 IMU |
| Lidar | RPLIDAR C1 |
| Middleware | rmw_zenoh |
| Simulator | Gazebo Harmonic |

## Read first
- `docs/STATUS.md`: current progress. Update it when you finish work.
- `docs/IMPLEMENTATION_PLAN.md`: phases, conventions, and the "Done when" checks.
- `docs/REQUIREMENTS.md`: agreed decisions. Don't change them without asking the user.
- To run a phase, use `/implement-phase <n>`.

## Environment
- The dev host is WSL2 Ubuntu 26.04 with ROS **Lyrical** in `/opt/ros/lyrical`. Never source it. Build and run inside the Jazzy container: `scripts/docker_run.sh`, or the devcontainer.
- The robot is at `ssh ${RIGGU_HOST:-riggu.local}` (user `riggu`). The Pi builds natively; deploy with `scripts/deploy.sh`.
- `riggu.local` reaches the Pi over **Tailscale**: a dedicated riggu tailnet, shared to users, mapped in `~/.ssh/config`. The Pi can't connect back to user machines, so design every robot↔laptop link with the laptop connecting to the Pi.
- Without internet, use `ssh riggu-local`, a Windows hotspot port-forward (`127.0.0.1:2222`; zenoh on `127.0.0.1:17447`; DriveLink agent on `127.0.0.1:47620`). Re-run `scripts/windows/riggu_portproxy.ps1` as admin if it stops working.
- Never commit tailnet names, account identities, or auth keys. The repo is public.
- `source scripts/env.sh sim|remote|robot` selects the rmw_zenoh profile. Container shells source `sim` automatically, both interactive and `bash -lc` (`docker_run.sh <cmd>`, CI).
- Git worktrees work in the container: `docker_run.sh` also mounts the main repo's git dir.

## Rules
- **One phase per branch:** `phase/<n>-<slug>`, then a PR into `main`. Never merge, and never push to `main`.
- **Sim and real share configs.** They differ only through `mode:=real|sim|mock` and `use_sim_time`.
- **Frames follow REP-105.** Topic names and the cmd_vel chain are in the plan's "Cross-cutting conventions".
- **Real-robot safety:** ask the user before anything that arms DriveLink or moves the robot. The first motion of any new config runs with the wheels off the ground.
- **Old workspaces are off limits:** never read, copy from, or run anything in older riggu workspaces (`~/riggu_ws_v2` on the Pi, `riggu_ws_old`). The robot was overhauled; their values are stale. Ask the user for physical values.
- **No secrets in the repo:** never commit DriveLink tokens (`/etc/drivelink/token`) or keys.
- **Ambiguity:** ask the user, with options and a recommendation.
- **Docs:** READMEs are terse and written for experts. Update the module `README.md` and `CLAUDE.md` when behaviour changes.

## Commands (inside the container)
```bash
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

## Layout
- `src/`: ROS packages. See `src/CLAUDE.md`.
- `docker/`: image and compose.
- `scripts/`: setup, deploy, and env scripts.
- `config/`: udev rules and the Tailscale policy. Zenoh profiles live in `scripts/env.sh` as overrides.
- `riggu.repos`: source deps, imported into `src/external/` (git-ignored).
- `docs/`: requirements, plan, status.
