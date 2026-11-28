# scripts/

Host-side tooling. `save_map.sh` is planned for Phase 7.

| Script | Use |
|---|---|
| `setup_host_tools.sh [--no-gpu] [--no-gh]` | Run with sudo on any Ubuntu dev host, including WSL2: Docker Engine + buildx/compose, NVIDIA Container Toolkit (auto-detected), `gh`. On WSL it also strips Docker Desktop's `credsStore` and context from `~/.docker/config.json` (backup kept) |
| `setup_local.sh --profile robot\|dev [--no-tailscale] [--drivelink-port P]` | Native Ubuntu 24.04 install, as your user (uses sudo). Both: ROS apt (`ros2-apt-source`), DriveLink apt (key pinned), Jazzy stack, rosdep for `src/`. `robot` (Pi): ros-base, DriveLink agent/cli, Tailscale (`--install-only`), `config/udev/`, `/etc/drivelink/agent.json` from `config/drivelink/` (port: `--drivelink-port` or the only `/dev/drivelink/*`; restarts the agent only on change), Bluetooth/joystick, chrony, avahi, groups (`dialout`, `drivelink`, `input`, `bluetooth`), units from `src/riggu_bringup/systemd/` (enables `rmw-zenohd`, `drivelink-agent`). `dev`: desktop-full, Gazebo Harmonic |
| `env.sh <sim\|remote\|robot> [--no-router]` | **Source it.** Sources Jazzy and the workspace, sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and the profile's `ZENOH_CONFIG_OVERRIDE` (see `config/README.md`). `sim` starts a local `rmw_zenohd` if none is listening |
| `docker_run.sh [--build] [--display M] [--gpu M] [--write-override] [cmd…]` | Runs the dev/sim container with auto-detected display and GPU (see `docker/README.md`) |
| `deploy.sh [--remote] [--ws P] [--no-pull] [--no-restart]` | On the Pi (or `--remote` over SSH, workspace `~/riggu_ws`): refuse a dirty tree, `git pull --ff-only`, rosdep, `colcon build` (skips `riggu_gazebo`), restart `riggu-base` if installed |
| `save_map.sh <name>` | Saves the slam_toolbox map (pgm/yaml plus serialized) to `~/riggu_maps` |
| `setup_tailscale.sh [--install-only] [--hostname N]` | Pi (Ubuntu 24.04): installs Tailscale from its apt repo and joins as `tag:robot` with routes, DNS and Tailscale SSH off. Interactive URL login as the riggu tailnet admin |
| `windows/riggu_portproxy.ps1 [-Remove]` | Windows, run as admin. Resolves `riggu.local` and adds portproxy rules `127.0.0.1:2222 → Pi:22` and `127.0.0.1:17447 → Pi:7447` and `127.0.0.1:47620 → Pi:47620` (DriveLink app), because WSL mirrored mode can't reach the hotspot subnet. Offline fallback (SSH alias `riggu-local`). Re-run if the Pi's IP changes |

All scripts are idempotent and support `--help`. Remote Pi: SSH host `RIGGU_HOST` (default `riggu.local`, mapped by `~/.ssh/config` to the Pi's tailnet name), zenoh `RIGGU_ZENOH_ENDPOINT` (default derived from that `HostName`, port 7447).
