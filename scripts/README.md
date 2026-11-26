# scripts/

Host-side tooling. *Planned — Phase 1, except `save_map.sh` (Phase 7).*

| Script | Use |
|---|---|
| `setup_host_tools.sh [--no-gpu] [--no-gh]` | **Available.** Run with sudo on any Ubuntu dev host, including WSL2: Docker Engine + buildx/compose, NVIDIA Container Toolkit (auto-detected), `gh`. On WSL it also strips Docker Desktop's `credsStore` and context from `~/.docker/config.json` (backup kept) |
| `setup_local.sh --profile robot\|dev` | Native Ubuntu 24.04 install. `robot` (Pi): ros-base, DriveLink agent/cli/bringup, lidar, Nav2, zenoh router unit, udev, Bluetooth, chrony, avahi, groups. `dev`: desktop-full, Gazebo Harmonic, Nav2 |
| `env.sh <sim\|remote\|robot>` | **Source it.** Sources Jazzy and the workspace, and selects the rmw_zenoh profile |
| `docker_run.sh [cmd…]` | Runs the dev/sim container with auto-detected display and GPU |
| `deploy.sh` | On the Pi: pull, rosdep, build, restart `riggu-base` |
| `save_map.sh <name>` | Saves the slam_toolbox map (pgm/yaml plus serialized) to `~/riggu_maps` |
| `setup_tailscale.sh [--install-only] [--hostname N]` | **Available.** Pi (Ubuntu 24.04): installs Tailscale from its apt repo and joins as `tag:robot` with routes, DNS and Tailscale SSH off. Interactive URL login as the riggu tailnet admin |
| `windows/riggu_portproxy.ps1 [-Remove]` | **Available.** Windows, run as admin. Resolves `riggu.local` and adds portproxy rules `127.0.0.1:2222 → Pi:22` and `127.0.0.1:17447 → Pi:7447`, because WSL mirrored mode can't reach the hotspot subnet. Offline fallback (SSH alias `riggu-local`). Re-run if the Pi's IP changes |

All scripts are idempotent and support `--help`. Remote Pi: SSH host `RIGGU_HOST` (default `riggu.local`, mapped by `~/.ssh/config` to the Pi's tailnet name), zenoh `RIGGU_ZENOH_ENDPOINT` (default derived from that `HostName`, port 7447).
