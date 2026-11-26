# scripts/

Host-side tooling. *Planned — Phase 1, except `save_map.sh` (Phase 7).*

| Script | Use |
|---|---|
| `setup_local.sh --profile robot\|dev` | Native Ubuntu 24.04 install. `robot` (Pi): ros-base, DriveLink agent/cli/bringup, lidar, Nav2, zenoh router unit, udev, Bluetooth, chrony, avahi, groups. `dev`: desktop-full, Gazebo Harmonic, Nav2 |
| `env.sh <sim\|remote\|robot>` | **Source it.** Sources Jazzy and the workspace, and selects the rmw_zenoh profile |
| `docker_run.sh [cmd…]` | Runs the dev/sim container with auto-detected display and GPU |
| `deploy.sh` | On the Pi: pull, rosdep, build, restart `riggu-base` |
| `save_map.sh <name>` | Saves the slam_toolbox map (pgm/yaml plus serialized) to `~/riggu_maps` |

All scripts are idempotent and support `--help`. Remote Pi address: `RIGGU_HOST` (default `riggu.local`).
