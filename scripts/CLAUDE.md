# scripts/

- **Bash style:** `#!/usr/bin/env bash`, `set -euo pipefail`, shellcheck-clean, and `--help` on every script.
- **Setup and deploy scripts:** they must be idempotent and refuse to run on anything but Ubuntu 24.04 (check `/etc/os-release`). The exception is `setup_host_tools.sh`, which runs on any Ubuntu, because the dev host is 26.04.
- **`setup_local.sh --profile robot|dev`:**
  - `robot` (Pi): no Gazebo.
  - `dev`: desktop-full and Gazebo Harmonic.
  - It installs udev rules from `config/udev/` and systemd units from `src/riggu_bringup/systemd/`.
- **`env.sh <sim|remote|robot>`** is sourced, not executed. It sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and the profile's `ZENOH_CONFIG_OVERRIDE` on top of rmw_zenoh's defaults. `sim` starts a local router; switching profile stops the ros2 daemon.
- **`deploy.sh`** runs on the Pi (or `--remote`). It refuses a dirty tree, then pulls, imports `riggu.repos`, runs rosdep, builds (skipping `riggu_gazebo`), and restarts `riggu-base`. It clears the ROS/colcon prefix variables first, so a workspace sourced by the calling shell never ends up chained into `install/setup.sh`.
- **Package lists:** the ROS apt list in `setup_local.sh` mirrors `docker/Dockerfile`. Change both together.
- **Secrets:** never print, copy, or commit `/etc/drivelink/token`.
