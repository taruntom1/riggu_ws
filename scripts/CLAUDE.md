# scripts/

- **Bash style:** `#!/usr/bin/env bash`, `set -euo pipefail`, shellcheck-clean, and `--help` on every script.
- **Setup and deploy scripts:** they must be idempotent and refuse to run on anything but Ubuntu 24.04 (check `/etc/os-release`). The exception is `setup_host_tools.sh`, which runs on any Ubuntu, because the dev host is 26.04.
- **`setup_local.sh --profile robot|dev`:**
  - `robot` (Pi): no Gazebo.
  - `dev`: desktop-full and Gazebo Harmonic.
  - It installs udev rules from `config/udev/` and systemd units from `src/riggu_bringup/systemd/`.
- **`env.sh <sim|remote|robot>`** is sourced, not executed. It sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and the zenoh config URIs from `config/zenoh/`.
- **`deploy.sh`** runs on the Pi. It refuses a dirty tree, then pulls, runs rosdep, builds, and restarts `riggu-base`.
- **Secrets:** never print, copy, or commit `/etc/drivelink/token`.
