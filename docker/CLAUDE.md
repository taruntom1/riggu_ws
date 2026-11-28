# docker/

- **Image:** amd64 only (dev and sim), based on `osrf/ros:jazzy-desktop-full`. The Pi runs natively, not in Docker.
- **Stages:**
  - `base`: ROS deps plus the DriveLink apt repo.
  - `dev`: tools and a non-root user matching the host UID.
- **One image for three uses:** `scripts/docker_run.sh`, `.devcontainer/`, and CI all use this image. Change them together.
- **Display and GPU detection** lives in `scripts/docker_run.sh`. Compose holds only static settings:
  - Display: WSLg, X11, or Wayland.
  - GPU: NVIDIA, `/dev/dri`, or software rendering.
- **Engine:** Docker Engine inside WSL with nvidia-container-toolkit, not Docker Desktop.
- **Shell profile:** `~/.bashrc` (interactive) and `~/.profile` (non-interactive login, i.e. `bash -lc`) both source `scripts/env.sh ${RIGGU_PROFILE:-sim}`, so every command runs on rmw_zenoh. Never let a command fall back to FastDDS: with host network and IPC it picks up other containers and stale `/dev/shm` segments.
