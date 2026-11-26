# docker/

Jazzy + Gazebo Harmonic image for development, simulation, and CI (amd64). *Planned — Phase 1.*

| File | Purpose |
|---|---|
| `Dockerfile` | `base` (from `osrf/ros:jazzy-desktop-full`, plus rmw_zenoh, Nav2, slam_toolbox, robot_localization, twist_mux, joy, ros_gz, gz_ros2_control, DriveLink apt) → `dev` (tools, host-UID user) |
| `compose.yaml` | Services `dev` and `sim`. Workspace mounted at `/ws`, host network |

Launch via `scripts/docker_run.sh`, which detects:
- Display: WSLg (`/mnt/wslg`), X11, or Wayland.
- GPU: NVIDIA (`--gpus all`), `/dev/dri`, or `LIBGL_ALWAYS_SOFTWARE=1`.

`.devcontainer/` and CI use the same image.

**Host requirements:** Docker Engine plus nvidia-container-toolkit (for an NVIDIA GPU). On WSL2, install the engine inside the distro, not Docker Desktop.
