#!/usr/bin/env bash
# Deploy the workspace on the Pi: refuse a dirty tree, pull, rosdep, build,
# restart riggu-base (when installed). Run it on the Pi, or with --remote from a
# dev machine to run it on $RIGGU_HOST over SSH. Idempotent.
set -euo pipefail

usage() {
  cat <<EOF
Usage: $0 [--remote] [--ws PATH] [--no-pull] [--no-restart]

  --remote      run on \$RIGGU_HOST (default riggu.local) over SSH instead of locally
  --ws PATH     workspace on the Pi (default: this checkout locally, ~/riggu_ws remotely)
  --no-pull     build the current checkout without 'git pull'
  --no-restart  don't restart riggu-base.service

riggu_gazebo is skipped: the robot has no Gazebo.
EOF
}

REMOTE=0
WS=""
PULL=1
RESTART=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --remote) REMOTE=1; shift ;;
    --ws) WS=${2:?}; shift 2 ;;
    --no-pull) PULL=0; shift ;;
    --no-restart) RESTART=0; shift ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

if [[ $REMOTE == 1 ]]; then
  host=${RIGGU_HOST:-riggu.local}
  args=()
  [[ $PULL == 0 ]] && args+=(--no-pull)
  [[ $RESTART == 0 ]] && args+=(--no-restart)
  remote_ws=riggu_ws # relative to the SSH user's home
  [[ -n $WS ]] && remote_ws=$(printf '%q' "$WS")
  exec ssh -t "$host" "cd $remote_ws && scripts/deploy.sh ${args[*]:-}"
fi

# shellcheck source=/dev/null
. /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 24.04 ]] || die "Ubuntu 24.04 only"
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
cd "$WS"

if [[ $PULL == 1 ]]; then
  [[ -z $(git status --porcelain) ]] || die "working tree is dirty; commit or discard first"
  log "git pull"
  git pull --ff-only
fi

# Build against Jazzy only. A shell that already sourced another workspace (e.g. from
# ~/.bashrc) would be chained into install/setup.sh, and riggu-base would load its packages.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH \
  ROS_PACKAGE_PATH
# ROS setup scripts reference unset variables.
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
set -u

log "vcs import (riggu.repos)"
mkdir -p src/external
vcs import --input riggu.repos src/external

log "rosdep install"
mapfile -t pkgs < <(colcon list --base-paths src --paths-only --packages-skip riggu_gazebo)
# drivelink_bringup comes from the DriveLink apt repo (setup_local.sh), not rosdistro.
rosdep install --from-paths "${pkgs[@]}" --ignore-src --rosdistro jazzy -y --skip-keys drivelink_bringup

log "colcon build"
colcon build --symlink-install --packages-skip riggu_gazebo

if [[ $RESTART == 1 ]] && systemctl list-unit-files riggu-base.service &>/dev/null \
  && systemctl list-unit-files riggu-base.service | grep -q riggu-base; then
  log "restart riggu-base"
  sudo systemctl restart riggu-base.service
fi
log "done"
