#!/usr/bin/env bash
# Native Ubuntu 24.04 install for riggu. Idempotent; run as your normal user
# (it uses sudo).
#   robot (Pi 5): ros-base + robot stack, DriveLink agent/cli/bringup, Tailscale,
#                 udev, Bluetooth/joystick, chrony, avahi, zenoh router unit, groups
#   dev:          desktop-full + Gazebo Harmonic + the same ROS stack
set -euo pipefail

usage() {
  cat <<EOF
Usage: $0 --profile robot|dev [--no-tailscale]

  --profile P     robot (Pi, no Gazebo) or dev (desktop-full + Gazebo Harmonic)
  --no-tailscale  robot: skip installing Tailscale
EOF
}

PROFILE=""
TAILSCALE=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile) PROFILE=${2:?}; shift 2 ;;
    --no-tailscale) TAILSCALE=0; shift ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ $PROFILE == robot || $PROFILE == dev ]] || { usage >&2; exit 2; }

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
has_systemd() { [[ -d /run/systemd/system ]]; }

# shellcheck source=/dev/null
. /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 24.04 ]] || die "Ubuntu 24.04 only (found ${PRETTY_NAME:-unknown})"
[[ $EUID -ne 0 ]] || die "run as your normal user, not root (sudo is used where needed)"
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ARCH=$(dpkg --print-architecture)
export DEBIAN_FRONTEND=noninteractive
APT=(sudo -E apt-get install -y --no-install-recommends)

DRIVELINK_KEY_URL=https://taruntom1.github.io/drivelink_v2/drivelink-archive-keyring.asc
DRIVELINK_KEY_FPR=0B7ABAC1B5A995FB3512E0686964AFD8333BB3D6

# ROS packages both profiles need. Keep in sync with docker/Dockerfile.
ROS_COMMON=(
  ros-jazzy-rmw-zenoh-cpp ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-localization
  ros-jazzy-twist-mux ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard
  ros-jazzy-rplidar-ros ros-jazzy-drivelink-bringup
)

# --- apt sources -----------------------------------------------------------------
log "Base packages and apt sources"
sudo apt-get update
"${APT[@]}" ca-certificates curl gnupg software-properties-common
if ! grep -rqs '^deb.*universe\|^Components:.*universe' /etc/apt/sources.list /etc/apt/sources.list.d/; then
  sudo add-apt-repository -y universe
fi

if ! dpkg -s ros2-apt-source &>/dev/null; then
  ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p')
  [[ -n $ver ]] || die "could not look up the ros2-apt-source release"
  tmp=$(mktemp -d)
  curl -fsSLo "$tmp/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"
  sudo apt-get install -y "$tmp/ros2-apt-source.deb"
  rm -rf "$tmp"
fi

if [[ ! -f /etc/apt/sources.list.d/drivelink.sources ]]; then
  tmp=$(mktemp)
  curl -fsSLo "$tmp" "$DRIVELINK_KEY_URL"
  gpg --show-keys --with-colons "$tmp" | awk -F: '$1=="fpr"{print $10}' | grep -qx "$DRIVELINK_KEY_FPR" ||
    die "DriveLink key fingerprint mismatch"
  sudo install -d -m 0755 /etc/apt/keyrings
  sudo install -m 0644 "$tmp" /etc/apt/keyrings/drivelink-archive-keyring.asc
  rm -f "$tmp"
  printf 'Types: deb\nURIs: https://taruntom1.github.io/drivelink_v2/\nSuites: %s\nComponents: main\nSigned-By: /etc/apt/keyrings/drivelink-archive-keyring.asc\n' \
    "$VERSION_CODENAME" | sudo tee /etc/apt/sources.list.d/drivelink.sources >/dev/null
fi
sudo apt-get update

# --- packages ----------------------------------------------------------------------
log "ROS 2 Jazzy ($PROFILE, $ARCH)"
"${APT[@]}" ros-dev-tools "${ROS_COMMON[@]}"
if [[ $PROFILE == robot ]]; then
  "${APT[@]}" ros-jazzy-ros-base drivelink-agent drivelink-cli \
    bluez joystick chrony avahi-daemon libnss-mdns
else
  "${APT[@]}" ros-jazzy-desktop-full ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
fi

log "rosdep"
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update --rosdistro jazzy
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
set -u
mapfile -t pkgs < <(find "$WS/src" -mindepth 2 -maxdepth 2 -name package.xml -printf '%h\n' | sort)
if [[ $PROFILE == robot ]]; then
  mapfile -t pkgs < <(printf '%s\n' "${pkgs[@]}" | grep -v '/riggu_gazebo$')
fi
rosdep install --from-paths "${pkgs[@]}" --ignore-src --rosdistro jazzy -y

# --- robot-only system setup ---------------------------------------------------
if [[ $PROFILE == robot ]]; then
  if [[ $TAILSCALE == 1 ]]; then
    log "Tailscale"
    "$WS/scripts/setup_tailscale.sh" --install-only
  fi

  log "udev rules"
  sudo install -m 0644 "$WS/config/udev/99-riggu.rules" /etc/udev/rules.d/99-riggu.rules
  if command -v udevadm &>/dev/null && has_systemd; then
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=tty
  fi

  log "Groups"
  for g in dialout drivelink input bluetooth; do
    if getent group "$g" >/dev/null && ! id -nG "$USER" | tr ' ' '\n' | grep -qx "$g"; then
      sudo usermod -aG "$g" "$USER"
      echo "added $USER to $g (log out and back in)"
    fi
  done

  log "systemd units"
  for unit in "$WS"/src/riggu_bringup/systemd/*.service; do
    sed -e "s|@USER@|$USER|g" -e "s|@WS@|$WS|g" "$unit" |
      sudo tee "/etc/systemd/system/$(basename "$unit")" >/dev/null
  done
  if has_systemd; then
    sudo systemctl daemon-reload
    sudo systemctl enable --now rmw-zenohd.service chrony avahi-daemon bluetooth
  else
    echo "systemd not running; units installed but not enabled"
  fi
fi

log "Done ($PROFILE). Next: source scripts/env.sh $([[ $PROFILE == robot ]] && echo robot || echo sim)"
