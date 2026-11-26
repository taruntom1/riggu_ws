#!/usr/bin/env bash
# Install Tailscale on the robot (Ubuntu 24.04) and join the dedicated riggu
# tailnet as tag:robot. Login is interactive: open the printed URL while signed
# in to the riggu tailnet's admin account (not your personal tailnet).
# Policy: config/tailscale/riggu-policy.hujson. Idempotent.
set -euo pipefail

usage() {
  cat <<EOF
Usage: $0 [--install-only] [--hostname NAME]

  --install-only   install the package and start tailscaled, don't log in
  --hostname NAME  tailnet machine name (default: riggu)
EOF
}

INSTALL_ONLY=0
TS_HOSTNAME=riggu
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-only) INSTALL_ONLY=1; shift ;;
    --hostname) TS_HOSTNAME=${2:?}; shift 2 ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

# shellcheck source=/dev/null
. /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 24.04 ]] || { echo "error: Ubuntu 24.04 only" >&2; exit 1; }
SUDO=""
[[ $EUID -eq 0 ]] || SUDO="sudo"

if ! command -v tailscale &>/dev/null; then
  curl -fsSL "https://pkgs.tailscale.com/stable/ubuntu/${VERSION_CODENAME}.noarmor.gpg" |
    $SUDO tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
  curl -fsSL "https://pkgs.tailscale.com/stable/ubuntu/${VERSION_CODENAME}.tailscale-keyring.list" |
    $SUDO tee /etc/apt/sources.list.d/tailscale.list >/dev/null
  $SUDO apt-get update
  $SUDO apt-get install -y tailscale
fi
$SUDO systemctl enable --now tailscaled

if [[ $INSTALL_ONLY == 1 ]]; then
  tailscale version | head -1
  exit 0
fi

# --advertise-tags makes the node tag-owned (no user owner, no key expiry).
# --accept-routes/--accept-dns off: the robot doesn't need anything from the tailnet.
$SUDO tailscale up \
  --hostname="$TS_HOSTNAME" \
  --advertise-tags=tag:robot \
  --accept-routes=false \
  --accept-dns=false \
  --ssh=false
tailscale status
