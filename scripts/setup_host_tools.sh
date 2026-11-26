#!/usr/bin/env bash
# Install dev-host tooling for riggu: Docker Engine (+ buildx, compose),
# NVIDIA Container Toolkit (when an NVIDIA GPU is visible) and GitHub CLI.
# On WSL2 it also detaches the Docker CLI config from Docker Desktop.
# Idempotent; safe to re-run.
set -euo pipefail

usage() {
  cat <<EOF
Usage: sudo $0 [--no-gpu] [--no-gh]

Installs Docker Engine, NVIDIA Container Toolkit and gh on Ubuntu (incl. WSL2).
  --no-gpu   skip NVIDIA Container Toolkit even if a GPU is found
  --no-gh    skip GitHub CLI
EOF
}

GPU=auto
GH=1
for arg in "$@"; do
  case "$arg" in
    --no-gpu) GPU=0 ;;
    --no-gh) GH=0 ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "run with sudo"
# shellcheck source=/dev/null
. /etc/os-release
[[ ${ID:-} == ubuntu ]] || die "Ubuntu only (found ${ID:-unknown})"
TARGET_USER=${SUDO_USER:-}
[[ -n $TARGET_USER && $TARGET_USER != root ]] || die "run via sudo from your normal user"
TARGET_HOME=$(getent passwd "$TARGET_USER" | cut -d: -f6)
CODENAME=${UBUNTU_CODENAME:-$VERSION_CODENAME}
ARCH=$(dpkg --print-architecture)
is_wsl() { grep -qi microsoft /proc/sys/kernel/osrelease; }
has_systemd() { [[ -d /run/systemd/system ]]; }

export DEBIAN_FRONTEND=noninteractive
log "Base packages"
apt-get update
apt-get install -y ca-certificates curl gnupg python3
install -d -m 0755 /etc/apt/keyrings

# --- Docker Engine -----------------------------------------------------------
log "Docker Engine ($CODENAME/$ARCH)"
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do
  if dpkg -s "$pkg" &>/dev/null; then apt-get remove -y "$pkg"; fi
done
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
cat >/etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $CODENAME
Components: stable
Architectures: $ARCH
Signed-By: /etc/apt/keyrings/docker.asc
EOF
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
usermod -aG docker "$TARGET_USER"
if has_systemd; then
  systemctl enable --now docker
else
  echo "WARNING: systemd not running; enable it (WSL: [boot] systemd=true in /etc/wsl.conf) and restart." >&2
fi

# --- NVIDIA Container Toolkit -------------------------------------------------
if [[ $GPU == auto ]]; then
  GPU=0
  for smi in nvidia-smi /usr/lib/wsl/lib/nvidia-smi; do
    if command -v "$smi" &>/dev/null && "$smi" -L &>/dev/null; then GPU=1; break; fi
  done
fi
if [[ $GPU == 1 ]]; then
  log "NVIDIA Container Toolkit"
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
    gpg --dearmor --yes -o /etc/apt/keyrings/nvidia-container-toolkit.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |
    sed 's#deb https://#deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.gpg] https://#g' \
      >/etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update
  apt-get install -y nvidia-container-toolkit
  nvidia-ctk runtime configure --runtime=docker
  if has_systemd; then systemctl restart docker; fi
else
  log "No NVIDIA GPU detected (or --no-gpu); skipping NVIDIA Container Toolkit"
fi

# --- GitHub CLI ---------------------------------------------------------------
if [[ $GH == 1 ]]; then
  log "GitHub CLI"
  curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
    -o /etc/apt/keyrings/githubcli-archive-keyring.gpg
  chmod a+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
  echo "deb [arch=$ARCH signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    >/etc/apt/sources.list.d/github-cli.list
  apt-get update
  apt-get install -y gh
fi

# --- Detach Docker CLI config from Docker Desktop (WSL) -----------------------
CFG="$TARGET_HOME/.docker/config.json"
if [[ -f $CFG ]]; then
  log "Cleaning Docker Desktop settings from $CFG"
  sudo -u "$TARGET_USER" python3 - "$CFG" <<'PY'
import json, shutil, sys, time
path = sys.argv[1]
with open(path) as f:
    cfg = json.load(f)
changed = False
if str(cfg.get("credsStore", "")).endswith(".exe"):
    cfg.pop("credsStore"); changed = True
if cfg.get("currentContext", "default") != "default":
    cfg.pop("currentContext"); changed = True
if changed:
    shutil.copy2(path, f"{path}.bak-{int(time.time())}")
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2)
    print("updated (backup saved next to it)")
else:
    print("nothing to change")
PY
fi

log "Done"
docker --version
docker compose version
if [[ $GH == 1 ]]; then gh --version | head -1; fi
cat <<EOF

Next:
  1. Open a new shell (or run 'newgrp docker') so '$TARGET_USER' is in the docker group.
  2. Check that 'command -v docker' prints /usr/bin/docker.
EOF
if is_wsl; then
  cat <<EOF
  3. WSL: quit Docker Desktop or disable its WSL integration for this distro,
     so its CLI on the Windows PATH doesn't shadow or fight this engine.
EOF
fi
if [[ $GPU == 1 ]]; then
  echo "  GPU test: docker run --rm --gpus all ubuntu nvidia-smi"
fi
