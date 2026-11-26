#!/usr/bin/env bash
# Source me: select the riggu rmw_zenoh profile and source Jazzy + the workspace.
#   source scripts/env.sh sim|remote|robot [--no-router]
# Profiles build on rmw_zenoh's default configs (router on tcp/[::]:7447, sessions
# connect to tcp/localhost:7447, multicast scouting off) through ZENOH_CONFIG_OVERRIDE:
#   sim     local router only; started here in the background if none is listening
#   robot   the Pi; router is rmw-zenohd.service
#   remote  client-mode session connecting to the Pi's router at $RIGGU_ZENOH_ENDPOINT
#           (default: tcp/<HostName of 'ssh -G $RIGGU_HOST'>:7447; offline: tcp/127.0.0.1:17447)
# Not 'set -e': this runs in your shell.

_riggu_env_usage() {
  cat <<'EOF'
Usage: source scripts/env.sh sim|remote|robot [--no-router]

  sim          local zenoh router (starts rmw_zenohd in the background if needed)
  remote       client session to the Pi router ($RIGGU_ZENOH_ENDPOINT, else derived
               from 'ssh -G ${RIGGU_HOST:-riggu.local}', port 7447)
  robot        sessions on the Pi, using the rmw-zenohd.service router
  --no-router  sim: don't start a router
EOF
}

_riggu_env() {
  local profile="" start_router=1 arg ws
  for arg in "$@"; do
    case "$arg" in
      sim | remote | robot) profile=$arg ;;
      --no-router) start_router=0 ;;
      -h | --help) _riggu_env_usage; return 0 ;;
      *) _riggu_env_usage >&2; return 2 ;;
    esac
  done
  [[ -n $profile ]] || { _riggu_env_usage >&2; return 2; }

  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "env.sh: /opt/ros/jazzy not found; use the Jazzy container (scripts/docker_run.sh) or the Pi" >&2
    return 1
  fi
  ws=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  if [[ -f $ws/install/setup.bash ]]; then
    # shellcheck source=/dev/null
    source "$ws/install/setup.bash"
  fi

  local previous="${RMW_IMPLEMENTATION:-}|${ZENOH_CONFIG_OVERRIDE:-}"
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export RIGGU_PROFILE=$profile
  unset ZENOH_SESSION_CONFIG_URI ZENOH_ROUTER_CONFIG_URI ZENOH_CONFIG_OVERRIDE

  case "$profile" in
    sim)
      if [[ $start_router == 1 ]] && ! ss -Hltn 'sport = :7447' 2>/dev/null | grep -q .; then
        nohup ros2 run rmw_zenoh_cpp rmw_zenohd >"${TMPDIR:-/tmp}/riggu_rmw_zenohd.log" 2>&1 &
        disown
        echo "env.sh: started local zenoh router (log: ${TMPDIR:-/tmp}/riggu_rmw_zenohd.log)"
      fi ;;
    robot)
      if command -v systemctl &>/dev/null && ! systemctl is-active --quiet rmw-zenohd.service; then
        echo "env.sh: warning: rmw-zenohd.service is not active" >&2
      fi ;;
    remote)
      local endpoint=${RIGGU_ZENOH_ENDPOINT:-} host=${RIGGU_HOST:-riggu.local} name=""
      if [[ -z $endpoint ]]; then
        if command -v ssh &>/dev/null; then
          name=$(ssh -G "$host" 2>/dev/null | awk '$1=="hostname"{print $2; exit}')
        fi
        endpoint="tcp/${name:-$host}:7447"
      fi
      export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"$endpoint\"]"
      echo "env.sh: remote zenoh client -> $endpoint" ;;
  esac

  # A running ros2 daemon keeps its old middleware settings; restart it on change.
  if [[ $previous != "rmw_zenoh_cpp|${ZENOH_CONFIG_OVERRIDE:-}" && $previous != "|" ]]; then
    ros2 daemon stop &>/dev/null || true
  fi
}

if ! (return 0 2>/dev/null); then
  echo "env.sh must be sourced: source ${BASH_SOURCE[0]} sim|remote|robot" >&2
  exit 2
fi
_riggu_env "$@"
