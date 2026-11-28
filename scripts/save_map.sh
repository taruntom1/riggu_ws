#!/usr/bin/env bash
# Save the running slam_toolbox map: <dir>/<name>.yaml + .pgm (for AMCL, via
# nav2 map_saver_cli) and <dir>/<name>.posegraph + .data (slam_toolbox's pose
# graph, to continue mapping later). Needs a sourced ROS environment that sees
# slam_toolbox.
#
# Run it on the machine that runs slam_toolbox (the Pi for the real robot):
# slam_toolbox writes the pose graph on its own filesystem.
set -euo pipefail

usage() {
  cat <<EOF
Usage: $0 [--dir DIR] [--force] <name>

  --dir DIR  output directory (default \${RIGGU_MAP_DIR:-\$HOME/riggu_maps})
  --force    overwrite an existing map of that name

Then: ros2 launch riggu_navigation localization.launch.py map:=<dir>/<name>.yaml
EOF
}

DIR=${RIGGU_MAP_DIR:-$HOME/riggu_maps}
FORCE=0
NAME=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dir) DIR=${2:?}; shift 2 ;;
    --force) FORCE=1; shift ;;
    -h | --help) usage; exit 0 ;;
    -*) usage >&2; exit 2 ;;
    *) [[ -z $NAME ]] || { usage >&2; exit 2; }; NAME=$1; shift ;;
  esac
done
[[ -n $NAME ]] || { usage >&2; exit 2; }
[[ $NAME =~ ^[A-Za-z0-9_-]+$ ]] || { echo "error: name must be [A-Za-z0-9_-]" >&2; exit 2; }
command -v ros2 >/dev/null || { echo "error: no ros2; source scripts/env.sh first" >&2; exit 1; }

mkdir -p "$DIR"
DIR=$(cd "$DIR" && pwd)
BASE="$DIR/$NAME"
if [[ $FORCE == 0 && -e $BASE.yaml ]]; then
  echo "error: $BASE.yaml exists (--force to overwrite)" >&2
  exit 1
fi

echo "==> map: $BASE.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "$BASE" --ros-args \
  -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 \
  -p free_thresh_default:=0.25 -p occupied_thresh_default:=0.65

echo "==> pose graph: $BASE.posegraph/.data"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$BASE'}" | tail -n 1

echo "localize: ros2 launch riggu_navigation localization.launch.py map:=$BASE.yaml"
