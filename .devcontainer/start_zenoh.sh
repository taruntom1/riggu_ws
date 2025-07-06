#!/bin/bash

echo "[Zenoh Router] Starting..."

# Start zenoh router in a detached tmux session
tmux new-session -d -s zenoh_router "export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/100.107.192.97:7447\"]'; ros2 run rmw_zenoh_cpp rmw_zenohd"
