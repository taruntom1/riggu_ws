#!/bin/bash

set -euo pipefail  # Exit on error, undefined variables, and pipe failures

# Configuration
ZENOH_SESSION_NAME="${ZENOH_SESSION_NAME:-zenoh_router}"
ZENOH_ENDPOINT="${ZENOH_ENDPOINT:-tcp/100.107.192.97:7447}"
ROS_SETUP_SCRIPT="${ROS_SETUP_SCRIPT:-/opt/ros/kilted/setup.bash}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[Zenoh Router]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[Zenoh Router]${NC} $1"
}

log_error() {
    echo -e "${RED}[Zenoh Router]${NC} $1" >&2
}

# Check if tmux is available
if ! command -v tmux &> /dev/null; then
    log_error "tmux is not installed or not in PATH"
    exit 1
fi

# Check if ROS setup script exists
if [[ ! -f "$ROS_SETUP_SCRIPT" ]]; then
    log_error "ROS setup script not found: $ROS_SETUP_SCRIPT"
    exit 1
fi

# Check if session already exists
if tmux has-session -t "$ZENOH_SESSION_NAME" 2>/dev/null; then
    log_warn "Session '$ZENOH_SESSION_NAME' already exists"
    read -p "Do you want to kill the existing session and start a new one? [y/N]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "Killing existing session..."
        tmux kill-session -t "$ZENOH_SESSION_NAME"
    else
        log_info "Exiting without changes"
        exit 0
    fi
fi

log_info "Starting Zenoh router with endpoint: $ZENOH_ENDPOINT"

# Create the tmux session with proper command structure
tmux new-session -d -s "$ZENOH_SESSION_NAME" \
    "source '$ROS_SETUP_SCRIPT' && \
     export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"$ZENOH_ENDPOINT\"]' && \
     log_info() { echo -e '\033[0;32m[Zenoh Router]\033[0m' \$1; }; \
     log_info 'Zenoh router started in tmux session: $ZENOH_SESSION_NAME' && \
     ros2 run rmw_zenoh_cpp rmw_zenohd"

# Verify session was created successfully
if tmux has-session -t "$ZENOH_SESSION_NAME" 2>/dev/null; then
    log_info "Zenoh router started successfully in tmux session: $ZENOH_SESSION_NAME"
    log_info "To attach to the session: tmux attach-session -t $ZENOH_SESSION_NAME"
    log_info "To kill the session: tmux kill-session -t $ZENOH_SESSION_NAME"
else
    log_error "Failed to start Zenoh router session"
    exit 1
fi
