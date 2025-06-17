#!/bin/bash

# This script will run zenoh_entrypoint.sh on container start if it exists and is executable
if [ -x /riggu_ws/.devcontainer/zenoh_entrypoint.sh ]; then
    /riggu_ws/.devcontainer/zenoh_entrypoint.sh || echo "zenoh_entrypoint.sh failed"
else
    echo "/riggu_ws/.devcontainer/zenoh_entrypoint.sh not found or not executable"
fi
