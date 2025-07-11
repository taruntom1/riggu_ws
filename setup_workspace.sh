#!/bin/bash

# Riggu Workspace Setup Script
# This script helps new contributors set up the workspace with all submodules

set -e  # Exit on any error

echo "🚀 Setting up Riggu ROS2 Workspace..."

# Check if we're in the right directory
if [ ! -f ".gitmodules" ]; then
    echo "❌ Error: This script should be run from the root of the riggu_ws repository"
    echo "   Make sure you're in the directory containing .gitmodules"
    exit 1
fi

# Initialize and update all submodules
echo "📦 Initializing and updating git submodules..."
git submodule update --init --recursive

# Check submodule status
echo "📋 Checking submodule status..."
git submodule status

# Source ROS2 and build the workspace
echo "🔨 Building the workspace..."
if [ -f "/opt/ros/kilted/setup.bash" ]; then
    source /opt/ros/kilted/setup.bash
    
    # Check for missing dependencies
    echo "🔍 Checking for missing dependencies..."
    rosdep check --from-paths src --ignore-src -r || {
        echo "⚠️  Some dependencies are missing. Installing them..."
        rosdep install --from-paths src --ignore-src -r -y
    }
    
    # Build the workspace
    echo "⚙️  Building packages..."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    # Source the built workspace
    source install/setup.bash
    
    echo "✅ Workspace setup complete!"
    echo ""
    echo "📚 Next steps:"
    echo "   1. Read the documentation in docs/"
    echo "   2. Check docs/git_submodules_guide.md for submodule management"
    echo "   3. Source the workspace: source install/setup.bash"
    echo "   4. Try running: ros2 launch riggu_bringup riggu_system.launch.py"
    echo ""
    echo "🎯 Quick commands:"
    echo "   - Build only: colcon build"
    echo "   - Clean build: rm -rf build/ install/ log/ && colcon build"
    echo "   - Update submodules: git submodule update --recursive"
    
else
    echo "⚠️  ROS2 Kilted not found at /opt/ros/kilted/setup.bash"
    echo "   Please install ROS2 Kilted or use the dev container"
    echo "   Submodules have been initialized, but workspace not built"
    exit 1
fi

echo ""
echo "🤖 Happy coding with Riggu!"
