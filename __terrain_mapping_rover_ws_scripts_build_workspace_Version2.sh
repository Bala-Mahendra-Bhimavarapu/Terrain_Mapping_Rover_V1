#!/bin/bash
# =============================================================================
# Terrain Mapping Rover - Build Script
# Builds the workspace with optimizations for Pi 5
# =============================================================================

set -e

WORKSPACE_DIR="$HOME/terrain_mapping_rover_ws"
cd "$WORKSPACE_DIR"

# Source ROS 2
if [ -f /opt/ros/humble/setup. bash ]; then
    source /opt/ros/humble/setup. bash
elif [ -f ~/ros2_humble/install/setup.bash ]; then
    source ~/ros2_humble/install/setup.bash
else
    echo "ERROR: Could not find ROS 2 Humble installation"
    exit 1
fi

echo "=============================================="
echo "Building Terrain Mapping Rover Workspace"
echo "=============================================="

# Check available memory
TOTAL_MEM=$(free -g | awk '/^Mem:/{print $2}')
echo "Available RAM: ${TOTAL_MEM}GB"

# Set parallel jobs based on available memory
# Nav2 and rtabmap are memory-intensive
if [ "$TOTAL_MEM" -ge 8 ]; then
    PARALLEL_JOBS=3
else
    PARALLEL_JOBS=2
fi

echo "Using $PARALLEL_JOBS parallel jobs"

# Build options
BUILD_TYPE="Release"  # Use Release for performance on Pi 5

# First build:  messages only (other packages depend on these)
echo ""
echo "[1/3] Building message packages..."
colcon build \
    --symlink-install \
    --packages-select tmr_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    --parallel-workers $PARALLEL_JOBS

# Source the workspace to make messages available
source install/setup.bash

# Second build: core dependencies (can take a long time)
echo ""
echo "[2/3] Building core dependencies (this may take a while)..."
colcon build \
    --symlink-install \
    --packages-up-to robot_localization \
    --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    --parallel-workers $PARALLEL_JOBS

# Third build: everything else
echo ""
echo "[3/3] Building remaining packages..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    --parallel-workers $PARALLEL_JOBS

echo ""
echo "=============================================="
echo "Build complete!"
echo ""
echo "To use the workspace, run:"
echo "  source ~/terrain_mapping_rover_ws/setup_env.bash"
echo "=============================================="