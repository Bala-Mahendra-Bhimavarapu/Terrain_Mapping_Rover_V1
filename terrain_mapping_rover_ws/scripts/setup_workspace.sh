#!/bin/bash
# =============================================================================
# Terrain Mapping Rover - Workspace Setup Script
# Phase 0: Creates workspace structure and clones dependencies
# =============================================================================

set -e  # Exit on error

WORKSPACE_DIR="$HOME/terrain_mapping_rover_ws"
SRC_DIR="$WORKSPACE_DIR/src"

echo "=============================================="
echo "Terrain Mapping Rover - Workspace Setup"
echo "=============================================="

# Create workspace directories
echo "[1/6] Creating workspace directories..."
mkdir -p "$WORKSPACE_DIR"/{src,scripts,vex_v5_code}
mkdir -p "$WORKSPACE_DIR/vex_v5_code/terrain_rover"

# Create all TMR package directories
echo "[2/6] Creating TMR package directories..."
TMR_PACKAGES=(
    "tmr_bringup"
    "tmr_description"
    "tmr_vex_serial"
    "tmr_imu_driver"
    "tmr_camera"
    "tmr_tof_camera"
    "tmr_ekf"
    "tmr_slam"
    "tmr_navigation"
    "tmr_terrain_layer"
    "tmr_auto_exposure"
    "tmr_classification"
    "tmr_mission_health"
    "tmr_waypoint"
    "tmr_landmark"
    "tmr_bluetooth_nav"
    "tmr_cross_rover_sync"
    "tmr_streaming"
    "tmr_teleop"
    "tmr_web_dashboard"
    "tmr_msgs"
    "tmr_calibration"
)

for pkg in "${TMR_PACKAGES[@]}"; do
    mkdir -p "$SRC_DIR/$pkg"
done

echo "[3/6] Cloning external dependencies from source..."
cd "$SRC_DIR"

# Clone if not already present
clone_if_missing() {
    local repo_url=$1
    local target_dir=$2
    local branch=${3:-humble}
    
    if [ !  -d "$target_dir" ]; then
        echo "  Cloning $target_dir..."
        git clone --branch "$branch" --depth 1 "$repo_url" "$target_dir"
    else
        echo "  $target_dir already exists, skipping..."
    fi
}

# Core dependencies
clone_if_missing "https://github.com/introlab/rtabmap_ros.git" "rtabmap_ros" "ros2"
clone_if_missing "https://github.com/introlab/rtabmap.git" "rtabmap" "master"
clone_if_missing "https://github.com/ros-navigation/navigation2.git" "navigation2" "humble"
clone_if_missing "https://github.com/cra-ros-pkg/robot_localization.git" "robot_localization" "ros2"

# Image/camera dependencies
clone_if_missing "https://github.com/ros-perception/image_common.git" "image_common" "humble"
clone_if_missing "https://github.com/ros-perception/vision_opencv.git" "vision_opencv" "humble"
clone_if_missing "https://github.com/ros-perception/image_pipeline.git" "image_pipeline" "humble"

# Diagnostics
clone_if_missing "https://github.com/ros/diagnostics.git" "diagnostics" "humble"

# BehaviorTree. CPP (required by Nav2)
clone_if_missing "https://github.com/BehaviorTree/BehaviorTree.CPP.git" "BehaviorTree. CPP" "v4. 6.2"

# Bond (required by lifecycle nodes)
clone_if_missing "https://github.com/ros/bond_core.git" "bond_core" "ros2"

# Angles (common dependency)
clone_if_missing "https://github.com/ros/angles.git" "angles" "ros2"

echo "[4/6] Installing Python dependencies..."
pip3 install --user --break-system-packages \
    pyserial \
    smbus2 \
    bleak \
    ultralytics \
    flask \
    flask-socketio \
    opencv-python \
    numpy \
    pyyaml \
    transforms3d


echo "[5/6] Creating environment setup script..."
cat > "$WORKSPACE_DIR/setup_env.bash" << 'EOF'
#!/bin/bash
# Source this file to set up the environment

# Source ROS 2 Humble (adjust path if different)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f ~/ros2_humble/install/setup.bash ]; then
    source ~/ros2_humble/install/setup.bash
else
    echo "WARNING: Could not find ROS 2 Humble installation"
fi

# Source workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

# Set ROS domain ID (change if needed for multi-robot)
export ROS_DOMAIN_ID=0

# Set RMW implementation (optional, for DDS tuning)
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Environment ready:  terrain_mapping_rover_ws"
EOF

chmod +x "$WORKSPACE_DIR/setup_env.bash"

echo "[6/6] Setup complete!"
echo ""
echo "=============================================="
echo "Next steps:"
echo "1. cd $WORKSPACE_DIR"
echo "2. Review and create package files (see Phase 0 documentation)"
echo "3. Build with:  colcon build --symlink-install"
echo "=============================================="
