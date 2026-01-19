#!/bin/bash
# =============================================================================
# Verify the workspace build
# =============================================================================

source "$HOME/terrain_mapping_rover_ws/setup_env.bash"

echo "=============================================="
echo "Verifying Terrain Mapping Rover Build"
echo "=============================================="

# Check that key packages are available
REQUIRED_PACKAGES=(
    "tmr_msgs"
    "tmr_bringup"
    "tmr_description"
    "robot_localization"
)

ALL_OK=true

echo ""
echo "Checking required packages..."
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo "  ✓ $pkg"
    else
        echo "  ✗ $pkg (NOT FOUND)"
        ALL_OK=false
    fi
done

# Check that custom messages are available
echo ""
echo "Checking custom messages..."
if ros2 interface list | grep -q "tmr_msgs"; then
    echo "  ✓ tmr_msgs interfaces found"
    
    # List a few key messages
    echo ""
    echo "  Available messages:"
    ros2 interface list | grep "tmr_msgs/msg" | head -5 | while read msg; do
        echo "    - $msg"
    done
else
    echo "  ✗ tmr_msgs interfaces NOT FOUND"
    ALL_OK=false
fi

# Check TF2
echo ""
echo "Checking TF2..."
if ros2 pkg list | grep -q "tf2_ros"; then
    echo "  ✓ tf2_ros available"
else
    echo "  ✗ tf2_ros NOT FOUND"
    ALL_OK=false
fi

echo ""
echo "=============================================="
if $ALL_OK; then
    echo "All checks passed! Workspace is ready."
    echo "=============================================="
    exit 0
else
    echo "Some checks failed. Please review the build."
    echo "=============================================="
    exit 1
fi
