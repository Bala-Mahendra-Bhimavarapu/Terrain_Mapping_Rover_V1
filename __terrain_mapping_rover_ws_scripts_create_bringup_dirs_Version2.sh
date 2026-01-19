#!/bin/bash
# Create bringup package directory structure

BRINGUP_DIR="$HOME/terrain_mapping_rover_ws/src/tmr_bringup"

mkdir -p "$BRINGUP_DIR/launch/phases"
mkdir -p "$BRINGUP_DIR/launch/subsystems"
mkdir -p "$BRINGUP_DIR/config/ekf"
mkdir -p "$BRINGUP_DIR/config/slam"
mkdir -p "$BRINGUP_DIR/config/nav2"
mkdir -p "$BRINGUP_DIR/config/cameras"
mkdir -p "$BRINGUP_DIR/config/terrain"
mkdir -p "$BRINGUP_DIR/config/mission"
mkdir -p "$BRINGUP_DIR/rviz"

echo "Bringup directories created!"