# Phase 0 — Workspace Setup Checklist

## Prerequisites
- [ ] Raspberry Pi 5 with Raspberry Pi OS Bookworm
- [ ] ROS 2 Humble built from source
- [ ] Git installed
- [ ] Python 3.10+ installed
- [ ] At least 10GB free disk space

## Setup Steps
- [ ] Run `setup_workspace.sh` to create directories and clone dependencies
- [ ] Run `create_bringup_dirs.sh` to create bringup structure
- [ ] Copy all package files (package.xml, CMakeLists. txt, etc.)
- [ ] Copy all message definitions to `tmr_msgs/msg/` and `tmr_msgs/srv/`
- [ ] Copy URDF files to `tmr_description/urdf/`
- [ ] Copy launch files to appropriate directories

## Build Steps
- [ ] Run `build_workspace.sh` (may take 30-60 minutes on Pi 5)
- [ ] Run `verify_build.sh` to confirm success

## Verification
- [ ] `source ~/terrain_mapping_rover_ws/setup_env.bash` works
- [ ] `ros2 pkg list | grep tmr` shows all tmr_ packages
- [ ] `ros2 interface show tmr_msgs/msg/MissionHealth` displays the message
- [ ] `ros2 launch tmr_description description. launch.py` runs without errors

## Definition of Done
- [ ] All TMR packages listed with `ros2 pkg list`
- [ ] All custom messages available with `ros2 interface list`
- [ ] URDF can be loaded and visualized
- [ ] No build errors or warnings

## Notes
_Add any issues or modifications here_