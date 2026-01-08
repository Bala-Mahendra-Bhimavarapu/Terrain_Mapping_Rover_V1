<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# could you explain to me how this system can be tested manually and autonamously

Perfect! Now let me create a comprehensive explanation of how to test your moon rover system both manually and autonomously:

***

## 🧪 **COMPLETE TESTING GUIDE: MANUAL vs AUTONOMOUS**

Your moon rover system supports both manual teleoperation and autonomous navigation. Here's how to test each mode:

***

## 🎮 **PART 1: MANUAL TESTING (Teleoperation)**

Manual testing allows you to control the rover directly using keyboard, joystick, or ROS commands.[^1][^2]

### **Method 1: Keyboard Teleoperation**

```bash
# Terminal 1: Start all sensors (Phases 1-3)
ros2 launch rover_launch phase_3_ekf.launch.py

# Terminal 2: Start keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Control with keyboard:
# i = forward
# k = stop
# j = turn left
# l = turn right
# , = backward
# q/z = increase/decrease speed
```

**What to Test:**

- ✅ Forward/backward motion (verify /vex/odom_raw updates)
- ✅ Left/right turning (verify IMU yaw changes)
- ✅ Speed control (start slow, increase gradually)
- ✅ Emergency stop (press 'k' immediately stops)
- ✅ Obstacle avoidance (manually navigate around objects)

***

### **Method 2: Joystick Teleoperation**

```bash
# Terminal 1: Start sensors
ros2 launch rover_launch phase_3_ekf.launch.py

# Terminal 2: Configure joystick
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
ros2 run joy joy_node

# Terminal 3: Map joystick to robot
ros2 run teleop_twist_joy teleop_node

# Control:
# Left stick = forward/backward
# Right stick = left/right turning
# Button 0 (A) = enable motion (deadman switch)
# Button 1 (B) = emergency stop
```

**What to Test:**

- ✅ Smooth motion control (joystick analog values)
- ✅ Deadman switch safety (motion only when held)
- ✅ Precision maneuvering (small adjustments)
- ✅ Response time (latency < 100ms)

***

### **Method 3: ROS Command Teleoperation**

```bash
# Terminal 1: Start sensors
ros2 launch rover_launch phase_3_ekf.launch.py

# Terminal 2: Send velocity commands directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.2, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}"

# Stop command:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}"

# Turn left:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.5}"
```

**What to Test:**

- ✅ Precise velocity control (linear.x = m/s, angular.z = rad/s)
- ✅ Command response (motion starts within 50ms)
- ✅ Safety limits (max velocity 0.3 m/s on moon)

***

### **Method 4: RViz Manual Control**

```bash
# Terminal 1: Start full system
ros2 launch rover_launch phase_4_rtabmap.launch.py

# Terminal 2: Start RViz with manual goal setting
ros2 run rviz2 rviz2

# In RViz:
# 1. Add → By Topic → /camera/image_raw (to see camera)
# 2. Add → By Topic → /rtabmap/mapData (to see 3D map)
# 3. Add → TF (to see robot pose)
# 4. Click "2D Nav Goal" button at top
# 5. Click on map where you want rover to go
# 6. Rover will plan path and execute
```

**What to Test:**[^3]

- ✅ Camera feed appears in real-time
- ✅ SLAM map building (3D points appear)
- ✅ TF tree shows all frames
- ✅ Goal setting works (path appears)
- ✅ Manual path correction (re-click if stuck)

***

## 🤖 **PART 2: AUTONOMOUS TESTING (Full Navigation)**

Autonomous mode allows the rover to navigate without human input using SLAM, sensors, and Nav2.[^4][^5]

### **Test 1: Autonomous Waypoint Navigation**

```bash
# Terminal 1: Start serial bridge
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Terminal 2: Launch full autonomous stack
ros2 launch rover_launch phase_10_waypoint_recording.launch.py

# Terminal 3: Record waypoints (drive manually first)
ros2 service call /waypoint_recorder/start_recording std_srvs/srv/Trigger

# Drive rover manually to 5-10 locations
# Each waypoint auto-saves every 0.5m

ros2 service call /waypoint_recorder/stop_recording std_srvs/srv/Trigger
ros2 service call /waypoint_recorder/save_waypoints std_srvs/srv/SetString \
  "data: '/tmp/mission_waypoints.yaml'"

# Terminal 4: Playback waypoints autonomously
ros2 service call /waypoint_player/load_waypoints std_srvs/srv/SetString \
  "data: '/tmp/mission_waypoints.yaml'"
ros2 service call /waypoint_player/start_playback std_srvs/srv/Trigger
```

**What Happens:**

1. Rover loads waypoint list
2. Nav2 plans path to first waypoint
3. Controller follows path using /odom and sensors
4. EKF fuses wheel encoders + IMU for localization
5. RTAB-Map corrects drift with loop closures
6. Rover reaches waypoint 1, plans to waypoint 2
7. Continues until all waypoints reached

**What to Test:**[^3]

- ✅ Waypoint accuracy (±10cm error acceptable)
- ✅ Obstacle avoidance (uses ToF depth camera)
- ✅ Path re-planning (if blocked, finds new route)
- ✅ Loop closure (re-recognizes previous locations)
- ✅ Mission completion (all waypoints reached)

***

### **Test 2: Autonomous Obstacle Avoidance**

```bash
# Terminal 1: Launch full system
ros2 launch rover_launch phase_9_nav2.launch.py

# Terminal 2: Set single navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, 
          pose: {position: {x: 3.0, y: 2.0, z: 0.0}, 
                 orientation: {w: 1.0}}}"

# Rover will:
# 1. Plan path to (3.0, 2.0)
# 2. Detect obstacles with ToF camera
# 3. Re-plan around obstacles
# 4. Reach goal or abort if unreachable
```

**What to Test:**

- ✅ Place obstacle in path → rover should stop and re-plan
- ✅ Move obstacle → rover should continue
- ✅ Block completely → rover should abort and report failure

***

### **Test 3: Full Autonomous Mission (30 minutes)**

```bash
# Terminal 1: Start serial bridge
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Terminal 2: Launch full mission
ros2 launch rover_launch phase_11_full_mission.launch.py

# Mission runs automatically:
# - Phase 1-3: Sensors + localization start
# - Phase 4: SLAM mapping begins
# - Phase 7: AI perception detects landmarks
# - Phase 8: Landmark verification confirms location
# - Phase 9-10: Nav2 executes waypoint mission
# - Phase 11: Mission logger records all data

# Monitor in Terminal 3:
ros2 topic echo /mission_health
# Shows: battery, sensor status, errors

# Monitor in Terminal 4:
ros2 topic echo /landmarks/verified
# Shows: detected mission markers

# View live video:
# Open browser: http://[rover_ip]:8080/stream
```

**What Happens Autonomously:**[^4]

1. **0-5 min:** Sensors initialize, SLAM builds initial map
2. **5-10 min:** Rover explores area, detects landmarks
3. **10-20 min:** Executes waypoint mission autonomously
4. **20-25 min:** AI identifies terrain (rock, sand, regolith)
5. **25-30 min:** Returns to start, logs mission data

**What to Test:**[^5][^3]

- ✅ **Heading accuracy:** ±5° over 5 meters
- ✅ **Trajectory error:** <10cm deviation from path
- ✅ **Obstacle detection:** Stops within 50cm of obstacles
- ✅ **Loop closure:** Re-recognizes start location
- ✅ **Mission completion:** Reaches all waypoints or reports failure

***

## 📊 **COMPARISON: MANUAL vs AUTONOMOUS**

| Aspect | Manual Testing | Autonomous Testing |
| :-- | :-- | :-- |
| **Control** | Human operator | AI navigation stack |
| **Speed** | Variable (0-0.5 m/s) | Fixed max (0.3 m/s on moon) |
| **Safety** | Deadman switch required | Auto-stop on obstacles |
| **Accuracy** | ±20cm (human error) | ±10cm (sensor fusion) |
| **Duration** | Continuous (operator fatigue) | 30+ min missions |
| **Obstacles** | Manual avoidance | Automatic detection + re-plan |
| **Mapping** | SLAM runs, operator controls | SLAM + Nav2 integrated |
| **Use Cases** | Initial testing, exploration | Mission execution, surveys |


***

## 🔍 **VERIFICATION PROCEDURES**

### **After Each Test, Verify:**

```bash
# 1. Check odometry accuracy
ros2 topic echo /odom --once
# Position should match actual rover location (±10cm)

# 2. Check TF tree validity
ros2 run tf2_tools view_frames.py
evince frames.pdf
# All frames should be connected: map→odom→base_link→sensors

# 3. Check sensor health
ros2 topic hz /vex/odom_raw  # Should be ~100 Hz
ros2 topic hz /imu/data      # Should be ~50 Hz
ros2 topic hz /camera/image_raw  # Should be ~30 Hz

# 4. Check mission health
ros2 topic echo /mission_health --once
# battery_voltage > 11.0V
# all sensor_status = "OK"
# error_count = 0

# 5. Review logged data
ros2 bag info /tmp/rover_missions/mission_YYYYMMDD_HHMMSS/
# Should show all topics recorded
```


***

## ✅ **TESTING CHECKLIST**

### **Manual Testing (1-2 hours)**

- [ ] Keyboard teleoperation works
- [ ] Joystick control responsive
- [ ] ROS command control accurate
- [ ] RViz manual goal setting functional
- [ ] Emergency stop immediate
- [ ] All sensors publishing data


### **Autonomous Testing (2-3 hours)**

- [ ] Waypoint recording successful
- [ ] Waypoint playback accurate
- [ ] Obstacle avoidance working
- [ ] Path re-planning functional
- [ ] SLAM loop closure detected
- [ ] Full 30-min mission completes
- [ ] Mission data logged correctly
- [ ] Live video stream accessible

***

## 🌙 **MOON-SPECIFIC TESTING**

Additional tests for lunar environment:

- ✅ **Low gravity handling:** Verify wheel slip detection (moon = 1/6 Earth)
- ✅ **Regolith navigation:** Test in sand/gravel (similar to moon dust)
- ✅ **Temperature extremes:** Test -180°C to +130°C (if possible)
- ✅ **Lighting conditions:** Test harsh shadows (simulate sun angles)
- ✅ **Communication latency:** Add 1.3s delay (Earth-Moon distance)

***

**Both manual and autonomous testing are essential to ensure your moon rover is mission-ready!** 🚀
<span style="display:none">[^10][^11][^12][^13][^6][^7][^8][^9]</span>

<div align="center">⁂</div>

[^1]: https://docs.pal-robotics.com/25.01/navigation/teleop-tutorial.html

[^2]: https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/adv-teleop/

[^3]: https://embeddedcomputing.com/application/industrial/automation-robotics/testing-and-analyzing-the-success-of-your-robot-navigation-system

[^4]: https://www.ijert.org/smart-rover-with-joystick-and-auto-nomous-modes-featuring-live-streaming-and-object-identification

[^5]: https://elib.dlr.de/194474/1/meyer2023testing_testingForTheMMXRoverAutonomousNavigationExperimentOnPhobos_copyright.pdf

[^6]: https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/145891694/3ae18336-8692-430c-be5d-969ee2ff7763/07_BUILD_PROCEDURES_UPDATED.md

[^7]: https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/145891694/0d3f3ff9-7197-4af9-8468-4ecfc432d8db/07_BUILD_PROCEDURES_UPDATED_WITH_PROS-1.md

[^8]: https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/145891694/376dea2b-db35-4573-8e6e-0893bf8deb63/06_LAUNCH_FILES.md

[^9]: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Testing/Testing-Main.html

[^10]: https://forums.tormach.com/t/teleoperation-using-moveit2-servo-ros2/2786

[^11]: https://www.youtube.com/watch?v=skv9fBnTFDU

[^12]: https://www.einfochips.com/blog/remote-tele-operation-of-robots-with-zenoh/

[^13]: https://www.sciencedirect.com/science/article/pii/S2405896325032392

