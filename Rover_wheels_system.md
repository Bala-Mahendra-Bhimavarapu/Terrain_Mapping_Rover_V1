<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# does the system use 2 motors to control the wheels only or a motor on each wheel for a total of 4 motors to control the rover and does it have tank steering or regular car steering.

Perfect! Based on the documentation, I can now give you a complete answer:

***

## 🤖 **MOTOR CONFIGURATION \& STEERING TYPE**

### **Motor Setup: 4 Independent Motors**

Your moon rover uses **4 motors total - one motor per wheel**.[^1]

**Motor Configuration:**

```
Motor Ports on VEX V5 Brain:
- Port 1: motor_FL (Front Left)
- Port 2: motor_FR (Front Right)
- Port 3: motor_BL (Back Left)
- Port 4: motor_BR (Back Right)
```

From the build procedures: *"Verify motors are detected - Check that ports 1-4 show motors"* and *"grep motorFL include/robot-config.h - Should show: extern pros::Motor motor_FL"*.[^1]

***

### **Steering Type: Tank/Skid Steering (Differential Drive)**

Your rover uses **tank steering** (also called skid steering or differential drive), NOT regular car steering.[^2][^3]

**How Tank Steering Works:**

```
Forward Motion:
  FL ➡️  FR ➡️
  BL ➡️  BR ➡️
  (All 4 wheels spin forward at same speed)

Turn Left:
  FL ⬅️  FR ➡️
  BL ⬅️  BR ➡️
  (Left wheels reverse, right wheels forward)

Turn Right:
  FL ➡️  FR ⬅️
  BL ➡️  BR ⬅️
  (Right wheels reverse, left wheels forward)

Spin in Place (Zero-Turn Radius):
  FL ➡️  FR ⬅️
  BL ➡️  BR ⬅️
  (Opposite sides spin in opposite directions)
```


***

## 🔄 **Why Tank Steering for Moon Rover?**

Tank steering is superior for lunar terrain because:[^3][^4]

1. **Zero-turn radius** - Can pivot in place (crucial for tight spaces)
2. **Simple mechanics** - No steering servos to fail
3. **Better traction** - All 4 wheels always powered
4. **Rugged design** - Fewer moving parts for harsh environment
5. **Precision control** - Independent wheel control for accurate navigation

***

## ⚙️ **Odometry Calculation**

The V5 Brain firmware reads all 4 wheel encoders and calculates odometry using **differential drive kinematics**:[^1]

```cpp
// Simplified odometry calculation (from PROS firmware):
float left_distance  = (motor_FL_ticks + motor_BL_ticks) / 2;
float right_distance = (motor_FR_ticks + motor_BR_ticks) / 2;

float linear_velocity  = (left_distance + right_distance) / 2;
float angular_velocity = (right_distance - left_distance) / wheel_base;

// Published to /vex/odom_raw at 100 Hz
```


***

## 🎮 **Control Comparison**

| Aspect | Tank Steering (Your Rover) | Car Steering |
| :-- | :-- | :-- |
| **Motors** | 4 (one per wheel) | 2 (rear wheels only) |
| **Steering Mechanism** | Differential speed | Front wheel angle |
| **Turn Radius** | 0 (can spin in place) | ~1-2 meters minimum |
| **Complexity** | Simple (no servos) | Complex (steering servo) |
| **Traction** | All 4 wheels powered | Only 2 wheels powered |
| **Best For** | Rough terrain, tight spaces | Roads, smooth surfaces |


***

## 📡 **ROS 2 Control Topics**

Your system publishes `/cmd_vel` which tank steering interprets as:[^5]

```bash
# Forward motion:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
# All 4 motors spin forward at 0.3 m/s

# Turn left while moving:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"
# Left wheels slower, right wheels faster

# Spin in place (left):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
# Left wheels reverse, right wheels forward
```


***

## ✅ **SUMMARY**

- ✅ **4 motors total** (FL, FR, BL, BR on ports 1-4)
- ✅ **Tank/skid steering** (differential drive)
- ✅ **NOT car steering** (no front wheel angle control)
- ✅ **All 4 wheels powered** at all times
- ✅ **Zero-turn radius** capable
- ✅ **Ideal for moon terrain** (rugged, simple, maneuverable)

Your rover is built like a tank - each side moves independently for maximum control in harsh lunar conditions! 🌙🚀
<span style="display:none">[^6][^7]</span>

<div align="center">⁂</div>

[^1]: 07_BUILD_PROCEDURES_UPDATED_WITH_PROS-1.md

[^2]: https://embeddedcomputing.com/application/industrial/automation-robotics/testing-and-analyzing-the-success-of-your-robot-navigation-system

[^3]: https://www.ijert.org/smart-rover-with-joystick-and-auto-nomous-modes-featuring-live-streaming-and-object-identification

[^4]: https://elib.dlr.de/194474/1/meyer2023testing_testingForTheMMXRoverAutonomousNavigationExperimentOnPhobos_copyright.pdf

[^5]: https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/adv-teleop/

[^6]: 07_BUILD_PROCEDURES_UPDATED.md

[^7]: 06_LAUNCH_FILES.md

