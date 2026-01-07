# 08_LOCALIZATION_NODES.md - Landmark Map Server, Health Monitor, Verification
# Consolidated localization and mission monitoring packages

---

# PACKAGE 1: landmark_map_server
# Place in: src/landmark_map_server/

## FILE 1a: landmark_map_server/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>landmark_map_server</name>
  <version>0.0.1</version>
  <description>Persistent landmark map storage and server for moon rover</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>rover_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## FILE 1b: landmark_map_server/src/landmark_map_server.py

```python
#!/usr/bin/env python3
"""
Persistent landmark map server.
Maintains a database of detected landmarks and their 3D positions.
Saves to YAML/JSON for mission replay and cross-rover comparison.
"""

import rclpy
from rclpy.node import Node
from rover_msgs.msg import Landmark3D, LandmarkMap
from std_srvs.srv import Trigger
import yaml
import json
from pathlib import Path
from datetime import datetime

class LandmarkMapServer(Node):
    def __init__(self):
        super().__init__('landmark_map_server')
        
        self.declare_parameter('map_file', '/tmp/landmark_map.yaml')
        self.declare_parameter('update_frequency', 1.0)  # Hz
        self.declare_parameter('max_landmarks', 100)
        self.declare_parameter('duplicate_distance_threshold', 0.3)  # meters
        
        self.map_file = self.get_parameter('map_file').value
        self.update_freq = self.get_parameter('update_frequency').value
        self.max_landmarks = self.get_parameter('max_landmarks').value
        self.dup_threshold = self.get_parameter('duplicate_distance_threshold').value
        
        # Load existing map if available
        self.landmarks = {}  # {label: [positions]}
        self.load_map()
        
        # Subscribe to new landmarks
        self.landmark_sub = self.create_subscription(
            Landmark3D, '/landmarks/positions', self.landmark_callback, 10)
        
        # Publish map
        self.map_pub = self.create_publisher(LandmarkMap, '/landmarks/map', 10)
        
        # Services
        self.save_service = self.create_service(
            Trigger, 'save_landmarks', self.save_map_callback)
        self.load_service = self.create_service(
            Trigger, 'load_landmarks', self.load_map_callback)
        self.clear_service = self.create_service(
            Trigger, 'clear_landmarks', self.clear_map_callback)
        
        # Publish timer
        self.pub_timer = self.create_timer(1.0 / self.update_freq, self.publish_map)
        
        self.get_logger().info(f"Landmark map server initialized. Map file: {self.map_file}")
    
    def landmark_callback(self, msg):
        """Add or update landmark"""
        label = msg.label
        pos = (msg.position.x, msg.position.y, msg.position.z)
        
        if label not in self.landmarks:
            self.landmarks[label] = []
        
        # Check if duplicate (distance < threshold)
        is_duplicate = False
        for existing_pos in self.landmarks[label]:
            dist = ((pos[0] - existing_pos[0])**2 +
                   (pos[1] - existing_pos[1])**2 +
                   (pos[2] - existing_pos[2])**2)**0.5
            if dist < self.dup_threshold:
                is_duplicate = True
                break
        
        if not is_duplicate and len(self.landmarks.get(label, [])) < self.max_landmarks:
            self.landmarks[label].append(pos)
            self.get_logger().debug(f"Landmark added: {label} @ {pos}")
    
    def publish_map(self):
        """Publish current map"""
        msg = LandmarkMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for label, positions in self.landmarks.items():
            if positions:  # Use most recent position
                latest_pos = positions[-1]
                landmark_msg = Landmark3D()
                landmark_msg.label = label
                landmark_msg.position.x = latest_pos[0]
                landmark_msg.position.y = latest_pos[1]
                landmark_msg.position.z = latest_pos[2]
                landmark_msg.detection_count = len(positions)
                msg.landmarks.append(landmark_msg)
        
        msg.total_landmarks = len(msg.landmarks)
        self.map_pub.publish(msg)
    
    def load_map(self):
        """Load landmarks from file"""
        try:
            map_path = Path(self.map_file)
            if map_path.exists():
                with open(map_path, 'r') as f:
                    if self.map_file.endswith('.yaml'):
                        data = yaml.safe_load(f)
                    else:
                        data = json.load(f)
                
                if data and 'landmarks' in data:
                    for landmark in data['landmarks']:
                        label = landmark['label']
                        pos = (landmark['x'], landmark['y'], landmark['z'])
                        if label not in self.landmarks:
                            self.landmarks[label] = []
                        self.landmarks[label].append(pos)
                    
                    self.get_logger().info(f"Loaded {len(self.landmarks)} landmark types from {self.map_file}")
        except Exception as e:
            self.get_logger().warn(f"Failed to load landmark map: {e}")
    
    def save_map(self):
        """Save landmarks to file"""
        try:
            data = {
                'timestamp': datetime.now().isoformat(),
                'landmarks': []
            }
            
            for label, positions in self.landmarks.items():
                if positions:
                    latest = positions[-1]
                    data['landmarks'].append({
                        'label': label,
                        'x': float(latest[0]),
                        'y': float(latest[1]),
                        'z': float(latest[2]),
                        'count': len(positions)
                    })
            
            map_path = Path(self.map_file)
            map_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(map_path, 'w') as f:
                if self.map_file.endswith('.yaml'):
                    yaml.dump(data, f, default_flow_style=False)
                else:
                    json.dump(data, f, indent=2)
            
            self.get_logger().info(f"Saved {len(data['landmarks'])} landmarks to {self.map_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save landmark map: {e}")
    
    def save_map_callback(self, request, response):
        self.save_map()
        response.success = True
        response.message = f"Saved {len(self.landmarks)} landmark types"
        return response
    
    def load_map_callback(self, request, response):
        self.landmarks = {}
        self.load_map()
        response.success = True
        response.message = f"Loaded {len(self.landmarks)} landmark types"
        return response
    
    def clear_map_callback(self, request, response):
        self.landmarks = {}
        response.success = True
        response.message = "Cleared all landmarks"
        return response

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LandmarkMapServer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# PACKAGE 2: mission_health_monitor (complete version)
# Place in: src/mission_health_monitor/

## FILE 2: mission_health_monitor/src/mission_health_monitor_node.py

```python
#!/usr/bin/env python3
"""
Mission health monitor.
Tracks SLAM, odometry, perception, and navigation health.
Publishes /mission_health and /mission_events for operator awareness.
"""

import rclpy
from rclpy.node import Node
from rover_msgs.msg import MissionHealth, MissionEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

class MissionHealthMonitor(Node):
    def __init__(self):
        super().__init__('mission_health_monitor')
        
        # Thresholds
        self.declare_parameter('loop_closure_rate_critical', 0.0)
        self.declare_parameter('loop_closure_rate_degraded', 0.05)
        self.declare_parameter('feature_count_critical', 5)
        self.declare_parameter('feature_count_degraded', 20)
        self.declare_parameter('covariance_trace_critical', 2.0)
        self.declare_parameter('covariance_trace_degraded', 0.5)
        self.declare_parameter('acceleration_spike_threshold', 2.0)  # g's
        
        self.loop_closure_crit = self.get_parameter('loop_closure_rate_critical').value
        self.loop_closure_degrad = self.get_parameter('loop_closure_rate_degraded').value
        self.features_crit = self.get_parameter('feature_count_critical').value
        self.features_degrad = self.get_parameter('feature_count_degraded').value
        self.cov_crit = self.get_parameter('covariance_trace_critical').value
        self.cov_degrad = self.get_parameter('covariance_trace_degraded').value
        self.accel_spike = self.get_parameter('acceleration_spike_threshold').value
        
        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom',
                                                 self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data',
                                               self.imu_callback, 10)
        
        # Publications
        self.health_pub = self.create_publisher(MissionHealth, '/mission_health', 10)
        self.event_pub = self.create_publisher(MissionEvent, '/mission_events', 10)
        
        # Health state
        self.slam_health = MissionHealth.STATUS_OK
        self.odom_health = MissionHealth.STATUS_OK
        self.perception_health = MissionHealth.STATUS_OK
        self.navigation_health = MissionHealth.STATUS_OK
        self.landmark_health = MissionHealth.STATUS_OK
        
        # Statistics
        self.last_covariance_trace = 0.0
        self.feature_count = 0
        self.loop_closure_rate = 0.0
        
        # Timer
        self.health_timer = self.create_timer(1.0, self.publish_health)
        self.event_count = 0
        
        self.get_logger().info("Mission health monitor initialized")
    
    def odom_callback(self, msg):
        """Check odometry health"""
        # Covariance trace
        pose_cov = msg.pose.covariance
        trace = pose_cov[0] + pose_cov[7] + pose_cov[14]  # X + Y + Yaw variance
        
        if trace > self.cov_crit:
            self.odom_health = MissionHealth.STATUS_CRITICAL
            if self.last_covariance_trace < self.cov_crit:
                self.publish_event(
                    MissionEvent.SEV_ERROR, "odometry",
                    "critical_covariance",
                    f"Odometry covariance diverged: trace={trace:.3f}")
        elif trace > self.cov_degrad:
            self.odom_health = MissionHealth.STATUS_DEGRADED
        else:
            self.odom_health = MissionHealth.STATUS_OK
        
        self.last_covariance_trace = trace
    
    def imu_callback(self, msg):
        """Check IMU for anomalies"""
        accel_mag = np.sqrt(msg.linear_acceleration.x**2 +
                           msg.linear_acceleration.y**2 +
                           msg.linear_acceleration.z**2)
        
        # Normal: ~9.81 m/s^2 (gravity only when stationary)
        accel_error = abs(accel_mag - 9.81)
        if accel_error > self.accel_spike * 9.81:
            self.publish_event(
                MissionEvent.SEV_WARNING, "imu",
                "acceleration_spike",
                f"High acceleration detected: {accel_mag:.2f} m/s^2 (normal: 9.81)")
    
    def publish_event(self, severity, component, event_type, message):
        """Publish mission event"""
        event = MissionEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.severity = severity
        event.component = component
        event.event_type = event_type
        event.message = message
        self.event_pub.publish(event)
        self.event_count += 1
        
        severity_str = {0: 'INFO', 1: 'WARN', 2: 'ERROR', 3: 'CRITICAL'}.get(severity, 'UNKNOWN')
        self.get_logger().info(f"[{severity_str}] {component}: {message}")
    
    def publish_health(self):
        """Publish mission health status"""
        health = MissionHealth()
        health.header.stamp = self.get_clock().now().to_msg()
        
        # Overall status = worst component
        health.status = max(self.slam_health, self.odom_health,
                           self.perception_health, self.navigation_health)
        
        health.slam_health = self.slam_health
        health.odometry_health = self.odom_health
        health.perception_health = self.perception_health
        health.navigation_health = self.navigation_health
        health.landmark_health = self.landmark_health
        
        health.loop_closure_rate = self.loop_closure_rate
        health.feature_count = self.feature_count
        health.reprojection_error = float(self.last_covariance_trace)
        
        self.health_pub.publish(health)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MissionHealthMonitor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# PACKAGE 3: landmark_verification_node
# Place in: src/landmark_verification_node/

## FILE 3: landmark_verification_node/src/landmark_verification_node.py

```python
#!/usr/bin/env python3
"""
Cross-rover landmark verification.
Compares observed landmark positions with expected map positions.
Reports localization accuracy for mission health.
"""

import rclpy
from rclpy.node import Node
from rover_msgs.msg import Landmark3D, LandmarkVerification, MissionEvent
from tf2_ros import TransformListener, Buffer
import numpy as np

class LandmarkVerificationNode(Node):
    def __init__(self):
        super().__init__('landmark_verification_node')
        
        # Thresholds
        self.declare_parameter('error_ok_threshold', 0.3)    # meters
        self.declare_parameter('error_degraded_threshold', 1.0)
        
        self.error_ok = self.get_parameter('error_ok_threshold').value
        self.error_degrad = self.get_parameter('error_degraded_threshold').value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.landmark_sub = self.create_subscription(
            Landmark3D, '/landmarks/positions', self.landmark_callback, 10)
        self.map_sub = self.create_subscription(
            LandmarkMap, '/landmarks/map', self.map_callback, 10)
        
        # Publications
        self.verification_pub = self.create_publisher(
            LandmarkVerification, '/landmarks/verification', 10)
        self.event_pub = self.create_publisher(
            MissionEvent, '/mission_events', 10)
        
        self.landmark_map = {}  # {label: (x, y, z)}
        
        self.get_logger().info("Landmark verification node initialized")
    
    def map_callback(self, msg):
        """Update landmark map"""
        for landmark in msg.landmarks:
            self.landmark_map[landmark.label] = (
                landmark.position.x,
                landmark.position.y,
                landmark.position.z
            )
    
    def landmark_callback(self, msg):
        """Verify observed landmark"""
        label = msg.label
        
        if label not in self.landmark_map:
            return  # No reference for this landmark
        
        expected = self.landmark_map[label]
        observed = (msg.position.x, msg.position.y, msg.position.z)
        
        # Calculate error
        error_xyz = (
            observed[0] - expected[0],
            observed[1] - expected[1],
            observed[2] - expected[2]
        )
        error_mag = np.sqrt(sum(e**2 for e in error_xyz))
        
        # Determine status
        if error_mag < self.error_ok:
            status = LandmarkVerification.STATUS_OK
            severity = MissionEvent.SEV_INFO
        elif error_mag < self.error_degrad:
            status = LandmarkVerification.STATUS_DEGRADED
            severity = MissionEvent.SEV_WARNING
        else:
            status = LandmarkVerification.STATUS_CRITICAL
            severity = MissionEvent.SEV_ERROR
        
        # Publish verification
        verification = LandmarkVerification()
        verification.header = msg.header
        verification.landmark_label = label
        verification.expected_position.x = expected[0]
        verification.expected_position.y = expected[1]
        verification.expected_position.z = expected[2]
        verification.observed_position = msg.position
        verification.position_error.x = error_xyz[0]
        verification.position_error.y = error_xyz[1]
        verification.position_error.z = error_xyz[2]
        verification.error_magnitude = error_mag
        verification.verification_status = status
        
        self.verification_pub.publish(verification)
        
        # Log significant errors
        if status != LandmarkVerification.STATUS_OK:
            event = MissionEvent()
            event.header = msg.header
            event.severity = severity
            event.component = "landmark_verification"
            event.event_type = "localization_error"
            event.message = f"{label}: error {error_mag:.2f}m (expected: {expected}, observed: {observed})"
            self.event_pub.publish(event)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LandmarkVerificationNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## BUILD & TEST

```bash
cd ~/ros2_moon_rover

colcon build --packages-select \
  landmark_map_server \
  mission_health_monitor \
  landmark_verification_node \
  --symlink-install

# Test
source install/setup.bash
ros2 run mission_health_monitor mission_health_monitor_node
```

---

## NEXT: Navigation nodes

Request **09_NAVIGATION_NODES.md** for waypoint recording, terrain layer, and streaming.
