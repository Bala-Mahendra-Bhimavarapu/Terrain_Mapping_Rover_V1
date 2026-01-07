# 09_NAVIGATION_NODES.md - Waypoint Recording, Terrain Layer, Logging, Streaming
# Consolidated navigation and mission support packages

---

# PACKAGE 1: waypoint_recorder_node & waypoint_player_node
# Place in: src/waypoint_recorder_node/ and src/waypoint_player_node/

## FILE 1a: waypoint_recorder_node/src/waypoint_recorder_node.py

```python
#!/usr/bin/env python3
"""
Waypoint recorder - saves rover trajectory for mission replay.
Records pose at configurable intervals (distance-based, time-based, or event-based).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import yaml
import json
from pathlib import Path
from math import sqrt

class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')
        
        self.declare_parameter('output_file', '/tmp/waypoints.yaml')
        self.declare_parameter('recording_enabled', False)
        self.declare_parameter('record_distance', 0.5)  # meters
        self.declare_parameter('record_time', 5.0)      # seconds
        self.declare_parameter('min_samples', 3)        # minimum to save
        
        self.output_file = self.get_parameter('output_file').value
        self.recording_enabled = self.get_parameter('recording_enabled').value
        self.record_distance = self.get_parameter('record_distance').value
        self.record_time = self.get_parameter('record_time').value
        self.min_samples = self.get_parameter('min_samples').value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.waypoints = []
        self.last_waypoint_pos = None
        self.last_record_time = self.get_clock().now()
        
        # Services
        self.start_recording_service = self.create_service(
            std_srvs.srv.SetBool, 'start_recording', self.start_recording_callback)
        self.save_service = self.create_service(
            std_srvs.srv.Trigger, 'save_waypoints', self.save_waypoints_callback)
        self.clear_service = self.create_service(
            std_srvs.srv.Trigger, 'clear_waypoints', self.clear_waypoints_callback)
        
        # Recording timer
        self.timer = self.create_timer(0.1, self.record_pose)
        
        self.get_logger().info(f"Waypoint recorder initialized. Output: {self.output_file}")
    
    def record_pose(self):
        """Periodically record rover pose"""
        if not self.recording_enabled:
            return
        
        try:
            # Get current pose from TF
            transform = self.tf_buffer.lookup_transform('map', 'base_link', 
                                                        rclpy.time.Time())
            
            t = transform.transform.translation
            r = transform.transform.rotation
            
            current_pos = (t.x, t.y)
            
            # Determine if we should record
            should_record = False
            
            # Time-based check
            time_delta = (self.get_clock().now() - self.last_record_time).nanoseconds / 1e9
            if time_delta >= self.record_time:
                should_record = True
            
            # Distance-based check
            if self.last_waypoint_pos:
                dist = sqrt((current_pos[0] - self.last_waypoint_pos[0])**2 +
                           (current_pos[1] - self.last_waypoint_pos[1])**2)
                if dist >= self.record_distance:
                    should_record = True
            elif len(self.waypoints) == 0:
                # First waypoint
                should_record = True
            
            if should_record:
                waypoint = {
                    'x': float(t.x),
                    'y': float(t.y),
                    'z': float(t.z),
                    'qx': float(r.x),
                    'qy': float(r.y),
                    'qz': float(r.z),
                    'qw': float(r.w),
                    'timestamp': self.get_clock().now().to_msg().nanosec
                }
                
                self.waypoints.append(waypoint)
                self.last_waypoint_pos = current_pos
                self.last_record_time = self.get_clock().now()
                
                if len(self.waypoints) % 10 == 0:
                    self.get_logger().info(f"Recorded {len(self.waypoints)} waypoints")
        
        except Exception as e:
            pass  # TF not available yet
    
    def start_recording_callback(self, request, response):
        self.recording_enabled = request.data
        response.success = True
        response.message = f"Recording {'started' if request.data else 'stopped'}"
        return response
    
    def save_waypoints_callback(self, request, response):
        self.save_waypoints()
        response.success = True
        response.message = f"Saved {len(self.waypoints)} waypoints"
        return response
    
    def clear_waypoints_callback(self, request, response):
        self.waypoints = []
        response.success = True
        response.message = "Cleared all waypoints"
        return response
    
    def save_waypoints(self):
        """Save waypoints to file"""
        if len(self.waypoints) < self.min_samples:
            self.get_logger().warn(f"Not enough waypoints ({len(self.waypoints)} < {self.min_samples})")
            return
        
        output_path = Path(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        data = {
            'mission': 'moon_rover_mission',
            'waypoint_count': len(self.waypoints),
            'waypoints': self.waypoints
        }
        
        try:
            with open(output_path, 'w') as f:
                if str(output_path).endswith('.yaml'):
                    yaml.dump(data, f, default_flow_style=False)
                else:
                    json.dump(data, f, indent=2)
            
            self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointRecorderNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## FILE 1b: waypoint_player_node/src/waypoint_player_node.py

```python
#!/usr/bin/env python3
"""
Waypoint player - replays recorded trajectory.
Sends goals to Nav2 and verifies waypoint completion.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import json
from pathlib import Path
from math import atan2

class WaypointPlayerNode(Node):
    def __init__(self):
        super().__init__('waypoint_player_node')
        
        self.declare_parameter('waypoint_file', '/tmp/waypoints.yaml')
        self.declare_parameter('goal_tolerance', 0.3)  # meters
        
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        self.navigator = BasicNavigator()
        self.waypoints = []
        self.current_waypoint = 0
        self.is_playing = False
        
        # Load waypoints
        self.load_waypoints()
        
        # Services
        self.start_service = self.create_service(
            std_srvs.srv.Trigger, 'start_replay', self.start_replay_callback)
        self.stop_service = self.create_service(
            std_srvs.srv.Trigger, 'stop_replay', self.stop_replay_callback)
        
        # Replay timer
        self.replay_timer = self.create_timer(1.0, self.replay_loop)
        
        self.get_logger().info(f"Waypoint player loaded {len(self.waypoints)} waypoints from {self.waypoint_file}")
    
    def load_waypoints(self):
        """Load waypoints from file"""
        try:
            waypoint_path = Path(self.waypoint_file)
            if not waypoint_path.exists():
                self.get_logger().error(f"Waypoint file not found: {self.waypoint_file}")
                return
            
            with open(waypoint_path, 'r') as f:
                if str(waypoint_path).endswith('.yaml'):
                    data = yaml.safe_load(f)
                else:
                    data = json.load(f)
            
            if 'waypoints' in data:
                self.waypoints = data['waypoints']
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
    
    def replay_loop(self):
        """Execute replay"""
        if not self.is_playing or not self.waypoints:
            return
        
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("Replay complete!")
            self.is_playing = False
            return
        
        wp = self.waypoints[self.current_waypoint]
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = wp['x']
        goal_pose.pose.position.y = wp['y']
        goal_pose.pose.position.z = wp['z']
        goal_pose.pose.orientation.x = wp['qx']
        goal_pose.pose.orientation.y = wp['qy']
        goal_pose.pose.orientation.z = wp['qz']
        goal_pose.pose.orientation.w = wp['qw']
        
        # Send goal
        self.navigator.goToPose(goal_pose)
        
        # Wait for completion
        if self.navigator.isTaskComplete():
            self.current_waypoint += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint}/{len(self.waypoints)}")
    
    def start_replay_callback(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints loaded"
            return response
        
        self.is_playing = True
        self.current_waypoint = 0
        response.success = True
        response.message = f"Starting replay of {len(self.waypoints)} waypoints"
        return response
    
    def stop_replay_callback(self, request, response):
        self.is_playing = False
        self.navigator.cancelTask()
        response.success = True
        response.message = "Replay stopped"
        return response

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointPlayerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# PACKAGE 2: mission_logger_node (rosbag + event logging)

## FILE 2: mission_logger_node/src/mission_logger_node.py

```python
#!/usr/bin/env python3
"""
Mission logger - records all sensor data and events to rosbag.
Enables mission replay and post-analysis.
"""

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import subprocess
from pathlib import Path
from datetime import datetime

class MissionLoggerNode(Node):
    def __init__(self):
        super().__init__('mission_logger_node')
        
        self.declare_parameter('log_directory', '/tmp/rover_missions')
        self.declare_parameter('record_topics', [
            '/camera/image_raw',
            '/tof/depth/image_raw',
            '/odom',
            '/imu/data',
            '/mission_health',
            '/mission_events',
            '/landmarks/positions',
            '/landmarks/detections'
        ])
        
        self.log_dir = Path(self.get_parameter('log_directory').value)
        self.topics = self.get_parameter('record_topics').value
        
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.recording_process = None
        self.get_logger().info(f"Mission logger initialized. Directory: {self.log_dir}")
    
    def start_recording(self):
        """Start rosbag recording"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = self.log_dir / f"mission_{timestamp}"
        
        cmd = ['ros2', 'bag', 'record'] + self.topics + ['-o', str(bag_path)]
        
        try:
            self.recording_process = subprocess.Popen(cmd)
            self.get_logger().info(f"Recording started: {bag_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")
    
    def stop_recording(self):
        """Stop rosbag recording"""
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.get_logger().info("Recording stopped")

def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
    node.start_recording()
    rclpy.spin(node)
    node.stop_recording()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# PACKAGE 3: streaming_node (MJPEG video server)

## FILE 3: streaming_node/src/streaming_node.py

```python
#!/usr/bin/env python3
"""
Streaming node - serves live camera feed via HTTP (MJPEG).
Accessible at http://<pi-ip>:8080/stream
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
import io

class StreamingNode(Node):
    def __init__(self):
        super().__init__('streaming_node')
        
        self.declare_parameter('port', 8080)
        self.declare_parameter('resolution_width', 320)
        self.declare_parameter('resolution_height', 240)
        self.declare_parameter('quality', 80)  # JPEG quality
        
        self.port = self.get_parameter('port').value
        self.width = self.get_parameter('resolution_width').value
        self.height = self.get_parameter('resolution_height').value
        self.quality = self.get_parameter('quality').value
        
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(Image, '/camera/image_raw',
                                                  self.image_callback, 10)
        
        # Start HTTP server
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f"Streaming server started on port {self.port}")
    
    def image_callback(self, msg):
        """Store latest frame"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # Resize if needed
            if self.latest_frame.shape[:2] != (self.height, self.width):
                self.latest_frame = cv2.resize(self.latest_frame, (self.width, self.height))
        except Exception as e:
            pass
    
    def start_server(self):
        """Start HTTP MJPEG server"""
        handler = self.create_handler()
        server = HTTPServer(('0.0.0.0', self.port), handler)
        server.serve_forever()
    
    def create_handler(self):
        """Create HTTP request handler"""
        parent = self
        
        class StreamHandler(SimpleHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    
                    try:
                        while True:
                            if parent.latest_frame is not None:
                                ret, jpeg = cv2.imencode('.jpg', parent.latest_frame,
                                                        [cv2.IMWRITE_JPEG_QUALITY, parent.quality])
                                frame_bytes = jpeg.tobytes()
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                self.wfile.write(f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode())
                                self.wfile.write(frame_bytes)
                                self.wfile.write(b'\r\n')
                    except:
                        pass
                else:
                    self.send_response(404)
                    self.end_headers()

        return StreamHandler

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StreamingNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## BUILD & TEST

```bash
cd ~/ros2_moon_rover

colcon build --packages-select \
  waypoint_recorder_node \
  waypoint_player_node \
  mission_logger_node \
  streaming_node \
  --symlink-install

# Test streaming
source install/setup.bash
ros2 run streaming_node streaming_node
# Access at: http://<pi-ip>:8080/stream
```

---

## NEXT: Configuration reference and final procedures

Request **11_CONFIGURATION_REFERENCE.md** for complete YAML configs and tuning guides.
