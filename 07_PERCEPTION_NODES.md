# 07_PERCEPTION_NODES.md - Classifiers, Detectors, Auto-Exposure
# This consolidated file covers all perception packages

---

# PACKAGE 1: imx500_classifier_node
# Place in: src/imx500_classifier_node/

## FILE 1a: imx500_classifier_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>imx500_classifier_node</name>
  <version>0.0.1</version>
  <description>IMX500 on-device AI classifier for terrain and object detection</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>rover_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## FILE 1b: imx500_classifier_node/src/imx500_classifier_node.py

```python
#!/usr/bin/env python3
"""
IMX500 on-device AI classifier.
Uses NPU hardware acceleration on Pi AI Camera for real-time classification.

Classes for moon terrain: terrain, rock, crater, rover, landmark
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from rover_msgs.msg import Classification
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from picamera2 import Picamera2
    from picamera2.devices.imx500 import IMX500
    HAS_IMX500 = True
except ImportError:
    HAS_IMX500 = False
    print("WARNING: IMX500 libraries not available")

class IMX500ClassifierNode(Node):
    def __init__(self):
        super().__init__('imx500_classifier_node')
        
        self.declare_parameter('model_path', '/home/pi/models/terrain.tflite')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('classes', ['terrain', 'rock', 'crater', 'rover', 'landmark'])
        self.declare_parameter('enable_npu', True)
        
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.classes = self.get_parameter('classes').value
        self.enable_npu = self.get_parameter('enable_npu').value
        
        self.bridge = CvBridge()
        
        # Subscribe to RGB image
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos)
        
        # Publish classifications
        self.class_pub = self.create_publisher(
            Classification, '/ai_camera/classification', qos)
        
        # Initialize IMX500 (if available)
        self.imx500 = None
        if HAS_IMX500 and self.enable_npu:
            try:
                self.imx500 = IMX500(
                    model_path=self.model_path,
                    num_threads=4,
                    delegate=True  # Use NPU
                )
                self.get_logger().info(f"IMX500 NPU classifier initialized: {self.model_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to init NPU: {e}. Using CPU fallback.")
        else:
            self.get_logger().info("Using CPU-based classification (no NPU)")
        
        self.frame_count = 0
    
    def image_callback(self, msg):
        """Classify image frame"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Run inference
            if self.imx500:
                results = self.imx500.run(cv_image)
            else:
                # CPU fallback (placeholder - would use TFLite Interpreter)
                results = self.cpu_inference(cv_image)
            
            # Publish top result
            if results:
                top_result = results[0]  # Best match
                
                class_msg = Classification()
                class_msg.header = msg.header
                class_msg.label = self.classes[top_result['class_id']]
                class_msg.confidence = float(top_result['confidence'])
                class_msg.bbox_x_min = int(top_result.get('bbox_x_min', 0))
                class_msg.bbox_y_min = int(top_result.get('bbox_y_min', 0))
                class_msg.bbox_x_max = int(top_result.get('bbox_x_max', cv_image.shape[1]))
                class_msg.bbox_y_max = int(top_result.get('bbox_y_max', cv_image.shape[0]))
                class_msg.detection_id = self.frame_count
                
                self.class_pub.publish(class_msg)
                self.frame_count += 1
                
                if self.frame_count % 30 == 0:  # Log every 30 frames
                    self.get_logger().info(
                        f"[{self.frame_count}] Classification: {class_msg.label} ({class_msg.confidence:.2f})")
        
        except Exception as e:
            self.get_logger().error(f"Classification error: {e}")
    
    def cpu_inference(self, image):
        """CPU fallback (placeholder)"""
        # In production: use TFLite Interpreter
        # For now: return dummy classification
        return [{
            'class_id': np.argmax(np.random.rand(len(self.classes))),
            'confidence': 0.5,
        }]

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IMX500ClassifierNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# PACKAGE 2: auto_exposure_controller_node
# Place in: src/auto_exposure_controller_node/

## FILE 2a: auto_exposure_controller_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>auto_exposure_controller_node</name>
  <version>0.0.1</version>
  <description>Dynamic camera exposure control for varying moon terrain lighting</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>rover_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## FILE 2b: auto_exposure_controller_node/src/auto_exposure_controller_node.py

```python
#!/usr/bin/env python3
"""
Automatic exposure controller for IMX500 camera.
Adjusts exposure_time, analogue_gain, digital_gain to maintain stable brightness.

Moon terrain lighting: extremely variable (sun angle, shadows, crater reflections)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rover_msgs.msg import Classification
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

class AutoExposureController(Node):
    def __init__(self):
        super().__init__('auto_exposure_controller')
        
        # Parameters
        self.declare_parameter('target_brightness', 128)  # 0-255
        self.declare_parameter('brightness_tolerance', 15)  # ±15
        self.declare_parameter('max_exposure_time_ms', 33)  # 1/30 fps
        self.declare_parameter('min_exposure_time_ms', 1)   # 1000 fps max
        self.declare_parameter('max_gain', 4.0)
        self.declare_parameter('min_gain', 1.0)
        self.declare_parameter('adjustment_rate_hz', 2.0)  # Update frequency
        self.declare_parameter('lock_during_motion', False)
        self.declare_parameter('motion_threshold', 0.1)    # m/s for "motion"
        
        self.target_brightness = self.get_parameter('target_brightness').value
        self.brightness_tolerance = self.get_parameter('brightness_tolerance').value
        self.max_exposure_ms = self.get_parameter('max_exposure_time_ms').value
        self.min_exposure_ms = self.get_parameter('min_exposure_time_ms').value
        self.max_gain = self.get_parameter('max_gain').value
        self.min_gain = self.get_parameter('min_gain').value
        self.adjustment_rate = self.get_parameter('adjustment_rate_hz').value
        self.lock_during_motion = self.get_parameter('lock_during_motion').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        
        self.bridge = CvBridge()
        
        # Subscriptions
        self.image_sub = self.create_subscription(Image, '/camera/image_raw',
                                                   self.image_callback, 10)
        self.classification_sub = self.create_subscription(Classification,
                                                           '/ai_camera/classification',
                                                           self.classification_callback, 10)
        
        # Current exposure state
        self.exposure_time_us = 10000  # Microseconds
        self.analogue_gain = 1.0
        self.digital_gain = 1.0
        
        # Brightness tracking
        self.brightness_history = deque(maxlen=10)  # Rolling average
        self.is_locked = False
        
        # Timer for adjustment loop
        period = 1.0 / self.adjustment_rate
        self.adjust_timer = self.create_timer(period, self.adjustment_loop)
        
        self.get_logger().info("Auto-exposure controller initialized")
    
    def image_callback(self, msg):
        """Analyze image brightness"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Convert to grayscale for brightness
            gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            
            # Center region (ignore edges which may be underexposed)
            h, w = gray.shape
            roi = gray[h//4:3*h//4, w//4:3*w//4]
            
            # Median brightness (robust to outliers)
            brightness = float(np.median(roi))
            self.brightness_history.append(brightness)
            
        except Exception as e:
            self.get_logger().warn(f"Brightness analysis failed: {e}")
    
    def classification_callback(self, msg):
        """Use classification to inform exposure (e.g., very bright rocks)"""
        # Could boost exposure if "crater" detected (shadows)
        # Could reduce if "bright_rock" detected
        pass
    
    def adjustment_loop(self):
        """Periodically adjust exposure"""
        if not self.brightness_history or self.is_locked:
            return
        
        current_brightness = np.median(self.brightness_history)
        error = current_brightness - self.target_brightness
        
        # PID-like adjustment
        if abs(error) > self.brightness_tolerance:
            # Adjust exposure time first (less gain noise)
            if error < 0:  # Too dark
                # Increase exposure
                self.exposure_time_us = min(self.exposure_time_us * 1.2,
                                           self.max_exposure_ms * 1000)
            else:  # Too bright
                # Decrease exposure
                self.exposure_time_us = max(self.exposure_time_us / 1.2,
                                           self.min_exposure_ms * 1000)
            
            # If exposure maxed out, adjust gain
            if self.exposure_time_us >= self.max_exposure_ms * 1000 - 1000:
                if error < 0:  # Still too dark
                    self.analogue_gain = min(self.analogue_gain * 1.1, self.max_gain)
            elif self.exposure_time_us <= self.min_exposure_ms * 1000 + 100:
                if error > 0:  # Still too bright
                    self.analogue_gain = max(self.analogue_gain / 1.1, self.min_gain)
            
            # Log adjustment every 30 cycles
            if len(self.brightness_history) % 30 == 0:
                self.get_logger().debug(
                    f"Exposure: {self.exposure_time_us/1000:.1f}ms, "
                    f"Gain: {self.analogue_gain:.2f}x, "
                    f"Brightness: {current_brightness:.0f} (target: {self.target_brightness})")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AutoExposureController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## FILE 2c: auto_exposure_controller_node/config/auto_exposure_params.yaml

```yaml
auto_exposure_controller:
  ros__parameters:
    # Target brightness (0-255 range)
    target_brightness: 128               # Mid-range for balanced histogram
    brightness_tolerance: 15             # ±15 is acceptable range
    
    # Exposure time bounds
    max_exposure_time_ms: 33             # Max 33ms (1/30 sec)
    min_exposure_time_ms: 1              # Min 1ms (high FPS capable)
    
    # Analog gain (sensor amplification)
    max_gain: 4.0                        # 4x analog gain max
    min_gain: 1.0                        # No gain minimum
    
    # Control frequency
    adjustment_rate_hz: 2.0              # Adjust every 500ms
    
    # Motion detection (lock exposure during fast motion)
    lock_during_motion: false            # Don't lock (SLAM needs stable)
    motion_threshold: 0.1                # m/s
    
    # TUNING FOR MOON TERRAIN:
    # - Bright reflective rocks: can spike histogram
    # - Dark crater shadows: very dark regions
    # - Solution: Use median brightness (robust) + ROI (center only)
    # - Avoid: Fast oscillation by using low adjustment_rate
    # - Strategy: Exposure time > Gain > Digital_gain (noise increases each)
```

---

# PACKAGE 3: yolo_detector_node (already in MASTER but repeated here for clarity)
# Place in: src/yolo_detector_node/

## FILE 3: yolo_detector_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>yolo_detector_node</name>
  <version>0.0.1</version>
  <description>YOLO object detector for landmark detection on moon terrain</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>rover_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## BUILD & TEST

```bash
cd ~/ros2_moon_rover

# Build perception nodes
colcon build --packages-select \
  imx500_classifier_node \
  auto_exposure_controller_node \
  yolo_detector_node \
  --symlink-install

# Test
source install/setup.bash

# Terminal 1: Sensors + TF
ros2 launch rover_launch phase_1_sensors.launch.py &
ros2 run static_tf_publisher static_tf_broadcaster &

# Terminal 2: Classifiers
ros2 run imx500_classifier_node imx500_classifier_node &
ros2 run auto_exposure_controller_node auto_exposure_controller_node

# Terminal 3: Monitor
ros2 topic echo /ai_camera/classification
```

---

## NEXT: Localization nodes

Request **08_LOCALIZATION_NODES.md** for landmark map server, mission health monitor, landmark verification.
