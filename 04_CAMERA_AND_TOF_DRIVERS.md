# PHASE 1: IMX500 CAMERA DRIVER (Pi AI Camera)
# Place this in: src/imx500_camera_node/

## FILE STRUCTURE:
```
imx500_camera_node/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── imx500_camera_node.py
├── config/
│   └── camera_params.yaml
└── scripts/
    └── camera_calibration_helper.py
```

---

## FILE 1: imx500_camera_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>imx500_camera_node</name>
  <version>0.0.1</version>
  <description>Pi AI Camera (IMX500) RGB image publisher with auto-exposure</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <build_depend>rosidl_cmake</build_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>image_transport</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## FILE 2: imx500_camera_node/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(imx500_camera_node)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/imx500_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## FILE 3: imx500_camera_node/src/imx500_camera_node.py

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

# Libcamera imports (Pi AI Camera)
try:
    from libcamera import controls, Transform
    import picamera2
    HAS_LIBCAMERA = True
except ImportError:
    HAS_LIBCAMERA = False
    print("WARNING: libcamera not available, using fallback")

class IMX500CameraNode(Node):
    """
    Pi AI Camera (IMX500) driver for ROS 2.
    Publishes RGB images + camera info at configurable rate with exposure control.
    """
    
    def __init__(self):
        super().__init__('imx500_camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('auto_exposure_enabled', True)
        self.declare_parameter('exposure_time_us', 10000)
        self.declare_parameter('analogue_gain', 1.0)
        self.declare_parameter('digital_gain', 1.0)
        self.declare_parameter('awb_mode', 'auto')  # auto, daylight, tungsten, cloudy
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('calibration_file', '')
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.resolution_width = self.get_parameter('resolution_width').value
        self.resolution_height = self.get_parameter('resolution_height').value
        self.fps = self.get_parameter('fps').value
        self.auto_exposure = self.get_parameter('auto_exposure_enabled').value
        self.exposure_time_us = self.get_parameter('exposure_time_us').value
        self.analogue_gain = self.get_parameter('analogue_gain').value
        self.digital_gain = self.get_parameter('digital_gain').value
        self.awb_mode = self.get_parameter('awb_mode').value
        self.frame_id = self.get_parameter('frame_id').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Camera setup
        self.camera = None
        self.camera_config = None
        self._setup_camera()
        
        # ROS publishers
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.image_pub = self.create_publisher(Image, 'image_raw', qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', qos)
        
        # Timer for capture loop
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.capture_and_publish)
        
        self.get_logger().info(f"IMX500 camera initialized: {self.resolution_width}x{self.resolution_height} @ {self.fps} FPS")
        
    def _setup_camera(self):
        """Initialize libcamera camera"""
        if not HAS_LIBCAMERA:
            self.get_logger().error("libcamera not available! Install: sudo apt install -y libcamera-tools python3-libcamera")
            return
        
        try:
            # Initialize Picamera2 (high-level libcamera wrapper)
            self.camera = picamera2.Picamera2(self.camera_index)
            
            # Configure camera
            config = self.camera.create_preview_configuration(
                main={"format": "RGB888", "size": (self.resolution_width, self.resolution_height)},
                raw={"format": "SRGGB10", "size": (4608, 2592)}  # Native IMX500 resolution
            )
            self.camera.configure(config)
            
            # Set exposure and gain
            if not self.auto_exposure:
                self.camera.set_controls({
                    controls.ExposureTime: self.exposure_time_us,
                    controls.AnalogueGain: self.analogue_gain,
                    controls.DigitalGain: self.digital_gain,
                })
            else:
                # Enable auto exposure with specific target
                self.camera.set_controls({
                    controls.AeExposureMode: controls.AeExposureMode.Normal,
                })
            
            # Set white balance
            self.set_awb_mode(self.awb_mode)
            
            # Start camera
            self.camera.start()
            self.get_logger().info("Camera started successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            self.camera = None
    
    def set_awb_mode(self, mode):
        """Set automatic white balance mode"""
        if not self.camera:
            return
        
        awb_map = {
            'auto': controls.AwbMode.Auto,
            'daylight': controls.AwbMode.Daylight,
            'tungsten': controls.AwbMode.Tungsten,
            'cloudy': controls.AwbMode.Cloudy,
        }
        
        if mode in awb_map:
            try:
                self.camera.set_controls({
                    controls.AwbMode: awb_map[mode]
                })
            except Exception as e:
                self.get_logger().warn(f"Failed to set AWB mode: {e}")
    
    def capture_and_publish(self):
        """Capture frame and publish as ROS Image + CameraInfo"""
        if not self.camera:
            self.get_logger().error("Camera not initialized")
            return
        
        try:
            # Capture array
            frame = self.camera.capture_array("main")
            
            # Convert BGR (if needed) to RGB
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                # Already RGB, no conversion needed
                pass
            elif len(frame.shape) == 2:
                # Grayscale - convert to RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            
            # Create ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            ros_image.header.frame_id = self.frame_id
            ros_image.header.stamp = self.get_clock().now().to_msg()
            
            # Create CameraInfo message
            camera_info = self._get_camera_info(ros_image.header)
            
            # Publish
            self.image_pub.publish(ros_image)
            self.camera_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f"Capture failed: {e}")
    
    def _get_camera_info(self, header):
        """Create CameraInfo message with calibration data"""
        camera_info = CameraInfo()
        camera_info.header = header
        camera_info.height = self.resolution_height
        camera_info.width = self.resolution_width
        camera_info.distortion_model = 'plumb_bob'
        
        # Default camera matrix for IMX500 @ 640x480
        # These are approximate - MUST be calibrated!
        focal_length_x = 320.0  # pixels
        focal_length_y = 320.0
        center_x = self.resolution_width / 2.0
        center_y = self.resolution_height / 2.0
        
        # Intrinsic camera matrix K
        camera_info.k = [
            focal_length_x, 0.0, center_x,
            0.0, focal_length_y, center_y,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix P
        camera_info.p = [
            focal_length_x, 0.0, center_x, 0.0,
            0.0, focal_length_y, center_y, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Distortion coefficients (assume minimal for now)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        return camera_info
    
    def set_exposure(self, exposure_us, gain):
        """Dynamically update exposure parameters (called by auto-exposure controller)"""
        if not self.camera or self.auto_exposure:
            return
        
        try:
            self.camera.set_controls({
                controls.ExposureTime: int(exposure_us),
                controls.AnalogueGain: float(gain),
            })
        except Exception as e:
            self.get_logger().warn(f"Failed to set exposure: {e}")
    
    def __del__(self):
        """Cleanup"""
        if self.camera:
            self.camera.close()

def main(args=None):
    rclpy.init(args=args)
    node = IMX500CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## FILE 4: imx500_camera_node/config/camera_params.yaml

```yaml
imx500_camera_node:
  ros__parameters:
    # Camera hardware
    camera_index: 0                     # Index if multiple cameras connected
    
    # Resolution (lower = faster on Pi)
    resolution_width: 640               # Pixels
    resolution_height: 480
    
    # Framerate
    fps: 30                             # Hz (reduce to 15-20 for compute-constrained)
    
    # Exposure control
    auto_exposure_enabled: true         # Enable libcamera auto-exposure
    exposure_time_us: 10000             # Microseconds (ignored if auto_exposure=true)
    analogue_gain: 1.0                  # 1.0 = no gain (ignored if auto_exposure=true)
    digital_gain: 1.0                   # Sensor-level digital gain
    
    # White balance
    awb_mode: "auto"                    # Options: auto, daylight, tungsten, cloudy
    
    # ROS metadata
    frame_id: "camera_link"             # TF frame name
    calibration_file: ""                # Path to calibration YAML (optional)
    
    # TUNING NOTES:
    # - Moon terrain is low-texture: reduce resolution, increase exposure
    # - Pi5 can handle 640x480 @ 30 fps comfortably
    # - For SLAM, prefer 20-30 fps steady over higher variable fps
    # - Increase exposure_time_us if images too dark (e.g., shadows in craters)
    # - Increase analogue_gain if noise acceptable over underexposure
```

---

# PHASE 1: ARDUCAM TOF CAMERA DRIVER
# Place this in: src/tof_camera_node/

## FILE STRUCTURE:
```
tof_camera_node/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── tof_camera_node.py
├── config/
│   └── tof_params.yaml
└── include/
    └── tof_helper.h  (C++ helper if needed)
```

---

## FILE 5: tof_camera_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>tof_camera_node</name>
  <version>0.0.1</version>
  <description>Arducam ToF depth camera driver (CSI)</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## FILE 6: tof_camera_node/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(tof_camera_node)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/tof_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## FILE 7: tof_camera_node/src/tof_camera_node.py

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs.point_cloud2 import create_cloud_xyz32
import cv2
import numpy as np
from cv_bridge import CvBridge
import struct
import time

# Arducam ToF library (via pivariety/libcamera)
try:
    from libcamera import controls
    import picamera2
    HAS_LIBCAMERA = True
except ImportError:
    HAS_LIBCAMERA = False
    print("WARNING: libcamera not available")

class ToFCameraNode(Node):
    """
    Arducam ToF (Time-of-Flight) depth camera driver.
    Publishes depth images + point clouds at configurable rate.
    """
    
    def __init__(self):
        super().__init__('tof_camera_node')
        
        # Parameters
        self.declare_parameter('camera_index', 1)  # ToF usually index 1
        self.declare_parameter('resolution_width', 320)
        self.declare_parameter('resolution_height', 240)
        self.declare_parameter('fps', 15)
        self.declare_parameter('min_depth_mm', 0)
        self.declare_parameter('max_depth_mm', 3000)
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('fx', 240.0)  # Focal length (pixels)
        self.declare_parameter('fy', 240.0)
        self.declare_parameter('cx', 160.0)  # Principal point
        self.declare_parameter('cy', 120.0)
        
        self.camera_index = self.get_parameter('camera_index').value
        self.resolution_width = self.get_parameter('resolution_width').value
        self.resolution_height = self.get_parameter('resolution_height').value
        self.fps = self.get_parameter('fps').value
        self.min_depth = self.get_parameter('min_depth_mm').value
        self.max_depth = self.get_parameter('max_depth_mm').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_cloud = self.get_parameter('publish_cloud').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        self.bridge = CvBridge()
        self.camera = None
        self._setup_camera()
        
        # Publishers
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.depth_pub = self.create_publisher(Image, 'depth/image_raw', qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', qos)
        if self.publish_cloud:
            self.cloud_pub = self.create_publisher(PointCloud2, 'points', qos)
        
        # Capture loop
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.capture_and_publish)
        
        self.get_logger().info(f"ToF camera initialized: {self.resolution_width}x{self.resolution_height} @ {self.fps} FPS")
    
    def _setup_camera(self):
        """Initialize ToF camera via libcamera"""
        if not HAS_LIBCAMERA:
            self.get_logger().error("libcamera not available")
            return
        
        try:
            self.camera = picamera2.Picamera2(self.camera_index)
            
            # Configure for depth (raw format)
            config = self.camera.create_preview_configuration(
                main={"format": "MONO16", "size": (self.resolution_width, self.resolution_height)}
            )
            self.camera.configure(config)
            self.camera.start()
            
            self.get_logger().info("ToF camera started")
        except Exception as e:
            self.get_logger().error(f"ToF camera init failed: {e}")
            self.camera = None
    
    def capture_and_publish(self):
        """Capture depth frame and publish image + point cloud"""
        if not self.camera:
            return
        
        try:
            # Capture depth frame (MONO16, depth in mm)
            depth_frame = self.camera.capture_array("main")
            
            # Clip to valid range
            depth_clipped = np.clip(depth_frame, self.min_depth, self.max_depth)
            
            # Publish as ROS Image
            ros_depth = self.bridge.cv2_to_imgmsg(depth_clipped, encoding='mono16')
            ros_depth.header.frame_id = self.frame_id
            ros_depth.header.stamp = self.get_clock().now().to_msg()
            self.depth_pub.publish(ros_depth)
            
            # Publish CameraInfo
            camera_info = self._get_camera_info(ros_depth.header)
            self.camera_info_pub.publish(camera_info)
            
            # Optionally publish point cloud
            if self.publish_cloud:
                cloud = self._depth_to_cloud(depth_clipped, ros_depth.header)
                self.cloud_pub.publish(cloud)
                
        except Exception as e:
            self.get_logger().error(f"Capture failed: {e}")
    
    def _depth_to_cloud(self, depth_frame, header):
        """Convert depth frame to PointCloud2 (XYZ)"""
        h, w = depth_frame.shape
        
        # Create point array
        points = []
        for y in range(h):
            for x in range(w):
                d = depth_frame[y, x]
                if d > self.min_depth and d < self.max_depth:
                    # Convert depth to meters
                    z = d / 1000.0
                    # Project to 3D
                    px = (x - self.cx) * z / self.fx
                    py = (y - self.cy) * z / self.fy
                    points.append([px, py, z])
        
        if not points:
            # Empty cloud
            points = [[0.0, 0.0, 0.0]]
        
        # Create PointCloud2 message
        cloud = create_cloud_xyz32(header, points)
        return cloud
    
    def _get_camera_info(self, header):
        """Create CameraInfo message"""
        camera_info = CameraInfo()
        camera_info.header = header
        camera_info.height = self.resolution_height
        camera_info.width = self.resolution_width
        camera_info.distortion_model = 'plumb_bob'
        
        # Intrinsic matrix
        camera_info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix
        camera_info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # No distortion for ToF
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        return camera_info
    
    def __del__(self):
        if self.camera:
            self.camera.close()

def main(args=None):
    rclpy.init(args=args)
    node = ToFCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## FILE 8: tof_camera_node/config/tof_params.yaml

```yaml
tof_camera_node:
  ros__parameters:
    # Hardware
    camera_index: 1                     # ToF camera (usually 1 if RGB is 0)
    
    # Resolution
    resolution_width: 320               # ToF typically 320x240
    resolution_height: 240
    
    # Framerate
    fps: 15                             # Hz (ToF slower than RGB)
    
    # Depth range
    min_depth_mm: 100                   # Minimum valid depth (mm)
    max_depth_mm: 3000                  # Maximum valid depth (mm, ~3m for moon terrain)
    
    # Camera intrinsics (MUST CALIBRATE!)
    fx: 240.0                           # Focal length X (pixels)
    fy: 240.0                           # Focal length Y
    cx: 160.0                           # Principal point X
    cy: 120.0                           # Principal point Y
    
    # ROS metadata
    frame_id: "tof_link"
    
    # Output
    publish_cloud: true                 # Also publish PointCloud2
    
    # TUNING NOTES:
    # - Moon terrain is uneven: keep fps modest to avoid motion blur
    # - Depth is critical for costmap: prioritize accuracy over speed
    # - Increase min_depth_mm if floor reflections are issue
    # - Calibrate intrinsics with checkerboard + intrinsic calibration tool
```

---

## BUILD & TEST

```bash
cd ~/ros2_moon_rover

# Build camera drivers
colcon build --packages-select imx500_camera_node tof_camera_node --symlink-install

# Test (requires actual Pi hardware)
source install/setup.bash

# Terminal 1: IMX500 RGB
ros2 run imx500_camera_node imx500_camera_node --ros-args \
  --params-file src/imx500_camera_node/config/camera_params.yaml

# Terminal 2: ToF depth
ros2 run tof_camera_node tof_camera_node --ros-args \
  --params-file src/tof_camera_node/config/tof_params.yaml

# Terminal 3: Monitor
ros2 topic list
ros2 topic echo /image_raw      # RGB frames
ros2 topic echo /depth/image_raw  # Depth
```

---

## NEXT: Static TF Publisher

Request **05_STATIC_TF_PUBLISHER.md** for the transform tree setup.
