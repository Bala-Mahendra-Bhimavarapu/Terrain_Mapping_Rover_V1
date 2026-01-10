# CAMERA AND TOF DRIVERS - UPDATED
## Inline Intrinsics + Official Arducam SDK

---

## IMX500 Camera Node (With Inline Intrinsics)

### File Structure
```
src/imx500_camera_node/
├── CMakeLists.txt
├── package.xml
└── src/
    └── imx500_camera_node.py
```

---

### FILE 1: imx500_camera_node/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>imx500_camera_node</name>
  <version>0.0.1</version>
  <description>Pi AI Camera IMX500 RGB image publisher with inline camera intrinsics</description>
  
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

### FILE 2: imx500_camera_node/CMakeLists.txt

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

ament_package()
```

---

### FILE 3: imx500_camera_node/src/imx500_camera_node.py

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

# Libcamera imports for Pi AI Camera
try:
    from libcamera import controls
    import picamera2
    HAS_LIBCAMERA = True
except ImportError:
    HAS_LIBCAMERA = False
    print("WARNING: libcamera not available, using fallback")


class IMX500CameraNode(Node):
    """Pi AI Camera IMX500 driver for ROS 2.
    
    Publishes RGB images with camera intrinsics (fx, fy, cx, cy, distortion)
    at configurable rate with exposure control.
    
    No YAML config files needed - all parameters inline.
    """

    def __init__(self):
        super().__init__('imx500_camera_node')

        # ==================== CAMERA INTRINSICS (INLINE) ====================
        # These are the calibration parameters - CUSTOMIZE FOR YOUR CAMERA!
        # You can get these from OpenCV calibration or camera datasheet
        
        # Focal lengths in pixels
        self.fx = 320.0      # Focal length X
        self.fy = 320.0      # Focal length Y
        
        # Principal point (image center)
        self.cx = 320.0      # Principal point X (usually width/2)
        self.cy = 240.0      # Principal point Y (usually height/2)
        
        # Distortion coefficients [k1, k2, p1, p2, k3]
        # k1, k2 = radial distortion
        # p1, p2 = tangential distortion
        # k3 = radial distortion (high-order)
        self.distortion_coeffs = [
            -0.05,    # k1 (radial)
            0.02,     # k2 (radial)
            0.0001,   # p1 (tangential)
            0.0001,   # p2 (tangential)
            0.0       # k3 (radial high-order)
        ]
        
        # Camera matrix K (3x3 intrinsic matrix)
        # K = [[fx,  0, cx],
        #      [0,  fy, cy],
        #      [0,   0,  1]]
        self.K = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix P (3x4)
        self.P = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # =====================================================================
        # Declare dynamic parameters (can be changed at runtime via ROS 2 CLI)
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
        
        # INLINE INTRINSICS - Override with your calibration values
        self.declare_parameter('intrinsic_fx', self.fx)
        self.declare_parameter('intrinsic_fy', self.fy)
        self.declare_parameter('intrinsic_cx', self.cx)
        self.declare_parameter('intrinsic_cy', self.cy)
        self.declare_parameter('distortion_k1', self.distortion_coeffs[0])
        self.declare_parameter('distortion_k2', self.distortion_coeffs[1])
        self.declare_parameter('distortion_p1', self.distortion_coeffs[2])
        self.declare_parameter('distortion_p2', self.distortion_coeffs[3])
        self.declare_parameter('distortion_k3', self.distortion_coeffs[4])
        
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
        
        # Load inline intrinsics from parameters
        self.fx = self.get_parameter('intrinsic_fx').value
        self.fy = self.get_parameter('intrinsic_fy').value
        self.cx = self.get_parameter('intrinsic_cx').value
        self.cy = self.get_parameter('intrinsic_cy').value
        
        self.distortion_coeffs = [
            self.get_parameter('distortion_k1').value,
            self.get_parameter('distortion_k2').value,
            self.get_parameter('distortion_p1').value,
            self.get_parameter('distortion_p2').value,
            self.get_parameter('distortion_k3').value,
        ]
        
        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Camera setup
        self.camera = None
        self.camera_config = None
        self.setup_camera()
        
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
        
        self.get_logger().info(
            f"IMX500 camera initialized: {self.resolution_width}x{self.resolution_height} @ {self.fps} FPS"
        )
        self.get_logger().info(
            f"Intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
        )

    def setup_camera(self):
        """Initialize libcamera camera"""
        if not HAS_LIBCAMERA:
            self.get_logger().error(
                "libcamera not available! Install with: sudo apt install -y libcamera-tools python3-libcamera"
            )
            return

        try:
            # Initialize Picamera2 (high-level libcamera wrapper)
            self.camera = picamera2.Picamera2(self.camera_index)
            
            # Configure camera
            config = self.camera.create_preview_configuration(
                main={
                    'format': 'RGB888',
                    'size': (self.resolution_width, self.resolution_height),
                },
                raw={
                    'format': 'SRGGB10',
                    'size': (4608, 2592)  # Native IMX500 resolution
                }
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
                self.camera.set_controls({controls.AwbMode: awb_map[mode]})
            except Exception as e:
                self.get_logger().warn(f"Failed to set AWB mode: {e}")

    def capture_and_publish(self):
        """Capture frame and publish as ROS Image + CameraInfo"""
        if not self.camera:
            self.get_logger().error("Camera not initialized")
            return

        try:
            # Capture array frame
            frame = self.camera.capture_array('main')
            
            # Handle color space: ensure RGB
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
            
            # Create CameraInfo message with inline intrinsics
            camera_info = self.get_camera_info(ros_image.header)
            
            # Publish
            self.image_pub.publish(ros_image)
            self.camera_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f"Capture failed: {e}")

    def get_camera_info(self, header):
        """Create CameraInfo message with inline calibration data"""
        camera_info = CameraInfo()
        camera_info.header = header
        camera_info.height = self.resolution_height
        camera_info.width = self.resolution_width
        camera_info.distortion_model = 'plumb_bob'
        
        # Intrinsic camera matrix K
        camera_info.k = self.K
        
        # Projection matrix P
        camera_info.p = self.P
        
        # Distortion coefficients
        camera_info.d = self.distortion_coeffs
        
        # Rectification matrix (identity)
        camera_info.r = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
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

## ToF Camera Node (Official Arducam SDK)

### File Structure
```
src/tof_camera_node/
├── CMakeLists.txt
├── package.xml
└── src/
    └── tof_camera_node.py
```

---

### FILE 4: tof_camera_node/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tof_camera_node</name>
  <version>0.0.1</version>
  <description>Arducam ToF depth camera driver with official SDK and inline intrinsics</description>
  
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

### FILE 5: tof_camera_node/CMakeLists.txt

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

ament_package()
```

---

### FILE 6: tof_camera_node/src/tof_camera_node.py

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
import time

# Official Arducam SDK
try:
    import ArducamDepthCamera as ac
    HAS_ARDUCAM_SDK = True
except ImportError:
    HAS_ARDUCAM_SDK = False
    print("WARNING: ArducamDepthCamera SDK not available")
    print("Install with: pip3 install ArducamDepthCamera")


class ToFCameraNode(Node):
    """Arducam ToF (Time-of-Flight) depth camera driver using official SDK.
    
    Publishes:
    - Depth images (mono16)
    - Point clouds (xyz32)
    - CameraInfo with inline calibration (fx, fy, cx, cy, distortion)
    
    No YAML config files needed - all parameters inline.
    Uses official Arducam APIs.
    """

    def __init__(self):
        super().__init__('tof_camera_node')

        # ==================== CAMERA INTRINSICS (INLINE) ====================
        # Arducam ToF 320x240 default intrinsics
        # CUSTOMIZE for your specific camera!
        
        self.fx = 240.0      # Focal length X (pixels)
        self.fy = 240.0      # Focal length Y (pixels)
        self.cx = 160.0      # Principal point X (image center)
        self.cy = 120.0      # Principal point Y (image center)
        
        # Distortion coefficients (ToF typically minimal)
        self.distortion_coeffs = [
            0.0,      # k1
            0.0,      # k2
            0.0,      # p1
            0.0,      # p2
            0.0       # k3
        ]
        
        # =====================================================================
        # Declare parameters
        self.declare_parameter('connection_type', 'CSI')  # CSI or USB
        self.declare_parameter('resolution_width', 320)
        self.declare_parameter('resolution_height', 240)
        self.declare_parameter('fps', 15)
        self.declare_parameter('min_depth_mm', 0)
        self.declare_parameter('max_depth_mm', 4000)  # 2000 or 4000
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('confidence_threshold', 30)
        
        # Inline intrinsics
        self.declare_parameter('intrinsic_fx', self.fx)
        self.declare_parameter('intrinsic_fy', self.fy)
        self.declare_parameter('intrinsic_cx', self.cx)
        self.declare_parameter('intrinsic_cy', self.cy)
        self.declare_parameter('distortion_k1', self.distortion_coeffs[0])
        self.declare_parameter('distortion_k2', self.distortion_coeffs[1])
        self.declare_parameter('distortion_p1', self.distortion_coeffs[2])
        self.declare_parameter('distortion_p2', self.distortion_coeffs[3])
        self.declare_parameter('distortion_k3', self.distortion_coeffs[4])
        
        # Get parameters
        self.connection_type = self.get_parameter('connection_type').value
        self.resolution_width = self.get_parameter('resolution_width').value
        self.resolution_height = self.get_parameter('resolution_height').value
        self.fps = self.get_parameter('fps').value
        self.min_depth = self.get_parameter('min_depth_mm').value
        self.max_depth = self.get_parameter('max_depth_mm').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_cloud = self.get_parameter('publish_cloud').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Load inline intrinsics
        self.fx = self.get_parameter('intrinsic_fx').value
        self.fy = self.get_parameter('intrinsic_fy').value
        self.cx = self.get_parameter('intrinsic_cx').value
        self.cy = self.get_parameter('intrinsic_cy').value
        
        self.distortion_coeffs = [
            self.get_parameter('distortion_k1').value,
            self.get_parameter('distortion_k2').value,
            self.get_parameter('distortion_p1').value,
            self.get_parameter('distortion_p2').value,
            self.get_parameter('distortion_k3').value,
        ]
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Arducam camera object (from official SDK)
        self.camera = None
        self.setup_camera()
        
        # ROS publishers
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
        period = 1.0 / self.fps if self.fps > 0 else 0.1
        self.timer = self.create_timer(period, self.capture_and_publish)
        
        self.get_logger().info(
            f"ToF camera initialized: {self.resolution_width}x{self.resolution_height} @ {self.fps} FPS"
        )
        self.get_logger().info(
            f"Depth range: {self.min_depth}-{self.max_depth} mm"
        )
        self.get_logger().info(
            f"Intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
        )

    def setup_camera(self):
        """Initialize Arducam ToF camera using official SDK"""
        if not HAS_ARDUCAM_SDK:
            self.get_logger().error(
                "ArducamDepthCamera SDK not available! "
                "Install with: pip3 install ArducamDepthCamera"
            )
            return

        try:
            # Create camera object (official SDK)
            self.camera = ac.ArducamCamera()
            
            # Open camera via CSI or USB
            if self.connection_type.upper() == 'USB':
                ret = self.camera.open(ac.Connection.USB, 0)
                self.get_logger().info("Opening camera via USB")
            else:
                ret = self.camera.open(ac.Connection.CSI, 0)
                self.get_logger().info("Opening camera via CSI")
            
            if ret != 0:
                self.get_logger().error(f"Failed to open camera. Error code: {ret}")
                return
            
            # Start depth frame capture (official SDK)
            ret = self.camera.start(ac.FrameType.DEPTH)
            if ret != 0:
                self.get_logger().error(f"Failed to start camera. Error code: {ret}")
                self.camera.close()
                return
            
            # Set depth range (2000 or 4000 mm)
            self.camera.setControl(ac.Control.RANGE, self.max_depth)
            
            # Get camera info from SDK
            info = self.camera.getCameraInfo()
            self.get_logger().info(f"Camera resolution: {info.width}x{info.height}")
            self.get_logger().info(f"Device type: {info.device_type}")
            
            self.get_logger().info("ToF camera started successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ToF camera: {e}")
            self.camera = None

    def capture_and_publish(self):
        """Capture depth frame using official SDK and publish"""
        if not self.camera:
            return

        try:
            # Request frame from SDK (timeout 2000ms)
            frame = self.camera.requestFrame(2000)
            
            if frame is None:
                # Timeout - no frame available
                return
            
            # Check frame type (official SDK)
            if not isinstance(frame, ac.DepthData):
                self.camera.releaseFrame(frame)
                return
            
            # Extract depth and confidence data from SDK frame
            depth_buf = frame.depth_data         # uint16, in mm
            confidence_buf = frame.confidence_data  # uint8 (if available)
            
            # Apply confidence filtering if available
            if confidence_buf is not None:
                depth_buf = np.nan_to_num(depth_buf)
                depth_buf[confidence_buf < self.confidence_threshold] = 0
            
            # Clip to valid range
            depth_clipped = np.clip(depth_buf, self.min_depth, self.max_depth)
            
            # Publish as ROS Image (mono16 - millimeters)
            ros_depth = self.bridge.cv2_to_imgmsg(depth_clipped, encoding='mono16')
            ros_depth.header.frame_id = self.frame_id
            ros_depth.header.stamp = self.get_clock().now().to_msg()
            
            self.depth_pub.publish(ros_depth)
            
            # Publish CameraInfo with inline intrinsics
            camera_info = self.get_camera_info(ros_depth.header)
            self.camera_info_pub.publish(camera_info)
            
            # Optionally publish point cloud
            if self.publish_cloud:
                cloud = self.depth_to_cloud(depth_clipped, ros_depth.header)
                self.cloud_pub.publish(cloud)
            
            # Release frame back to SDK
            self.camera.releaseFrame(frame)
            
        except Exception as e:
            self.get_logger().error(f"Capture failed: {e}")

    def depth_to_cloud(self, depth_frame, header):
        """Convert depth frame to PointCloud2 using inline intrinsics"""
        h, w = depth_frame.shape
        points = []
        
        for y in range(h):
            for x in range(w):
                d = depth_frame[y, x]
                
                # Valid depth check
                if d >= self.min_depth and d <= self.max_depth:
                    # Convert depth from mm to meters
                    z = d / 1000.0
                    
                    # Project pixel to 3D using intrinsics
                    px = (x - self.cx) * z / self.fx
                    py = (y - self.cy) * z / self.fy
                    
                    points.append([px, py, z])
        
        # Create empty cloud if no valid points
        if not points:
            points = [[0.0, 0.0, 0.0]]
        
        # Create PointCloud2 message (official SDK-based data)
        cloud = create_cloud_xyz32(header, points)
        return cloud

    def get_camera_info(self, header):
        """Create CameraInfo message with inline calibration"""
        camera_info = CameraInfo()
        camera_info.header = header
        camera_info.height = self.resolution_height
        camera_info.width = self.resolution_width
        camera_info.distortion_model = 'plumb_bob'
        
        # Intrinsic matrix K
        camera_info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix P
        camera_info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Distortion (ToF typically minimal)
        camera_info.d = self.distortion_coeffs
        
        # Rectification matrix (identity)
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        return camera_info

    def __del__(self):
        """Cleanup"""
        if self.camera:
            try:
                self.camera.stop()
                self.camera.close()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ToFCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Installation & Testing

### Install Arducam SDK

```bash
# Install official Arducam SDK
pip3 install ArducamDepthCamera

# Verify installation
python3 -c "import ArducamDepthCamera as ac; print(ac.__version__)"
```

### Build Packages

```bash
cd ~/ros2_moon_rover

# Build camera drivers
colcon build --packages-select imx500_camera_node tof_camera_node --symlink-install

# Source workspace
source install/setup.bash
```

### Test IMX500 Camera (RGB)

```bash
# Terminal 1: Start serial bridge (if using VEX Brain)
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Terminal 2: Run IMX500 node (uses INLINE intrinsics - no YAML needed!)
ros2 run imx500_camera_node imx500_camera_node

# Terminal 3: Verify topics
ros2 topic list
ros2 topic echo /image_raw        # RGB images
ros2 topic echo /camera_info      # Camera intrinsics
```

### Test ToF Camera (Depth with Official SDK)

```bash
# Terminal 1: Start serial bridge (if needed)
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Terminal 2: Run ToF node (uses official Arducam SDK + INLINE intrinsics)
ros2 run tof_camera_node tof_camera_node

# Terminal 3: Verify topics
ros2 topic list
ros2 topic echo /depth/image_raw   # Depth (mono16, mm)
ros2 topic echo /points            # PointCloud2
ros2 topic echo /camera_info       # Camera intrinsics
```

---

## Customizing Intrinsics

### For IMX500 RGB Camera

Edit the parameters directly in the code or use ROS 2 CLI:

```bash
# Run with custom intrinsics
ros2 run imx500_camera_node imx500_camera_node --ros-args \
  -p intrinsic_fx:=350.5 \
  -p intrinsic_fy:=348.2 \
  -p intrinsic_cx:=320.0 \
  -p intrinsic_cy:=240.5 \
  -p distortion_k1:=-0.08 \
  -p distortion_k2:=0.03
```

### For ToF Camera

```bash
# Run with custom intrinsics
ros2 run tof_camera_node tof_camera_node --ros-args \
  -p intrinsic_fx:=250.0 \
  -p intrinsic_fy:=248.5 \
  -p intrinsic_cx:=160.0 \
  -p intrinsic_cy:=120.5 \
  -p max_depth_mm:=2000
```

---

## Key Changes

### ✅ What's Different

1. **No YAML Files** - All intrinsics are inline parameters (fx, fy, cx, cy, distortion coefficients)
2. **Official Arducam SDK** - ToF now uses official `ArducamDepthCamera` APIs
3. **Inline Calibration** - Define camera matrix K and distortion directly in code
4. **Parameter Override** - Change intrinsics at runtime via ROS 2 CLI
5. **Complete Frame Handling** - Uses official SDK methods: `requestFrame()`, `releaseFrame()`, `getCameraInfo()`

### 📊 CameraInfo Output

Both nodes now publish complete `CameraInfo` with:
- **K matrix** (3x3 intrinsic matrix)
- **P matrix** (3x4 projection matrix)
- **Distortion coefficients** [k1, k2, p1, p2, k3]
- **Rectification matrix** (identity for now)

All without needing external YAML files!
