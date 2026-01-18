#!/usr/bin/env python3
"""
IMX500 Camera Node - ROS 2 driver for Pi AI Camera.

Publishes: 
- /camera/image_raw (sensor_msgs/Image)
- /camera/camera_info (sensor_msgs/CameraInfo)

Author: Rover Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs. msg import Header
from cv_bridge import CvBridge

from rover_interfaces.msg import CameraParams, ExposureState

from imx500_camera import IMX500Driver, CameraInfoManager
from imx500_camera.camera_driver import CameraConfig


class IMX500CameraNode(Node):
    """ROS 2 node for IMX500 camera."""
    
    def __init__(self):
        super().__init__('imx500_camera_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get configuration
        config = self._get_camera_config()
        
        # Initialize camera driver
        self. driver = IMX500Driver(config)
        
        if not self.driver.initialize():
            self.get_logger().error('Failed to initialize IMX500 camera!')
            raise RuntimeError('Camera initialization failed')
        
        self.get_logger().info('IMX500 camera initialized')
        
        # Initialize camera info manager
        self.camera_info_manager = CameraInfoManager()
        self._setup_camera_info()
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Get frame ID and rate
        self.frame_id = self.get_parameter('frame_id').value
        update_rate = self.get_parameter('update_rate').value
        
        # QoS profile for camera data
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', camera_qos)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', camera_qos)
        self.exposure_state_pub = self.create_publisher(
            ExposureState, '/camera/exposure_state', 10)
        
        # Subscribers
        self.params_sub = self.create_subscription(
            CameraParams, '/camera/set_params', self._params_callback, 10)
        
        # Timer for capture
        period = 1.0 / update_rate
        self.timer = self.create_timer(period, self._capture_callback)
        
        self.get_logger().info(
            f'Publishing camera images at {update_rate} Hz on /camera/image_raw')
    
    def _declare_parameters(self):
        """Declare all node parameters."""
        # Camera config
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        
        # Exposure
        self.declare_parameter('ae_enable', True)
        self.declare_parameter('exposure_time_us', 20000)
        self.declare_parameter('analogue_gain', 1.0)
        self.declare_parameter('digital_gain', 1.0)
        
        # Limits
        self.declare_parameter('min_exposure_us', 100)
        self.declare_parameter('max_exposure_us', 100000)
        self.declare_parameter('min_gain', 1.0)
        self.declare_parameter('max_gain', 16.0)
        
        # Intrinsics - can be set directly without YAML
        self.declare_parameter('use_yaml_intrinsics', False)
        self.declare_parameter('intrinsics_yaml_path', '')
        
        # Direct intrinsic parameters (used when use_yaml_intrinsics=False)
        self.declare_parameter('camera_fx', 600.0)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 360.0)
        self.declare_parameter('distortion_coefficients', [0.0, 0.0, 0.0, 0.0, 0.0])
    
    def _get_camera_config(self) -> CameraConfig:
        """Build camera config from parameters."""
        return CameraConfig(
            width=self.get_parameter('width').value,
            height=self. get_parameter('height').value,
            framerate=int(self.get_parameter('update_rate').value),
            ae_enable=self.get_parameter('ae_enable').value,
            exposure_time_us=self.get_parameter('exposure_time_us').value,
            analogue_gain=self.get_parameter('analogue_gain').value,
            digital_gain=self.get_parameter('digital_gain').value,
            min_exposure_us=self.get_parameter('min_exposure_us').value,
            max_exposure_us=self.get_parameter('max_exposure_us').value,
            min_gain=self.get_parameter('min_gain').value,
            max_gain=self.get_parameter('max_gain').value,
        )
    
    def _setup_camera_info(self):
        """Set up camera intrinsics."""
        use_yaml = self.get_parameter('use_yaml_intrinsics').value
        
        if use_yaml:
            yaml_path = self.get_parameter('intrinsics_yaml_path').value
            if yaml_path and self.camera_info_manager.load_from_yaml(yaml_path):
                self.get_logger().info(f'Loaded intrinsics from {yaml_path}')
                return
            else:
                self.get_logger().warn('Failed to load YAML, using parameters')
        
        # Use direct parameters
        width = self.get_parameter('width').value
        height = self. get_parameter('height').value
        fx = self.get_parameter('camera_fx').value
        fy = self.get_parameter('camera_fy').value
        cx = self.get_parameter('camera_cx').value
        cy = self.get_parameter('camera_cy').value
        distortion = self.get_parameter('distortion_coefficients').value
        
        self.camera_info_manager.set_from_parameters(
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            distortion=distortion
        )
        
        self.get_logger().info(
            f'Camera intrinsics:  fx={fx}, fy={fy}, cx={cx}, cy={cy}')
    
    def _capture_callback(self):
        """Capture and publish image."""
        result = self.driver.capture()
        if result is None:
            return
        
        # Create timestamp
        now = self.get_clock().now()
        
        # Create and publish Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(result.image, encoding='rgb8')
            image_msg. header.stamp = now. to_msg()
            image_msg.header.frame_id = self.frame_id
            self.image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Publish CameraInfo
        camera_info_msg = self.camera_info_manager.to_ros_msg()
        if camera_info_msg: 
            camera_info_msg.header = image_msg.header
            self.camera_info_pub.publish(camera_info_msg)
        
        # Publish exposure state
        exp_state = ExposureState()
        exp_state.header. stamp = now.to_msg()
        exp_state.current_brightness = result.brightness
        exp_state.target_brightness = 128. 0  # Default target
        exp_state.exposure_time_us = float(result.exposure_time_us)
        exp_state.analogue_gain = result.analogue_gain
        exp_state.digital_gain = result.digital_gain
        exp_state.ae_locked = self.driver.config.ae_enable
        exp_state.motion_detected = False
        exp_state.adjustment_state = 'STABLE'
        self.exposure_state_pub.publish(exp_state)
    
    def _params_callback(self, msg:  CameraParams):
        """Handle camera parameter updates."""
        if msg.ae_enabled: 
            self.driver.set_auto_exposure(True)
        else:
            self.driver.set_auto_exposure(False)
            self.driver.set_exposure(int(msg.exposure_time_us))
            self.driver.set_gain(msg.analogue_gain, msg.digital_gain)
        
        self.get_logger().info('Camera parameters updated')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IMX500CameraNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Failed to start camera node: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
