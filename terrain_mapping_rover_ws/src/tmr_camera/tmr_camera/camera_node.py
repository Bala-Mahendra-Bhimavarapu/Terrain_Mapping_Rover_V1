#!/usr/bin/env python3
"""
Camera ROS 2 Node

Publishes images from IMX500 camera. 

Published Topics:
    - /camera/image_raw (sensor_msgs/Image): Raw camera images
    - /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info
    - /camera/image_raw/compressed (sensor_msgs/CompressedImage): Compressed images
    - /camera/connected (std_msgs/Bool): Connection status
    - /camera/brightness (std_msgs/Float32): Image brightness level
    - /camera/exposure_time (std_msgs/Float32): Current exposure time

Services:
    - /camera/save_image (std_srvs/Trigger): Save current image to file
    - /camera/trigger_calibration (std_srvs/Trigger): Trigger camera calibration

Subscriptions:
    - /camera/exposure_control (tmr_msgs/ExposureControl): Exposure control commands

Usage:
    ros2 run tmr_camera camera_node
    ros2 run tmr_camera camera_node --ros-args -p width:=1920 -p height:=1080
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs. msg import Bool, Float32
from std_srvs.srv import Trigger
from diagnostic_msgs. msg import DiagnosticArray, DiagnosticStatus, KeyValue

from cv_bridge import CvBridge

import os
import time
import threading
import numpy as np
from typing import Optional
from datetime import datetime

from . imx500_driver import (
    IMX500Driver, CameraConfig, CameraFrame, 
    ExposureMode, AWBMode, DenoiseMode
)
from .camera_info_manager import CameraInfoManager
from .image_processor import ImageProcessor

# Try to import custom messages
try:
    from tmr_msgs.msg import ExposureControl
    TMR_MSGS_AVAILABLE = True
except ImportError:
    TMR_MSGS_AVAILABLE = False


class CameraNode(Node):
    """
    ROS 2 node for IMX500 camera.
    """
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # Create camera config
        self. camera_config = self._create_camera_config()
        
        # Initialize components
        self.camera:  Optional[IMX500Driver] = None
        self.camera_info_manager: Optional[CameraInfoManager] = None
        self. image_processor: Optional[ImageProcessor] = None
        self.cv_bridge = CvBridge()
        
        self.connected = False
        
        # Statistics
        self.frame_count = 0
        self.last_rate_check_time = time.time()
        self.measured_rate = 0.0
        self.last_brightness = 128.0
        self.last_exposure_us = 20000
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy. BEST_EFFORT,
            history=HistoryPolicy. KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy. KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', sensor_qos)
        self.camera_info_pub = self. create_publisher(CameraInfo, 'camera/camera_info', sensor_qos)
        self.connected_pub = self.create_publisher(Bool, 'camera/connected', 10)
        self.brightness_pub = self.create_publisher(Float32, 'camera/brightness', 10)
        self.exposure_pub = self.create_publisher(Float32, 'camera/exposure_time', 10)
        
        if self.enable_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage, 'camera/image_raw/compressed', sensor_qos)
        
        if self.publish_diagnostics:
            self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Create subscribers
        if TMR_MSGS_AVAILABLE:
            self.exposure_control_sub = self.create_subscription(
                ExposureControl, 'camera/exposure_control',
                self.exposure_control_callback, 10)
        
        # Create services
        self. save_image_srv = self. create_service(
            Trigger, 'camera/save_image', self.save_image_callback)
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Latest frame for saving
        self.latest_frame: Optional[CameraFrame] = None
        
        # Connect to camera
        self._connect()
        
        # Setup timers
        # Main capture/publish timer
        self.capture_timer = self.create_timer(
            1.0 / self.frame_rate, self. capture_callback)
        
        # Connection status timer
        self. status_timer = self.create_timer(1.0, self. status_callback)
        
        # Diagnostics timer
        if self.publish_diagnostics:
            self.diag_timer = self.create_timer(
                1.0 / self.diag_rate, self.diagnostics_callback)
        
        self.get_logger().info('Camera Node initialized')
        self.get_logger().info(f'  Resolution: {self.width}x{self.height}')
        self.get_logger().info(f'  Frame rate: {self. frame_rate} fps')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info(f'  Encoding: {self.encoding}')
    
    def _declare_parameters(self):
        """Declare all node parameters"""
        # Camera identification
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('camera_name', 'imx500')
        
        # Resolution and frame rate
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('frame_rate', 30.0)
        
        # Image format
        self.declare_parameter('encoding', 'bgr8')
        self.declare_parameter('enable_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        
        # Exposure
        self.declare_parameter('exposure_mode', 'auto')
        self.declare_parameter('exposure_time_us', 20000)
        self.declare_parameter('exposure_compensation', 0.0)
        self.declare_parameter('gain_mode', 'auto')
        self.declare_parameter('analog_gain', 1.0)
        
        # White balance
        self.declare_parameter('awb_mode', 'auto')
        self.declare_parameter('awb_gain_red', 1.5)
        self.declare_parameter('awb_gain_blue', 1.5)
        
        # Image quality
        self.declare_parameter('sharpness', 0.0)
        self.declare_parameter('contrast', 1.0)
        self.declare_parameter('brightness', 0.0)
        self.declare_parameter('saturation', 1.0)
        self.declare_parameter('denoise_mode', 'fast')
        
        # Calibration
        self.declare_parameter('camera_matrix', 
            [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0])
        self.declare_parameter('distortion_coefficients', 
            [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('distortion_model', 'plumb_bob')
        self.declare_parameter('calibration_file', '')
        
        # Performance
        self.declare_parameter('buffer_count', 4)
        self.declare_parameter('use_hardware_acceleration', True)
        self.declare_parameter('drop_frames', True)
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('diagnostics_rate_hz', 1.0)
        self.declare_parameter('frame_rate_warning_threshold', 0.9)
        
        # Save directory for captured images
        self.declare_parameter('save_directory', '~/camera_captures')
    
    def _get_parameters(self):
        """Get all parameter values"""
        # Camera identification
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_name = self.get_parameter('camera_name').value
        
        # Resolution and frame rate
        self.width = self.get_parameter('width').value
        self.height = self. get_parameter('height').value
        self.frame_rate = self.get_parameter('frame_rate').value
        
        # Image format
        self.encoding = self.get_parameter('encoding').value
        self.enable_compressed = self.get_parameter('enable_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Exposure
        exposure_mode_str = self.get_parameter('exposure_mode').value
        self.exposure_mode = ExposureMode. AUTO if exposure_mode_str == 'auto' else ExposureMode. MANUAL
        self.exposure_time_us = self.get_parameter('exposure_time_us').value
        self.exposure_compensation = self.get_parameter('exposure_compensation').value
        gain_mode_str = self.get_parameter('gain_mode').value
        self.gain_mode = ExposureMode.AUTO if gain_mode_str == 'auto' else ExposureMode.MANUAL
        self.analog_gain = self.get_parameter('analog_gain').value
        
        # White balance
        awb_mode_str = self.get_parameter('awb_mode').value
        self.awb_mode = self._parse_awb_mode(awb_mode_str)
        self.awb_gain_red = self.get_parameter('awb_gain_red').value
        self.awb_gain_blue = self.get_parameter('awb_gain_blue').value
        
        # Image quality
        self.sharpness = self.get_parameter('sharpness').value
        self.contrast = self. get_parameter('contrast').value
        self.brightness_setting = self.get_parameter('brightness').value
        self.saturation = self.get_parameter('saturation').value
        denoise_str = self.get_parameter('denoise_mode').value
        self.denoise_mode = self._parse_denoise_mode(denoise_str)
        
        # Calibration
        self.camera_matrix = list(self.get_parameter('camera_matrix').value)
        self.distortion_coefficients = list(self.get_parameter('distortion_coefficients').value)
        self.distortion_model = self.get_parameter('distortion_model').value
        self. calibration_file = self.get_parameter('calibration_file').value
        
        # Performance
        self.buffer_count = self.get_parameter('buffer_count').value
        self.use_hardware_acceleration = self.get_parameter('use_hardware_acceleration').value
        self.drop_frames = self.get_parameter('drop_frames').value
        
        # Diagnostics
        self. publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.diag_rate = self.get_parameter('diagnostics_rate_hz').value
        self.rate_warning = self.get_parameter('frame_rate_warning_threshold').value
        
        # Save directory
        self.save_directory = os.path.expanduser(
            self.get_parameter('save_directory').value)
    
    def _parse_awb_mode(self, mode_str: str) -> AWBMode:
        """Parse AWB mode string to enum"""
        mode_map = {
            'auto': AWBMode.AUTO,
            'manual': AWBMode.MANUAL,
            'daylight': AWBMode.DAYLIGHT,
            'cloudy': AWBMode.CLOUDY,
            'tungsten': AWBMode.TUNGSTEN,
            'fluorescent': AWBMode.FLUORESCENT,
        }
        return mode_map. get(mode_str.lower(), AWBMode.AUTO)
    
    def _parse_denoise_mode(self, mode_str: str) -> DenoiseMode:
        """Parse denoise mode string to enum"""
        mode_map = {
            'off': DenoiseMode.OFF,
            'fast': DenoiseMode.FAST,
            'high_quality': DenoiseMode.HIGH_QUALITY,
        }
        return mode_map. get(mode_str.lower(), DenoiseMode. FAST)
    
    def _create_camera_config(self) -> CameraConfig:
        """Create camera configuration from parameters"""
        return CameraConfig(
            width=self.width,
            height=self.height,
            frame_rate=self.frame_rate,
            encoding=self.encoding,
            exposure_mode=self.exposure_mode,
            exposure_time_us=self.exposure_time_us,
            exposure_compensation=self.exposure_compensation,
            gain_mode=self.gain_mode,
            analog_gain=self.analog_gain,
            awb_mode=self.awb_mode,
            awb_gain_red=self.awb_gain_red,
            awb_gain_blue=self.awb_gain_blue,
            sharpness=self.sharpness,
            contrast=self.contrast,
            brightness=self.brightness_setting,
            saturation=self. saturation,
            denoise_mode=self.denoise_mode,
            buffer_count=self.buffer_count,
            use_hardware_acceleration=self.use_hardware_acceleration,
        )
    
    def _connect(self):
        """Connect to the camera and initialize components"""
        self.get_logger().info('Connecting to IMX500 camera...')
        
        try:
            # Create camera driver
            self.camera = IMX500Driver(
                camera_index=self.camera_index,
                config=self.camera_config
            )
            
            if self.camera. connect():
                self.connected = True
                self.get_logger().info('Connected to camera successfully')
                
                # Start camera
                if self.camera.start():
                    self.get_logger().info('Camera streaming started')
                else:
                    self.get_logger().error('Failed to start camera streaming')
            else:
                self.connected = False
                self.get_logger().error('Failed to connect to camera')
                
        except Exception as e:
            self.connected = False
            self.get_logger().error(f'Exception connecting to camera: {e}')
        
        # Initialize camera info manager
        self.camera_info_manager = CameraInfoManager(
            camera_name=self.camera_name,
            frame_id=self.frame_id,
            width=self.width,
            height=self.height
        )
        
        # Load calibration
        if self.calibration_file and os.path.exists(self. calibration_file):
            self.camera_info_manager.load_calibration_file(self.calibration_file)
        else:
            self.camera_info_manager.set_calibration_from_params(
                self.camera_matrix,
                self.distortion_coefficients,
                self.distortion_model
            )
        
        # Initialize image processor
        self.image_processor = ImageProcessor()
    
    def capture_callback(self):
        """Timer callback to capture and publish images"""
        if not self.connected or self.camera is None:
            return
        
        try:
            # Capture frame
            frame = self. camera.capture()
            
            if frame is None:
                return
            
            # Store for saving
            with self.lock:
                self. latest_frame = frame
            
            # Update statistics
            self.frame_count += 1
            self.last_exposure_us = frame.exposure_time_us
            
            # Calculate brightness
            if self.image_processor is not None:
                self.last_brightness = self.image_processor.calculate_brightness(frame. image)
            
            # Get current timestamp
            now = self.get_clock().now()
            stamp = now.to_msg()
            
            # Publish raw image
            self._publish_image(frame. image, stamp)
            
            # Publish camera info
            camera_info_msg = self.camera_info_manager.get_camera_info(stamp)
            self.camera_info_pub.publish(camera_info_msg)
            
            # Publish compressed image
            if self.enable_compressed:
                self._publish_compressed(frame.image, stamp)
            
            # Publish brightness
            brightness_msg = Float32()
            brightness_msg.data = self.last_brightness
            self.brightness_pub.publish(brightness_msg)
            
            # Publish exposure time
            exposure_msg = Float32()
            exposure_msg.data = float(self.last_exposure_us)
            self.exposure_pub. publish(exposure_msg)
            
        except Exception as e: 
            self.get_logger().error(f'Capture error: {e}')
    
    def _publish_image(self, image: np.ndarray, stamp):
        """Convert and publish image message"""
        try:
            # Convert numpy array to ROS Image message
            if self.encoding == 'bgr8':
                msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
            elif self.encoding == 'rgb8':
                msg = self. cv_bridge.cv2_to_imgmsg(image, encoding='rgb8')
            elif self.encoding == 'mono8':
                if len(image.shape) == 3:
                    import cv2
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                else:
                    gray = image
                msg = self.cv_bridge.cv2_to_imgmsg(gray, encoding='mono8')
            else:
                msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
            
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            
            self.image_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')
    
    def _publish_compressed(self, image: np. ndarray, stamp):
        """Publish compressed image"""
        try:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg. format = 'jpeg'
            
            # Compress image
            compressed_data = self.image_processor.compress_jpeg(
                image, self.jpeg_quality)
            msg.data = list(compressed_data)
            
            self.compressed_pub.publish(msg)
            
        except Exception as e:
            self. get_logger().error(f'Failed to publish compressed image: {e}')
    
    def status_callback(self):
        """Timer callback to publish connection status"""
        # Calculate actual frame rate
        current_time = time.time()
        elapsed = current_time - self.last_rate_check_time
        
        if elapsed > 0:
            self.measured_rate = self.frame_count / elapsed
        
        self.frame_count = 0
        self.last_rate_check_time = current_time
        
        # Publish connection status
        conn_msg = Bool()
        conn_msg.data = self.connected
        self. connected_pub.publish(conn_msg)
        
        # Try to reconnect if disconnected
        if not self.connected:
            self._connect()
    
    def exposure_control_callback(self, msg):
        """Handle exposure control commands"""
        if not self.connected or self.camera is None:
            return
        
        try: 
            if msg.auto_exposure:
                self.camera.set_exposure(
                    mode=ExposureMode.AUTO,
                    exposure_us=self.exposure_time_us,
                    gain=self.analog_gain
                )
                self.get_logger().debug('Set auto exposure')
            else:
                self.camera.set_exposure(
                    mode=ExposureMode.MANUAL,
                    exposure_us=int(msg.exposure_time_us),
                    gain=msg.analog_gain
                )
                self.get_logger().debug(
                    f'Set manual exposure:  {msg.exposure_time_us}us, gain={msg.analog_gain}')
                
        except Exception as e: 
            self.get_logger().error(f'Failed to set exposure: {e}')
    
    def save_image_callback(self, request, response):
        """Service callback to save current image"""
        try:
            with self.lock:
                if self.latest_frame is None:
                    response.success = False
                    response.message = 'No image available'
                    return response
                
                frame = self.latest_frame
            
            # Create save directory if needed
            os.makedirs(self.save_directory, exist_ok=True)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = f'capture_{timestamp}.jpg'
            filepath = os.path.join(self.save_directory, filename)
            
            # Save image
            import cv2
            cv2.imwrite(filepath, frame.image)
            
            response.success = True
            response.message = f'Saved image to:  {filepath}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response. message = f'Failed to save image: {e}'
            self. get_logger().error(response.message)
        
        return response
    
    def diagnostics_callback(self):
        """Timer callback to publish diagnostics"""
        if not self.publish_diagnostics:
            return
        
        # Create diagnostic message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'Camera/IMX500'
        status.hardware_id = f'camera_{self.camera_index}'
        
        # Determine overall status
        if not self.connected:
            status. level = DiagnosticStatus.ERROR
            status.message = 'Camera disconnected'
        elif self.measured_rate < self.frame_rate * self.rate_warning:
            status.level = DiagnosticStatus. WARN
            status.message = f'Low frame rate: {self. measured_rate:.1f} fps'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'OK - {self.measured_rate:.1f} fps'
        
        # Add key-value pairs
        status.values = [
            KeyValue(key='connected', value=str(self.connected)),
            KeyValue(key='frame_rate_actual', value=f'{self.measured_rate:.1f}'),
            KeyValue(key='frame_rate_target', value=f'{self.frame_rate:.1f}'),
            KeyValue(key='resolution', value=f'{self.width}x{self.height}'),
            KeyValue(key='encoding', value=self.encoding),
            KeyValue(key='brightness', value=f'{self.last_brightness:.1f}'),
            KeyValue(key='exposure_us', value=str(self.last_exposure_us)),
            KeyValue(key='total_frames', value=str(self.camera. get_frame_count() if self.camera else 0)),
        ]
        
        diag_msg.status. append(status)
        self.diag_pub.publish(diag_msg)
    
    def destroy_node(self):
        """Clean up on node shutdown"""
        self.get_logger().info('Shutting down Camera node...')
        
        if self.camera is not None:
            self.camera.stop()
            self.camera.disconnect()
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
