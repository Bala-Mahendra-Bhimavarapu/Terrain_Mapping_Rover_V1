#!/usr/bin/env python3
"""
ToF Camera ROS 2 Node

Publishes depth images and point clouds from Arducam ToF camera. 

Published Topics:
    - /tof/depth/image_raw (sensor_msgs/Image): Depth image (16UC1, mm)
    - /tof/depth/camera_info (sensor_msgs/CameraInfo): Camera calibration
    - /tof/confidence/image_raw (sensor_msgs/Image): Confidence image (8UC1)
    - /tof/amplitude/image_raw (sensor_msgs/Image): Amplitude/IR image (16UC1)
    - /tof/points (sensor_msgs/PointCloud2): 3D point cloud
    - /tof/depth/image_colored (sensor_msgs/Image): Colorized depth for viz
    - /tof/connected (std_msgs/Bool): Connection status

Services:
    - /tof/set_near_mode (std_srvs/Trigger): Set near depth mode
    - /tof/set_far_mode (std_srvs/Trigger): Set far depth mode

Usage:
    ros2 run tmr_tof_camera tof_camera_node
    ros2 run tmr_tof_camera tof_camera_node --ros-args -p depth_mode:=far
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs. msg import Bool, Float32
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from cv_bridge import CvBridge

import os
import time
import threading
import numpy as np
from typing import Optional

from .arducam_tof_driver import ArducamToFDriver, ToFFrame, DepthMode
from . pointcloud_generator import PointCloudGenerator, CameraIntrinsics
from .depth_processor import DepthProcessor, HoleFillMode


class ToFCameraNode(Node):
    """
    ROS 2 node for Arducam ToF camera. 
    """
    
    def __init__(self):
        super().__init__('tof_camera_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # Initialize components
        self.camera:  Optional[ArducamToFDriver] = None
        self.pointcloud_gen: Optional[PointCloudGenerator] = None
        self.depth_processor: Optional[DepthProcessor] = None
        self.cv_bridge = CvBridge()
        
        self.connected = False
        
        # Statistics
        self.frame_count = 0
        self.last_rate_check_time = time.time()
        self.measured_rate = 0.0
        self.avg_depth = 0.0
        self.valid_points_ratio = 0.0
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publishers
        self.depth_pub = self.create_publisher(
            Image, 'tof/depth/image_raw', sensor_qos)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 'tof/depth/camera_info', sensor_qos)
        self.confidence_pub = self.create_publisher(
            Image, 'tof/confidence/image_raw', sensor_qos)
        self.amplitude_pub = self.create_publisher(
            Image, 'tof/amplitude/image_raw', sensor_qos)
        self.depth_colored_pub = self.create_publisher(
            Image, 'tof/depth/image_colored', sensor_qos)
        self.connected_pub = self.create_publisher(
            Bool, 'tof/connected', 10)
        
        if self.publish_pointcloud:
            self.pointcloud_pub = self.create_publisher(
                PointCloud2, 'tof/points', sensor_qos)
        
        if self.publish_diagnostics:
            self.diag_pub = self.create_publisher(
                DiagnosticArray, '/diagnostics', 10)
        
        # Create services
        self. set_near_srv = self.create_service(
            Trigger, 'tof/set_near_mode', self.set_near_mode_callback)
        self.set_middle_srv = self.create_service(
            Trigger, 'tof/set_middle_mode', self.set_middle_mode_callback)
        self.set_far_srv = self.create_service(
            Trigger, 'tof/set_far_mode', self.set_far_mode_callback)
        self.reset_filter_srv = self.create_service(
            Trigger, 'tof/reset_filter', self.reset_filter_callback)
        
        # Lock for thread safety
        self.lock = threading. Lock()
        
        # Connect to camera
        self._connect()
        
        # Setup timers
        self.capture_timer = self.create_timer(
            1.0 / self.frame_rate, self.capture_callback)
        
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        if self.publish_diagnostics:
            self.diag_timer = self.create_timer(
                1.0 / self. diag_rate, self.diagnostics_callback)
        
        self.get_logger().info('ToF Camera Node initialized')
        self.get_logger().info(f'  Device ID: {self.device_id}')
        self.get_logger().info(f'  Depth Mode: {self.depth_mode}')
        self.get_logger().info(f'  Frame Rate: {self.frame_rate} fps')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info(f'  Publish PointCloud: {self.publish_pointcloud}')
    
    def _declare_parameters(self):
        """Declare all node parameters"""
        # Camera identification
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_id', 'tof_optical_frame')
        self.declare_parameter('camera_name', 'tof_camera')
        
        # Sensor configuration
        self.declare_parameter('depth_mode', 'near')
        self.declare_parameter('frame_rate', 15. 0)
        self.declare_parameter('confidence_threshold', 30)
        
        # Output configuration
        self.declare_parameter('publish_depth_image', True)
        self.declare_parameter('publish_confidence_image', True)
        self.declare_parameter('publish_amplitude_image', True)
        self.declare_parameter('publish_pointcloud', True)
        self.declare_parameter('pointcloud_decimation', 1)
        
        # Depth processing
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('enable_temporal_filter', True)
        self.declare_parameter('temporal_filter_alpha', 0.4)
        self.declare_parameter('enable_spatial_filter', True)
        self.declare_parameter('spatial_filter_size', 3)
        self.declare_parameter('enable_edge_preserving_filter', False)
        self.declare_parameter('hole_filling_mode', 'none')
        
        # Camera intrinsics
        self.declare_parameter('fx', 120.0)
        self.declare_parameter('fy', 120.0)
        self.declare_parameter('cx', 120.0)
        self.declare_parameter('cy', 90.0)
        self.declare_parameter('distortion_coefficients', [0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Point cloud settings
        self.declare_parameter('pointcloud_frame_id', 'tof_optical_frame')
        self.declare_parameter('include_intensity', True)
        self.declare_parameter('organized_pointcloud', False)
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('diagnostics_rate_hz', 1.0)
        self.declare_parameter('frame_rate_warning_threshold', 0.8)
    
    def _get_parameters(self):
        """Get all parameter values"""
        # Camera identification
        self.device_id = self.get_parameter('device_id').value
        self.frame_id = self. get_parameter('frame_id').value
        self.camera_name = self.get_parameter('camera_name').value
        
        # Sensor configuration
        depth_mode_str = self.get_parameter('depth_mode').value
        self.depth_mode = self._parse_depth_mode(depth_mode_str)
        self.frame_rate = self.get_parameter('frame_rate').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Output configuration
        self. publish_depth_image = self. get_parameter('publish_depth_image').value
        self.publish_confidence_image = self.get_parameter('publish_confidence_image').value
        self.publish_amplitude_image = self.get_parameter('publish_amplitude_image').value
        self.publish_pointcloud = self.get_parameter('publish_pointcloud').value
        self.pointcloud_decimation = self.get_parameter('pointcloud_decimation').value
        
        # Depth processing
        self.min_depth = self.get_parameter('min_depth_m').value
        self.max_depth = self.get_parameter('max_depth_m').value
        self.enable_temporal = self.get_parameter('enable_temporal_filter').value
        self.temporal_alpha = self.get_parameter('temporal_filter_alpha').value
        self.enable_spatial = self. get_parameter('enable_spatial_filter').value
        self.spatial_size = self.get_parameter('spatial_filter_size').value
        self.enable_edge_preserving = self.get_parameter('enable_edge_preserving_filter').value
        hole_fill_str = self.get_parameter('hole_filling_mode').value
        self.hole_fill_mode = self._parse_hole_fill_mode(hole_fill_str)
        
        # Camera intrinsics
        self.fx = self.get_parameter('fx').value
        self. fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self. cy = self.get_parameter('cy').value
        self.distortion = list(self.get_parameter('distortion_coefficients').value)
        
        # Point cloud settings
        self.pointcloud_frame_id = self.get_parameter('pointcloud_frame_id').value
        self.include_intensity = self.get_parameter('include_intensity').value
        self.organized_pointcloud = self.get_parameter('organized_pointcloud').value
        
        # Diagnostics
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.diag_rate = self.get_parameter('diagnostics_rate_hz').value
        self.rate_warning = self.get_parameter('frame_rate_warning_threshold').value
    
    def _parse_depth_mode(self, mode_str: str) -> DepthMode:
        """Parse depth mode string to enum"""
        mode_map = {
            'near': DepthMode.NEAR,
            'middle': DepthMode.MIDDLE,
            'far': DepthMode. FAR,
        }
        return mode_map. get(mode_str.lower(), DepthMode.NEAR)
    
    def _parse_hole_fill_mode(self, mode_str: str) -> HoleFillMode: 
        """Parse hole fill mode string to enum"""
        mode_map = {
            'none': HoleFillMode. NONE,
            'nearest': HoleFillMode. NEAREST,
            'linear': HoleFillMode.LINEAR,
        }
        return mode_map. get(mode_str.lower(), HoleFillMode.NONE)
    
    def _connect(self):
        """Connect to the ToF camera and initialize components"""
        self.get_logger().info('Connecting to Arducam ToF camera...')
        
        try:
            # Create camera driver
            self.camera = ArducamToFDriver(
                device_id=self.device_id,
                depth_mode=self.depth_mode,
                frame_rate=self.frame_rate
            )
            
            if self.camera.connect():
                self.connected = True
                self.get_logger().info('Connected to ToF camera successfully')
                
                # Get depth range
                min_range, max_range = self.camera. get_depth_range()
                self.get_logger().info(f'  Depth range: {min_range}m - {max_range}m')
            else:
                self.connected = False
                self.get_logger().error('Failed to connect to ToF camera')
                
        except Exception as e:
            self.connected = False
            self. get_logger().error(f'Exception connecting to ToF camera: {e}')
        
        # Initialize depth processor
        self.depth_processor = DepthProcessor(
            temporal_filter_alpha=self.temporal_alpha,
            spatial_filter_size=self.spatial_size,
            min_depth=self.min_depth,
            max_depth=self.max_depth
        )
        
        # Initialize point cloud generator
        intrinsics = CameraIntrinsics(
            fx=self. fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy,
            width=ArducamToFDriver.WIDTH,
            height=ArducamToFDriver.HEIGHT
        )
        
        self.pointcloud_gen = PointCloudGenerator(
            intrinsics=intrinsics,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
            decimation=self.pointcloud_decimation
        )
    
    def capture_callback(self):
        """Timer callback to capture and publish ToF data"""
        if not self.connected or self.camera is None:
            return
        
        try:
            # Capture frame
            frame = self. camera.capture()
            
            if frame is None:
                return
            
            # Update statistics
            self.frame_count += 1
            
            # Get current timestamp
            now = self.get_clock().now()
            stamp = now.to_msg()
            
            # Convert depth to meters for processing
            depth_m = frame.depth_image.astype(np.float32) * frame.depth_scale
            
            # Apply depth processing
            if self.depth_processor is not None:
                depth_processed = self.depth_processor.process(
                    depth_m,
                    confidence=frame.confidence_image,
                    confidence_threshold=self.confidence_threshold,
                    enable_temporal=self.enable_temporal,
                    enable_spatial=self.enable_spatial,
                    hole_fill_mode=self.hole_fill_mode
                )
            else:
                depth_processed = depth_m
            
            # Calculate statistics
            valid_mask = (depth_processed > 0) & ~np.isnan(depth_processed)
            if valid_mask.any():
                self.avg_depth = float(np.mean(depth_processed[valid_mask]))
                self.valid_points_ratio = float(np.sum(valid_mask)) / valid_mask.size
            
            # Publish depth image
            if self.publish_depth_image:
                self._publish_depth_image(depth_processed, stamp)
            
            # Publish camera info
            self._publish_camera_info(stamp)
            
            # Publish confidence image
            if self.publish_confidence_image:
                self._publish_confidence_image(frame.confidence_image, stamp)
            
            # Publish amplitude image
            if self.publish_amplitude_image:
                self._publish_amplitude_image(frame.amplitude_image, stamp)
            
            # Publish colorized depth
            self._publish_colored_depth(depth_processed, stamp)
            
            # Publish point cloud
            if self.publish_pointcloud:
                self._publish_pointcloud(
                    depth_processed, frame.confidence_image,
                    frame.amplitude_image, stamp
                )
            
        except Exception as e:
            self.get_logger().error(f'Capture error: {e}')
    
    def _publish_depth_image(self, depth_m: np.ndarray, stamp):
        """Publish depth image as 16UC1 (millimeters)"""
        try:
            # Convert to millimeters uint16
            depth_mm = DepthProcessor.depth_to_uint16(depth_m, scale=1000.0)
            
            # Convert to ROS message
            msg = self.cv_bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            
            self.depth_pub.publish(msg)
            
        except Exception as e: 
            self.get_logger().error(f'Failed to publish depth image: {e}')
    
    def _publish_camera_info(self, stamp):
        """Publish camera info message"""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        msg.width = ArducamToFDriver. WIDTH
        msg.height = ArducamToFDriver.HEIGHT
        
        msg.distortion_model = 'plumb_bob'
        msg. d = self.distortion
        
        # Camera matrix K
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        # Rectification matrix R (identity)
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix P
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.camera_info_pub.publish(msg)
    
    def _publish_confidence_image(self, confidence:  np.ndarray, stamp):
        """Publish confidence image as 8UC1"""
        try: 
            msg = self.cv_bridge.cv2_to_imgmsg(confidence, encoding='8UC1')
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            
            self.confidence_pub. publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish confidence image: {e}')
    
    def _publish_amplitude_image(self, amplitude: np.ndarray, stamp):
        """Publish amplitude image as 16UC1"""
        try: 
            msg = self.cv_bridge.cv2_to_imgmsg(amplitude, encoding='16UC1')
            msg.header. stamp = stamp
            msg.header.frame_id = self.frame_id
            
            self. amplitude_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish amplitude image: {e}')
    
    def _publish_colored_depth(self, depth_m: np.ndarray, stamp):
        """Publish colorized depth image for visualization"""
        try:
            # Colorize depth
            colored = DepthProcessor.colorize_depth(
                depth_m, self.min_depth, self.max_depth
            )
            
            msg = self.cv_bridge.cv2_to_imgmsg(colored, encoding='bgr8')
            msg.header.stamp = stamp
            msg. header.frame_id = self. frame_id
            
            self.depth_colored_pub.publish(msg)
            
        except Exception as e: 
            self.get_logger().error(f'Failed to publish colored depth: {e}')
    
    def _publish_pointcloud(
        self,
        depth_m: np. ndarray,
        confidence: np.ndarray,
        amplitude: np.ndarray,
        stamp
    ):
        """Generate and publish point cloud"""
        try:
            if self.pointcloud_gen is None:
                return
            
            # Generate point cloud message
            pc_msg = self.pointcloud_gen.to_pointcloud2(
                depth_image=depth_m,
                depth_scale=1.0,  # Already in meters
                frame_id=self.pointcloud_frame_id,
                stamp=stamp,
                confidence_image=confidence,
                confidence_threshold=self.confidence_threshold,
                intensity_image=amplitude if self.include_intensity else None,
                include_intensity=self.include_intensity
            )
            
            if pc_msg is not None: 
                self.pointcloud_pub.publish(pc_msg)
            
        except Exception as e: 
            self.get_logger().error(f'Failed to publish point cloud: {e}')
    
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
    
    def set_near_mode_callback(self, request, response):
        """Service callback to set near depth mode"""
        try:
            if self.camera is not None:
                self.camera. set_depth_mode(DepthMode.NEAR)
                self.depth_processor. reset()
                response.success = True
                response. message = 'Set to NEAR mode (0.2m - 1.2m)'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Camera not connected'
        except Exception as e:
            response.success = False
            response. message = f'Failed:  {e}'
        return response
    
    def set_middle_mode_callback(self, request, response):
        """Service callback to set middle depth mode"""
        try:
            if self. camera is not None:
                self.camera.set_depth_mode(DepthMode.MIDDLE)
                self.depth_processor.reset()
                response.success = True
                response.message = 'Set to MIDDLE mode (0.5m - 3.0m)'
                self. get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Camera not connected'
        except Exception as e: 
            response.success = False
            response.message = f'Failed: {e}'
        return response
    
    def set_far_mode_callback(self, request, response):
        """Service callback to set far depth mode"""
        try:
            if self.camera is not None:
                self. camera.set_depth_mode(DepthMode.FAR)
                self.depth_processor. reset()
                response.success = True
                response.message = 'Set to FAR mode (1.0m - 4.0m)'
                self. get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Camera not connected'
        except Exception as e: 
            response.success = False
            response.message = f'Failed: {e}'
        return response
    
    def reset_filter_callback(self, request, response):
        """Service callback to reset depth filter"""
        try:
            if self.depth_processor is not None:
                self.depth_processor.reset()
                response.success = True
                response.message = 'Depth filter reset'
                self. get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Depth processor not initialized'
        except Exception as e:
            response.success = False
            response.message = f'Failed: {e}'
        return response
    
    def diagnostics_callback(self):
        """Timer callback to publish diagnostics"""
        if not self.publish_diagnostics:
            return
        
        # Create diagnostic message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'ToF Camera/Arducam B0410'
        status.hardware_id = f'tof_{self.device_id}'
        
        # Determine overall status
        if not self.connected:
            status. level = DiagnosticStatus.ERROR
            status.message = 'ToF camera disconnected'
        elif self.measured_rate < self.frame_rate * self.rate_warning:
            status.level = DiagnosticStatus. WARN
            status.message = f'Low frame rate:  {self.measured_rate:.1f} fps'
        elif self.valid_points_ratio < 0.3:
            status.level = DiagnosticStatus.WARN
            status.message = f'Low valid points:  {self.valid_points_ratio*100:.1f}%'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'OK - {self.measured_rate:. 1f} fps'
        
        # Get depth range
        if self.camera is not None:
            min_range, max_range = self.camera.get_depth_range()
        else:
            min_range, max_range = 0, 0
        
        # Add key-value pairs
        status.values = [
            KeyValue(key='connected', value=str(self.connected)),
            KeyValue(key='frame_rate_actual', value=f'{self.measured_rate:.1f}'),
            KeyValue(key='frame_rate_target', value=f'{self.frame_rate:.1f}'),
            KeyValue(key='depth_mode', value=self.depth_mode.value if hasattr(self.depth_mode, 'value') else str(self.depth_mode)),
            KeyValue(key='depth_range_min_m', value=f'{min_range:.2f}'),
            KeyValue(key='depth_range_max_m', value=f'{max_range:.2f}'),
            KeyValue(key='avg_depth_m', value=f'{self. avg_depth:.3f}'),
            KeyValue(key='valid_points_ratio', value=f'{self.valid_points_ratio:.2f}'),
            KeyValue(key='resolution', value=f'{ArducamToFDriver.WIDTH}x{ArducamToFDriver.HEIGHT}'),
            KeyValue(key='total_frames', value=str(self.camera.get_frame_count() if self.camera else 0)),
        ]
        
        diag_msg.status. append(status)
        self.diag_pub.publish(diag_msg)
    
    def destroy_node(self):
        """Clean up on node shutdown"""
        self.get_logger().info('Shutting down ToF Camera node...')
        
        if self.camera is not None:
            self.camera.disconnect()
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy. init(args=args)
    
    node = ToFCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
