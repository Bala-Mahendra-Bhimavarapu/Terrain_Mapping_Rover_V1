#!/usr/bin/env python3
"""
ToF Camera Diagnostics Tool

Real-time monitoring and visualization of ToF camera data. 

Usage:
    ros2 run tmr_tof_camera tof_diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool

import time
import sys
import struct
from collections import deque

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError: 
    OPENCV_AVAILABLE = False


class ToFDiagnosticsNode(Node):
    """
    Diagnostic node for monitoring ToF camera data quality. 
    """
    
    def __init__(self):
        super().__init__('tof_diagnostics')
        
        # Parameters
        self.declare_parameter('show_images', True)
        self.declare_parameter('window_scale', 2.0)
        
        self.show_images = self.get_parameter('show_images').value
        self.window_scale = self.get_parameter('window_scale').value
        
        # Data tracking
        self.frame_count = 0
        self.pointcloud_count = 0
        self.last_rate_time = time.time()
        self.depth_rate = 0.0
        self.pc_rate = 0.0
        
        # Depth statistics
        self.min_depth = 0.0
        self.max_depth = 0.0
        self.avg_depth = 0.0
        self.valid_ratio = 0.0
        
        # Point cloud statistics
        self.num_points = 0
        
        # Status
        self.connected = False
        
        # Latest images
        self.latest_depth = None
        self.latest_confidence = None
        self.latest_colored = None
        
        # Create QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, '/tof/depth/image_raw', self.depth_callback, sensor_qos)
        self.confidence_sub = self.create_subscription(
            Image, '/tof/confidence/image_raw', self.confidence_callback, sensor_qos)
        self.colored_sub = self.create_subscription(
            Image, '/tof/depth/image_colored', self.colored_callback, sensor_qos)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/tof/points', self.pointcloud_callback, sensor_qos)
        self.conn_sub = self.create_subscription(
            Bool, '/tof/connected', self.connected_callback, 10)
        
        # Display timer
        self.display_timer = self.create_timer(0.1, self.display_callback)
        
        # Console update timer
        self.console_timer = self.create_timer(0.5, self.console_callback)
        
        self.get_logger().info('ToF Diagnostics started')
        self.get_logger().info('Monitoring /tof/* topics.. .')
        
        if self.show_images and OPENCV_AVAILABLE:
            cv2.namedWindow('ToF Depth', cv2.WINDOW_NORMAL)
            cv2.namedWindow('ToF Confidence', cv2.WINDOW_NORMAL)
    
    def depth_callback(self, msg:  Image):
        """Process depth image message"""
        self.frame_count += 1
        
        try:
            # Convert to numpy array (16UC1 = uint16, mm)
            depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            
            # Convert to meters
            depth_m = depth_mm. astype(np.float32) / 1000.0
            
            # Calculate statistics
            valid = depth_m > 0
            if valid.any():
                self.min_depth = float(np.min(depth_m[valid]))
                self.max_depth = float(np.max(depth_m[valid]))
                self.avg_depth = float(np.mean(depth_m[valid]))
                self.valid_ratio = float(np.sum(valid)) / valid.size
            
            self.latest_depth = depth_m
            
        except Exception as e:
            self.get_logger().error(f'Depth processing error: {e}')
    
    def confidence_callback(self, msg: Image):
        """Process confidence image message"""
        try:
            confidence = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg. height, msg.width)
            self.latest_confidence = confidence
        except Exception as e:
            self.get_logger().error(f'Confidence processing error: {e}')
    
    def colored_callback(self, msg: Image):
        """Process colored depth image"""
        try:
            # BGR8 format
            colored = np.frombuffer(msg. data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            self.latest_colored = colored
        except Exception as e:
            self.get_logger().error(f'Colored depth processing error: {e}')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process point cloud message"""
        self.pointcloud_count += 1
        self.num_points = msg.width * msg.height
    
    def connected_callback(self, msg: Bool):
        """Process connection status"""
        self. connected = msg.data
    
    def display_callback(self):
        """Display images"""
        if not self.show_images or not OPENCV_AVAILABLE: 
            return
        
        # Display colored depth
        if self.latest_colored is not None:
            display = self.latest_colored.copy()
            
            # Scale up for visibility (ToF is 240x180)
            h, w = display.shape[:2]
            new_h = int(h * self.window_scale)
            new_w = int(w * self.window_scale)
            display = cv2.resize(display, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
            
            # Add text overlay
            cv2.putText(display, f'Avg: {self.avg_depth:. 2f}m', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display, f'Range: {self.min_depth:. 2f}-{self.max_depth:.2f}m', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display, f'Valid: {self.valid_ratio*100:.1f}%', (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('ToF Depth', display)
        
        # Display confidence
        if self.latest_confidence is not None:
            conf_display = self.latest_confidence. copy()
            
            # Scale up
            h, w = conf_display.shape[:2]
            new_h = int(h * self.window_scale)
            new_w = int(w * self.window_scale)
            conf_display = cv2.resize(conf_display, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
            
            # Convert to color for display
            conf_color = cv2.applyColorMap(conf_display, cv2.COLORMAP_VIRIDIS)
            
            cv2.imshow('ToF Confidence', conf_color)
        
        cv2.waitKey(1)
    
    def console_callback(self):
        """Update console display"""
        # Calculate rates
        current_time = time. time()
        elapsed = current_time - self.last_rate_time
        
        if elapsed > 0:
            self.depth_rate = self.frame_count / elapsed
            self.pc_rate = self.pointcloud_count / elapsed
        
        self.frame_count = 0
        self.pointcloud_count = 0
        self.last_rate_time = current_time
        
        # Clear screen and display
        print('\033[2J\033[H', end='')  # Clear screen
        
        print('=' * 60)
        print('           TOF CAMERA DIAGNOSTICS')
        print('=' * 60)
        
        # Connection status
        status_str = '\033[92mCONNECTED\033[0m' if self. connected else '\033[91mDISCONNECTED\033[0m'
        print(f'\nStatus: {status_str}')
        print(f'Resolution: 240 x 180')
        
        print('\n' + '-' * 60)
        print('FRAME RATES')
        print('-' * 60)
        print(f'  Depth Images:   {self.depth_rate:.1f} fps')
        print(f'  Point Clouds:  {self.pc_rate:.1f} fps')
        
        print('\n' + '-' * 60)
        print('DEPTH STATISTICS')
        print('-' * 60)
        print(f'  Minimum Depth: {self.min_depth:.3f} m')
        print(f'  Maximum Depth: {self.max_depth:.3f} m')
        print(f'  Average Depth: {self.avg_depth:.3f} m')
        print(f'  Valid Pixels:   {self.valid_ratio * 100:.1f}%')
        
        print('\n' + '-' * 60)
        print('POINT CLOUD')
        print('-' * 60)
        print(f'  Number of Points: {self.num_points}')
        
        # Quality indicators
        print('\n' + '-' * 60)
        print('QUALITY INDICATORS')
        print('-' * 60)
        
        if self.depth_rate > 12: 
            print('  Frame Rate:     \033[92m● Good\033[0m')
        elif self.depth_rate > 8:
            print('  Frame Rate:    \033[93m● Low\033[0m')
        else:
            print('  Frame Rate:    \033[91m● Very Low\033[0m')
        
        if self.valid_ratio > 0.7:
            print('  Valid Pixels:  \033[92m● Good\033[0m')
        elif self.valid_ratio > 0.4:
            print('  Valid Pixels:  \033[93m● Moderate\033[0m')
        else:
            print('  Valid Pixels:  \033[91m● Low\033[0m')
        
        if 0.2 < self.avg_depth < 3. 0:
            print('  Depth Range:   \033[92m● In Range\033[0m')
        else:
            print('  Depth Range:   \033[93m● At Limits\033[0m')
        
        print('\n' + '=' * 60)
        print('Press Ctrl+C to exit')
        if self.show_images and OPENCV_AVAILABLE:
            print('Press Q in image windows to close them')
    
    def destroy_node(self):
        """Clean up"""
        if self.show_images and OPENCV_AVAILABLE:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = ToFDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()