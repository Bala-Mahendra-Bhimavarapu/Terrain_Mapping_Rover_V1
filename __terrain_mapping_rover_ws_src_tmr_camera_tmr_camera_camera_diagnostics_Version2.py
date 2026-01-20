#!/usr/bin/env python3
"""
Camera Diagnostics Tool

Real-time monitoring and visualization of camera data. 

Usage:
    ros2 run tmr_camera camera_diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32

import time
import sys
from collections import deque

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError: 
    OPENCV_AVAILABLE = False


class CameraDiagnosticsNode(Node):
    """
    Diagnostic node for monitoring camera data quality.
    """
    
    def __init__(self):
        super().__init__('camera_diagnostics')
        
        # Parameters
        self.declare_parameter('show_image', True)
        self.declare_parameter('window_name', 'Camera Diagnostics')
        
        self.show_image = self.get_parameter('show_image').value
        self.window_name = self.get_parameter('window_name').value
        
        # Data tracking
        self.frame_count = 0
        self.last_rate_time = time.time()
        self.measured_rate = 0.0
        self.brightness_history = deque(maxlen=100)
        self.frame_times = deque(maxlen=100)
        self.last_frame_time = None
        
        # Status
        self.connected = False
        self.brightness = 0.0
        self.exposure_us = 0
        self.resolution = "N/A"
        
        # Latest image
        self.latest_image = None
        
        # Create QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sensor_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, sensor_qos)
        self.conn_sub = self.create_subscription(
            Bool, '/camera/connected', self.connected_callback, 10)
        self.brightness_sub = self.create_subscription(
            Float32, '/camera/brightness', self.brightness_callback, 10)
        self.exposure_sub = self.create_subscription(
            Float32, '/camera/exposure_time', self.exposure_callback, 10)
        
        # Display timer
        self.display_timer = self.create_timer(0.1, self.display_callback)
        
        # Console update timer
        self.console_timer = self.create_timer(0.5, self.console_callback)
        
        self.get_logger().info('Camera Diagnostics started')
        self.get_logger().info('Monitoring /camera/image_raw.. .')
        
        if self.show_image and OPENCV_AVAILABLE:
            cv2.namedWindow(self. window_name, cv2.WINDOW_NORMAL)
    
    def image_callback(self, msg:  Image):
        """Process image message"""
        current_time = time.time()
        
        # Track frame rate
        if self.last_frame_time is not None:
            dt = current_time - self.last_frame_time
            self. frame_times.append(dt)
        self.last_frame_time = current_time
        
        self.frame_count += 1
        self.resolution = f"{msg.width}x{msg. height}"
        
        # Convert to OpenCV image if showing
        if self.show_image and OPENCV_AVAILABLE:
            try:
                # Convert ROS Image to numpy array
                if msg.encoding == 'bgr8':
                    self.latest_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg. width, 3)
                elif msg.encoding == 'rgb8':
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 3)
                    self.latest_image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif msg.encoding == 'mono8':
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width)
                    self.latest_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                else: 
                    self.latest_image = None
            except Exception as e:
                self.get_logger().error(f'Image conversion error: {e}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Process camera info message"""
        pass  # Camera info received
    
    def connected_callback(self, msg: Bool):
        """Process connection status"""
        self.connected = msg. data
    
    def brightness_callback(self, msg: Float32):
        """Process brightness message"""
        self.brightness = msg.data
        self.brightness_history.append(msg.data)
    
    def exposure_callback(self, msg:  Float32):
        """Process exposure time message"""
        self.exposure_us = int(msg.data)
    
    def display_callback(self):
        """Display image with overlays"""
        if not self.show_image or not OPENCV_AVAILABLE: 
            return
        
        if self.latest_image is None:
            return
        
        # Create display image with overlays
        display = self.latest_image.copy()
        
        # Calculate stats
        if self.frame_times:
            avg_dt = sum(self.frame_times) / len(self.frame_times)
            fps = 1.0 / avg_dt if avg_dt > 0 else 0
        else:
            fps = 0
        
        # Draw info overlay
        overlay_lines = [
            f"FPS: {fps:.1f}",
            f"Resolution: {self.resolution}",
            f"Brightness: {self.brightness:.1f}",
            f"Exposure: {self.exposure_us} us",
            f"Connected: {'Yes' if self.connected else 'No'}",
        ]
        
        y_offset = 30
        for line in overlay_lines:
            cv2.putText(display, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y_offset += 30
        
        # Draw histogram
        if len(display.shape) == 3:
            gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
        else:
            gray = display
        
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        hist = hist.flatten() / hist.max() * 100
        
        hist_img = np.zeros((100, 256, 3), dtype=np.uint8)
        for i in range(256):
            cv2.line(hist_img, (i, 100), (i, 100 - int(hist[i])), (255, 255, 255), 1)
        
        # Place histogram in corner
        h, w = display.shape[:2]
        hist_h, hist_w = hist_img. shape[:2]
        display[h-hist_h-10:h-10, w-hist_w-10:w-10] = hist_img
        
        # Show image
        cv2.imshow(self.window_name, display)
        cv2.waitKey(1)
    
    def console_callback(self):
        """Update console display"""
        # Calculate rate
        current_time = time. time()
        elapsed = current_time - self.last_rate_time
        
        if elapsed > 0:
            self.measured_rate = self.frame_count / elapsed
        
        self. frame_count = 0
        self.last_rate_time = current_time
        
        # Calculate brightness stats
        if self.brightness_history:
            avg_brightness = sum(self.brightness_history) / len(self.brightness_history)
            min_brightness = min(self.brightness_history)
            max_brightness = max(self.brightness_history)
        else:
            avg_brightness = min_brightness = max_brightness = 0
        
        # Clear screen and display
        print('\033[2J\033[H', end='')  # Clear screen
        
        print('=' * 60)
        print('           CAMERA DIAGNOSTICS')
        print('=' * 60)
        
        # Connection status
        status_str = '\033[92mCONNECTED\033[0m' if self. connected else '\033[91mDISCONNECTED\033[0m'
        print(f'\nStatus: {status_str}')
        print(f'Resolution: {self.resolution}')
        print(f'Frame Rate: {self.measured_rate:.1f} fps')
        
        print('\n' + '-' * 60)
        print('EXPOSURE')
        print('-' * 60)
        print(f'  Exposure Time: {self.exposure_us} µs')
        print(f'  Brightness: {self.brightness:.1f} (avg: {avg_brightness:.1f})')
        print(f'  Brightness Range: [{min_brightness:.1f}, {max_brightness:.1f}]')
        
        # Quality indicators
        print('\n' + '-' * 60)
        print('QUALITY INDICATORS')
        print('-' * 60)
        
        if self.measured_rate > 25:
            print('  Frame Rate:  \033[92m● Good\033[0m')
        elif self.measured_rate > 15:
            print('  Frame Rate: \033[93m● Low\033[0m')
        else:
            print('  Frame Rate: \033[91m● Very Low\033[0m')
        
        if 80 < avg_brightness < 180:
            print('  Exposure: \033[92m● Good\033[0m')
        elif 40 < avg_brightness < 220:
            print('  Exposure: \033[93m● Marginal\033[0m')
        else:
            print(f'  Exposure: \033[91m● Poor ({"too dark" if avg_brightness < 80 else "too bright"})\033[0m')
        
        print('\n' + '=' * 60)
        print('Press Ctrl+C to exit')
        if self.show_image and OPENCV_AVAILABLE:
            print('Press Q in image window to close it')
    
    def destroy_node(self):
        """Clean up"""
        if self.show_image and OPENCV_AVAILABLE:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = CameraDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()