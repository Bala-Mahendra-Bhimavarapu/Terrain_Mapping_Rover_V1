#!/usr/bin/env python3
"""
Camera Mock Node - Simulates camera for testing without hardware. 
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np


class CameraMockNode(Node):
    """Mock camera node for testing."""
    
    def __init__(self):
        super().__init__('camera_mock_node')
        
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.frame_id = self.get_parameter('frame_id').value
        update_rate = self.get_parameter('update_rate').value
        
        self.bridge = CvBridge()
        
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        self.timer = self.create_timer(1.0 / update_rate, self._timer_callback)
        
        self.frame_count = 0
        
        self.get_logger().info('Camera Mock Node started (SIMULATION MODE)')
        self.get_logger().warn('=' * 50)
        self.get_logger().warn('  THIS IS A MOCK NODE - NO REAL HARDWARE')
        self.get_logger().warn('=' * 50)
    
    def _timer_callback(self):
        now = self.get_clock().now()
        
        # Generate test pattern image
        image = self._generate_test_image()
        
        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        img_msg.header.stamp = now.to_msg()
        img_msg.header.frame_id = self.frame_id
        self.image_pub. publish(img_msg)
        
        # Publish camera info
        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.width = self.width
        info_msg.height = self.height
        info_msg.k = [600.0, 0.0, 640.0, 0.0, 600.0, 360.0, 0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [600.0, 0.0, 640.0, 0.0, 0.0, 600.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info_msg)
        
        self.frame_count += 1
    
    def _generate_test_image(self) -> np.ndarray:
        """Generate a test pattern image."""
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Create gradient background
        for y in range(self.height):
            image[y, :, 0] = int(255 * y / self.height)  # Red gradient
        
        # Add moving bar
        bar_pos = (self.frame_count * 5) % self.width
        bar_width = 50
        image[: , bar_pos: bar_pos+bar_width, 1] = 255  # Green bar
        
        # Add frame counter text (simple rectangle)
        cv_y = 50
        cv_x = 50
        image[cv_y:cv_y+30, cv_x:cv_x+100, 2] = 255  # Blue rectangle
        
        return image


def main(args=None):
    rclpy.init(args=args)
    node = CameraMockNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node. destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
