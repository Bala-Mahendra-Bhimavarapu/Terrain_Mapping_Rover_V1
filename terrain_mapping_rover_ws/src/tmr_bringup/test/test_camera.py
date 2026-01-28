#!/usr/bin/env python3
"""
Camera Test

Unit tests for RGB camera.
"""

import unittest
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool


class TestCamera(unittest.TestCase):
    """Test camera functionality."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy. init()
        cls.node = rclpy.create_node('test_camera')
        
        cls.connected = False
        cls.image_count = 0
        cls.last_image = None
        cls.camera_info = None
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        cls.connected_sub = cls.node.create_subscription(
            Bool, '/camera/connected', cls.connected_callback, 10)
        cls.image_sub = cls.node.create_subscription(
            Image, '/camera/image_raw', cls.image_callback, qos)
        cls.info_sub = cls.node.create_subscription(
            CameraInfo, '/camera/camera_info', cls.info_callback, qos)
        
        # Collect data
        timeout = 5.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def connected_callback(cls, msg):
        cls.connected = msg.data
    
    @classmethod
    def image_callback(cls, msg):
        cls.image_count += 1
        cls.last_image = msg
    
    @classmethod
    def info_callback(cls, msg):
        cls.camera_info = msg
    
    def test_connection(self):
        """Test camera is connected."""
        self.assertTrue(self.connected, "Camera not connected")
    
    def test_images_received(self):
        """Test images are being received."""
        self.assertGreater(self.image_count, 0, "No images received")
    
    def test_image_dimensions(self):
        """Test image has valid dimensions."""
        if self.last_image is None:
            self.skipTest("No image received")
        
        self.assertGreater(self.last_image.width, 0, "Invalid image width")
        self.assertGreater(self.last_image.height, 0, "Invalid image height")
    
    def test_image_encoding(self):
        """Test image encoding is valid."""
        if self.last_image is None:
            self.skipTest("No image received")
        
        valid_encodings = ['bgr8', 'rgb8', 'mono8', 'yuv422']
        self.assertIn(self.last_image.encoding, valid_encodings,
                     f"Invalid encoding: {self.last_image.encoding}")
    
    def test_image_data(self):
        """Test image has data."""
        if self.last_image is None:
            self.skipTest("No image received")
        
        expected_size = self.last_image.height * self.last_image.step
        self.assertEqual(len(self.last_image.data), expected_size,
                        "Image data size mismatch")
    
    def test_camera_info(self):
        """Test camera info is received."""
        self.assertIsNotNone(self.camera_info, "No camera info received")
        
        if self.camera_info:
            self.assertGreater(self.camera_info.width, 0)
            self.assertGreater(self.camera_info.height, 0)
    
    def test_frame_rate(self):
        """Test frame rate is acceptable."""
        # Should have at least 5 fps
        expected_frames = 5.0 * 5  # 5 seconds at 5 fps
        self.assertGreater(self.image_count, expected_frames * 0.5,
                          f"Low frame rate: {self.image_count} frames in 5s")


if __name__ == '__main__':
    unittest.main()
