#!/usr/bin/env python3
"""
ToF Camera Test

Unit tests for ToF depth camera.
"""

import unittest
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool


class TestToFCamera(unittest.TestCase):
    """Test ToF camera functionality."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_tof')
        
        cls.connected = False
        cls.depth_count = 0
        cls.points_count = 0
        cls.last_depth = None
        cls.last_points = None
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        cls.connected_sub = cls.node.create_subscription(
            Bool, '/tof/connected', cls.connected_callback, 10)
        cls.depth_sub = cls.node. create_subscription(
            Image, '/tof/depth/image_raw', cls.depth_callback, qos)
        cls.points_sub = cls.node.create_subscription(
            PointCloud2, '/tof/points', cls.points_callback, qos)
        
        # Collect data
        timeout = 5.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls. node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def connected_callback(cls, msg):
        cls.connected = msg. data
    
    @classmethod
    def depth_callback(cls, msg):
        cls.depth_count += 1
        cls.last_depth = msg
    
    @classmethod
    def points_callback(cls, msg):
        cls.points_count += 1
        cls.last_points = msg
    
    def test_connection(self):
        """Test ToF camera is connected."""
        self.assertTrue(self. connected, "ToF camera not connected")
    
    def test_depth_images_received(self):
        """Test depth images are being received."""
        self.assertGreater(self.depth_count, 0, "No depth images received")
    
    def test_depth_dimensions(self):
        """Test depth image has expected dimensions."""
        if self.last_depth is None:
            self.skipTest("No depth image received")
        
        # Arducam B0410 is 240x180
        self.assertEqual(self.last_depth.width, 240, "Unexpected width")
        self.assertEqual(self.last_depth.height, 180, "Unexpected height")
    
    def test_depth_encoding(self):
        """Test depth encoding is correct."""
        if self.last_depth is None:
            self. skipTest("No depth image received")
        
        # Should be 16-bit unsigned
        self.assertEqual(self. last_depth.encoding, '16UC1',
                        f"Unexpected encoding: {self.last_depth.encoding}")
    
    def test_point_cloud_received(self):
        """Test point cloud is received."""
        self. assertGreater(self.points_count, 0, "No point cloud received")
    
    def test_point_cloud_has_points(self):
        """Test point cloud contains points."""
        if self.last_points is None:
            self. skipTest("No point cloud received")
        
        num_points = self.last_points.width * self.last_points.height
        self.assertGreater(num_points, 0, "Point cloud is empty")
    
    def test_frame_rate(self):
        """Test frame rate is acceptable."""
        # Should have at least 5 fps
        expected_frames = 5.0 * 5  # 5 seconds at 5 fps
        self.assertGreater(self.depth_count, expected_frames * 0.5,
                          f"Low depth rate: {self.depth_count} frames in 5s")


if __name__ == '__main__':
    unittest.main()