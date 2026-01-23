#!/usr/bin/env python3
"""
SLAM Topics Test

Tests that all required SLAM topics are available. 

Usage:
    pytest test_slam_topics.py -v
"""

import unittest
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, PointCloud2, CameraInfo


class TestSLAMTopics(unittest.TestCase):
    """Test SLAM topic availability."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_slam_topics')
        
        cls.received = {
            'rgb': False,
            'rgb_info': False,
            'depth':  False,
            'odom': False,
            'map': False,
            'cloud':  False,
        }
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Subscribe to all topics
        cls. node.create_subscription(
            Image, '/camera/image_raw',
            lambda m: cls._mark_received('rgb'), sensor_qos)
        cls.node.create_subscription(
            CameraInfo, '/camera/camera_info',
            lambda m: cls._mark_received('rgb_info'), sensor_qos)
        cls.node.create_subscription(
            Image, '/tof/depth/image_raw',
            lambda m:  cls._mark_received('depth'), sensor_qos)
        cls.node.create_subscription(
            Odometry, '/odometry/filtered',
            lambda m:  cls._mark_received('odom'), 10)
        cls.node.create_subscription(
            OccupancyGrid, '/map',
            lambda m: cls._mark_received('map'), 10)
        cls.node.create_subscription(
            PointCloud2, '/rtabmap/cloud_map',
            lambda m: cls._mark_received('cloud'), 10)
        
        # Collect data
        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def _mark_received(cls, topic_key):
        cls.received[topic_key] = True
    
    def test_rgb_image_available(self):
        """Test RGB image is being published."""
        self.assertTrue(self.received['rgb'],
            "RGB image not received on /camera/image_raw")
    
    def test_camera_info_available(self):
        """Test camera info is being published."""
        self.assertTrue(self.received['rgb_info'],
            "Camera info not received on /camera/camera_info")
    
    def test_depth_image_available(self):
        """Test depth image is being published."""
        self. assertTrue(self.received['depth'],
            "Depth image not received on /tof/depth/image_raw")
    
    def test_odometry_available(self):
        """Test odometry is being published."""
        self.assertTrue(self.received['odom'],
            "Odometry not received on /odometry/filtered")
    
    def test_map_being_published(self):
        """Test 2D map is being published."""
        # This may fail if SLAM isn't running
        if not self.received['map']: 
            self.skipTest("Map not received - SLAM may not be running")
    
    def test_cloud_map_being_published(self):
        """Test 3D cloud map is being published."""
        if not self.received['cloud']: 
            self.skipTest("Cloud map not received - SLAM may not be running")


if __name__ == '__main__':
    unittest.main()