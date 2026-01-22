#!/usr/bin/env python3
"""
EKF Fusion Integration Tests

Tests for verifying EKF sensor fusion is working correctly. 

Usage:
    pytest test_ekf_fusion.py -v
    ros2 run tmr_localization test_ekf_fusion.py
"""

import unittest
import time
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class TestEKFFusion(unittest.TestCase):
    """Test EKF sensor fusion."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_ekf_fusion')
        
        cls.odom_raw_received = False
        cls.odom_filtered_received = False
        cls.latest_raw = None
        cls.latest_filtered = None
        
        # Subscribers
        cls.raw_sub = cls.node.create_subscription(
            Odometry, '/vex/odom_raw', cls.raw_callback, 10)
        cls.filtered_sub = cls. node.create_subscription(
            Odometry, '/odometry/filtered', cls.filtered_callback, 10)
        
        # Wait for data
        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if cls.odom_raw_received and cls. odom_filtered_received: 
                break
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls. node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def raw_callback(cls, msg):
        cls.odom_raw_received = True
        cls.latest_raw = msg
    
    @classmethod
    def filtered_callback(cls, msg):
        cls.odom_filtered_received = True
        cls.latest_filtered = msg
    
    def test_raw_odometry_received(self):
        """Test raw odometry is being published."""
        self.assertTrue(self.odom_raw_received, 
            "Raw odometry not received - is vex_serial_node running?")
    
    def test_filtered_odometry_received(self):
        """Test filtered odometry is being published."""
        self.assertTrue(self.odom_filtered_received,
            "Filtered odometry not received - is EKF running?")
    
    def test_filtered_has_valid_frame_ids(self):
        """Test filtered odometry has correct frame IDs."""
        if self.latest_filtered is None:
            self.skipTest("No filtered odometry received")
        
        self.assertEqual(self.latest_filtered.header.frame_id, "odom",
            f"Wrong frame_id: {self.latest_filtered.header.frame_id}")
        self.assertEqual(self.latest_filtered.child_frame_id, "base_link",
            f"Wrong child_frame_id: {self.latest_filtered.child_frame_id}")
    
    def test_filtered_position_valid(self):
        """Test filtered position values are valid (not NaN)."""
        if self.latest_filtered is None:
            self.skipTest("No filtered odometry received")
        
        pos = self.latest_filtered.pose. pose.position
        self.assertFalse(math.isnan(pos.x), "Position X is NaN")
        self.assertFalse(math.isnan(pos.y), "Position Y is NaN")
        self.assertFalse(math.isnan(pos.z), "Position Z is NaN")
    
    def test_filtered_orientation_valid(self):
        """Test filtered orientation is a valid quaternion."""
        if self.latest_filtered is None:
            self.skipTest("No filtered odometry received")
        
        q = self.latest_filtered.pose.pose.orientation
        
        # Check not NaN
        self.assertFalse(math.isnan(q.x), "Quaternion X is NaN")
        self.assertFalse(math.isnan(q.y), "Quaternion Y is NaN")
        self.assertFalse(math.isnan(q.z), "Quaternion Z is NaN")
        self.assertFalse(math.isnan(q.w), "Quaternion W is NaN")
        
        # Check normalized
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        self.assertAlmostEqual(norm, 1.0, delta=0.01,
            msg=f"Quaternion not normalized: {norm}")
    
    def test_covariance_valid(self):
        """Test covariance values are valid."""
        if self. latest_filtered is None:
            self.skipTest("No filtered odometry received")
        
        pose_cov = self.latest_filtered.pose.covariance
        
        # Diagonal elements should be positive
        diag_indices = [0, 7, 14, 21, 28, 35]
        for i in diag_indices:
            self.assertGreater(pose_cov[i], 0,
                f"Covariance diagonal {i} not positive:  {pose_cov[i]}")
    
    def test_positions_reasonably_close(self):
        """Test raw and filtered positions are reasonably close."""
        if self.latest_raw is None or self.latest_filtered is None:
            self.skipTest("Missing odometry data")
        
        raw_pos = self.latest_raw.pose.pose.position
        filt_pos = self.latest_filtered.pose.pose.position
        
        distance = math.sqrt(
            (raw_pos.x - filt_pos.x)**2 +
            (raw_pos.y - filt_pos.y)**2
        )
        
        # Should be within 1 meter for reasonable operation
        self.assertLess(distance, 1.0,
            f"Raw and filtered positions differ by {distance:.3f}m")


def main():
    unittest.main()


if __name__ == '__main__':
    main()
