#!/usr/bin/env python3
"""
Phase 3 EKF Unit Tests

Unit tests for EKF sensor fusion.

Usage:
    pytest test_phase3_ekf.py -v
"""

import unittest
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class TestPhase3EKF(unittest.TestCase):
    """Unit tests for Phase 3 EKF."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_phase3_ekf')
        
        cls.odom_raw_received = False
        cls.odom_filtered_received = False
        cls.imu_received = False
        
        cls.latest_raw = None
        cls.latest_filtered = None
        cls.latest_imu = None
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        cls.raw_sub = cls.node.create_subscription(
            Odometry, '/vex/odom_raw', cls.raw_callback, 10)
        cls.filtered_sub = cls.node.create_subscription(
            Odometry, '/odometry/filtered', cls.filtered_callback, 10)
        cls.imu_sub = cls. node.create_subscription(
            Imu, '/imu/data', cls.imu_callback, sensor_qos)
        
        # Collect data
        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if cls.odom_raw_received and cls.odom_filtered_received: 
                break
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def raw_callback(cls, msg):
        cls.odom_raw_received = True
        cls.latest_raw = msg
    
    @classmethod
    def filtered_callback(cls, msg):
        cls.odom_filtered_received = True
        cls.latest_filtered = msg
    
    @classmethod
    def imu_callback(cls, msg):
        cls.imu_received = True
        cls. latest_imu = msg
    
    def test_raw_odometry_received(self):
        """Test raw odometry is being published."""
        self.assertTrue(self.odom_raw_received,
            "Raw odometry not received - check vex_serial_node")
    
    def test_filtered_odometry_received(self):
        """Test filtered odometry is being published."""
        self. assertTrue(self.odom_filtered_received,
            "Filtered odometry not received - check EKF node")
    
    def test_imu_received(self):
        """Test IMU data is being published."""
        self. assertTrue(self.imu_received,
            "IMU data not received - check imu_node")
    
    def test_filtered_frame_id(self):
        """Test filtered odometry has correct frame_id."""
        if self.latest_filtered is None:
            self.skipTest("No filtered odometry")
        
        self.assertEqual(self.latest_filtered.header.frame_id, "odom",
            f"Wrong frame_id: {self.latest_filtered.header.frame_id}")
    
    def test_filtered_child_frame_id(self):
        """Test filtered odometry has correct child_frame_id."""
        if self.latest_filtered is None:
            self.skipTest("No filtered odometry")
        
        self.assertEqual(self.latest_filtered.child_frame_id, "base_link",
            f"Wrong child_frame_id: {self.latest_filtered.child_frame_id}")
    
    def test_position_not_nan(self):
        """Test position values are not NaN."""
        if self.latest_filtered is None:
            self.skipTest("No filtered odometry")
        
        pos = self.latest_filtered.pose.pose.position
        self.assertFalse(math.isnan(pos.x), "Position X is NaN")
        self.assertFalse(math.isnan(pos.y), "Position Y is NaN")
        self.assertFalse(math.isnan(pos.z), "Position Z is NaN")
    
    def test_orientation_valid(self):
        """Test orientation is valid quaternion."""
        if self.latest_filtered is None:
            self.skipTest("No filtered odometry")
        
        q = self.latest_filtered.pose.pose.orientation
        
        # Not NaN
        self.assertFalse(math.isnan(q.x), "Quaternion X is NaN")
        self.assertFalse(math.isnan(q.y), "Quaternion Y is NaN")
        self.assertFalse(math.isnan(q.z), "Quaternion Z is NaN")
        self.assertFalse(math.isnan(q.w), "Quaternion W is NaN")
        
        # Normalized
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q. w**2)
        self.assertAlmostEqual(norm, 1.0, delta=0.01,
            msg=f"Quaternion not normalized: {norm}")
    
    def test_covariance_positive(self):
        """Test covariance diagonal elements are positive."""
        if self. latest_filtered is None:
            self.skipTest("No filtered odometry")
        
        cov = self.latest_filtered.pose.covariance
        diag_indices = [0, 7, 14, 21, 28, 35]
        
        for i in diag_indices: 
            self.assertGreater(cov[i], 0,
                f"Covariance[{i}] not positive: {cov[i]}")
    
    def test_velocity_reasonable(self):
        """Test velocity values are reasonable."""
        if self. latest_filtered is None:
            self.skipTest("No filtered odometry")
        
        vx = self.latest_filtered. twist.twist.linear.x
        wz = self.latest_filtered. twist.twist.angular.z
        
        # When stationary or slow, should be within bounds
        self.assertLess(abs(vx), 2.0, f"Linear velocity too high: {vx}")
        self.assertLess(abs(wz), 5.0, f"Angular velocity too high: {wz}")


if __name__ == '__main__':
    unittest.main()