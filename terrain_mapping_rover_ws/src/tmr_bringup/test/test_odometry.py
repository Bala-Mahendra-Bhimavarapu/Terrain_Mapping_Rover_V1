#!/usr/bin/env python3
"""
Odometry Test

Unit tests for odometry system.
"""

import unittest
import time
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TestOdometry(unittest.TestCase):
    """Test odometry functionality."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_odometry')
        
        cls.odom_data = []
        
        cls.cmd_vel_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.odom_sub = cls. node.create_subscription(
            Odometry, '/odom', cls.odom_callback, 10)
        
        # Wait for connection
        time.sleep(1.0)
        for _ in range(30):
            rclpy.spin_once(cls.node, timeout_sec=0.1)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        # Stop robot
        twist = Twist()
        cls.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def odom_callback(cls, msg):
        cls.odom_data.append(msg)
    
    def get_position(self):
        """Get current position from odometry."""
        if not self.odom_data:
            return (0.0, 0.0, 0.0)
        
        odom = self.odom_data[-1]
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position. y
        
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        return (x, y, yaw)
    
    def send_cmd_vel(self, linear: float, angular: float, duration: float):
        """Send velocity command."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start = time.time()
        while time.time() - start < duration: 
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
    
    def test_odometry_received(self):
        """Test odometry messages are received."""
        self.assertGreater(len(self.odom_data), 0, "No odometry received")
    
    def test_odometry_rate(self):
        """Test odometry publish rate."""
        initial_count = len(self.odom_data)
        
        start = time.time()
        while time.time() - start < 2.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        new_messages = len(self.odom_data) - initial_count
        rate = new_messages / 2.0
        
        self.assertGreater(rate, 10.0, f"Odometry rate too low: {rate:. 1f} Hz")
    
    def test_odometry_valid(self):
        """Test odometry values are valid."""
        if not self.odom_data:
            self.skipTest("No odometry data")
        
        odom = self.odom_data[-1]
        
        # Check for NaN
        self.assertFalse(math.isnan(odom.pose.pose.position.x), "X is NaN")
        self.assertFalse(math.isnan(odom.pose.pose.position.y), "Y is NaN")
        
        # Check quaternion is normalized
        q = odom.pose.pose.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        self.assertAlmostEqual(norm, 1.0, delta=0.01, msg="Quaternion not normalized")
    
    def test_odometry_updates_on_motion(self):
        """Test odometry updates when robot moves."""
        # Record start position
        start_x, start_y, _ = self.get_position()
        
        # Move robot
        self.send_cmd_vel(0.1, 0.0, 2.0)
        self.send_cmd_vel(0.0, 0.0, 0.5)  # Stop
        
        # Record end position
        end_x, end_y, _ = self.get_position()
        
        # Should have moved
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        self.assertGreater(distance, 0.05, "Odometry did not update with motion")
    
    def test_frame_ids(self):
        """Test odometry frame IDs are correct."""
        if not self.odom_data:
            self.skipTest("No odometry data")
        
        odom = self.odom_data[-1]
        
        self.assertEqual(odom.header.frame_id, "odom",
                        f"Wrong header frame:  {odom.header.frame_id}")
        self.assertEqual(odom.child_frame_id, "base_link",
                        f"Wrong child frame: {odom.child_frame_id}")


if __name__ == '__main__':
    unittest.main()
