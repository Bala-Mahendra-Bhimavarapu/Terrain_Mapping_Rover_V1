#!/usr/bin/env python3
"""
VEX Connection Test

Unit tests for VEX serial connection. 
"""

import unittest
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TestVexConnection(unittest. TestCase):
    """Test VEX connection and basic communication."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_vex_connection')
        
        cls.connected = False
        cls.odom_received = False
        cls.last_odom = None
        
        # Subscribers
        cls.connected_sub = cls.node.create_subscription(
            Bool, '/vex/connected', cls.connected_callback, 10)
        cls.odom_sub = cls. node.create_subscription(
            Odometry, '/odom', cls.odom_callback, 10)
        
        # Publisher
        cls.cmd_vel_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Wait for connection
        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if cls. connected:
                break
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls. node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def connected_callback(cls, msg):
        cls.connected = msg.data
    
    @classmethod
    def odom_callback(cls, msg):
        cls.odom_received = True
        cls.last_odom = msg
    
    def test_connection(self):
        """Test VEX is connected."""
        self.assertTrue(self.connected, "VEX not connected")
    
    def test_odometry_received(self):
        """Test odometry messages are received."""
        # Spin to receive messages
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        self.assertTrue(self.odom_received, "No odometry received")
    
    def test_cmd_vel_accepted(self):
        """Test velocity commands are accepted."""
        if not self.connected:
            self.skipTest("VEX not connected")
        
        # Send command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Should still be connected after commands
        self.assertTrue(self. connected, "Lost connection after cmd_vel")
    
    def test_odometry_valid(self):
        """Test odometry data is valid."""
        if self.last_odom is None:
            self.skipTest("No odometry data")
        
        # Check position is reasonable (not NaN or Inf)
        x = self.last_odom.pose.pose.position.x
        y = self.last_odom.pose.pose.position.y
        
        import math
        self.assertFalse(math.isnan(x), "Odometry X is NaN")
        self.assertFalse(math.isnan(y), "Odometry Y is NaN")
        self.assertFalse(math. isinf(x), "Odometry X is Inf")
        self.assertFalse(math.isinf(y), "Odometry Y is Inf")


if __name__ == '__main__':
    unittest.main()
