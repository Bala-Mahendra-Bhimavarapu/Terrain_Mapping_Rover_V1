#!/usr/bin/env python3
"""
Teleop Test

Unit tests for teleoperation system.
"""

import unittest
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TestTeleop(unittest. TestCase):
    """Test teleop functionality."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_teleop')
        
        cls.cmd_vel_received = False
        cls.last_cmd_vel = None
        cls.last_odom = None
        
        # Publisher (simulating teleop)
        cls.cmd_vel_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        cls.odom_sub = cls.node.create_subscription(
            Odometry, '/odom', cls.odom_callback, 10)
        
        # Wait for connections
        time.sleep(1.0)
        for _ in range(20):
            rclpy.spin_once(cls.node, timeout_sec=0.1)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        # Send stop command
        twist = Twist()
        cls.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def odom_callback(cls, msg):
        cls.last_odom = msg
    
    def send_cmd_vel(self, linear:  float, angular: float, duration: float):
        """Send velocity command for duration."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
    
    def test_zero_velocity(self):
        """Test zero velocity command."""
        self.send_cmd_vel(0.0, 0.0, 0.5)
        
        if self.last_odom: 
            vx = self.last_odom.twist.twist.linear.x
            vz = self.last_odom.twist.twist.angular.z
            
            self.assertAlmostEqual(vx, 0.0, delta=0.05, msg="Linear velocity not zero")
            self.assertAlmostEqual(vz, 0.0, delta=0.1, msg="Angular velocity not zero")
    
    def test_forward_command(self):
        """Test forward velocity command."""
        target_vel = 0.1
        self.send_cmd_vel(target_vel, 0.0, 1.0)
        
        if self.last_odom:
            vx = self.last_odom.twist.twist.linear.x
            # Should be moving forward (positive velocity)
            self.assertGreater(vx, 0.0, "Robot not moving forward")
        
        # Stop
        self.send_cmd_vel(0.0, 0.0, 0.5)
    
    def test_rotation_command(self):
        """Test rotation velocity command."""
        target_vel = 0.2
        self.send_cmd_vel(0.0, target_vel, 1.0)
        
        if self.last_odom:
            vz = self.last_odom. twist.twist.angular.z
            # Should be rotating
            self.assertGreater(abs(vz), 0.05, "Robot not rotating")
        
        # Stop
        self. send_cmd_vel(0.0, 0.0, 0.5)
    
    def test_combined_command(self):
        """Test combined linear and angular velocity."""
        self.send_cmd_vel(0.1, 0.1, 1.0)
        
        if self.last_odom:
            vx = self.last_odom.twist.twist.linear.x
            vz = self.last_odom.twist. twist.angular.z
            
            # Should have both components
            self.assertGreater(abs(vx), 0.01, "No linear velocity")
        
        # Stop
        self. send_cmd_vel(0.0, 0.0, 0.5)


if __name__ == '__main__':
    unittest.main()