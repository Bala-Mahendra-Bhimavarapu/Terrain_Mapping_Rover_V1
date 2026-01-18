#!/usr/bin/env python3
"""
VEX Mock Node - Simulates VEX V5 Brain for testing without hardware. 

This node simulates: 
- Wheel odometry based on received velocity commands
- Status messages
- Serial protocol behavior

Use this for testing the rest of the ROS 2 stack without the physical robot. 

Author:  Rover Team
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from rover_interfaces.msg import VexStatus, WheelOdom

import math
import time


class VexMockNode(Node):
    """Mock VEX V5 node for testing."""
    
    def __init__(self):
        super().__init__('vex_mock_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.0508)
        self.declare_parameter('track_width', 0.2921)
        self.declare_parameter('ticks_per_revolution', 900)
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self. track_width = self.get_parameter('track_width').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.update_rate = self.get_parameter('update_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.target_left_vel = 0.0
        self.target_right_vel = 0.0
        self.start_time = time.time()
        
        # Derived constants
        self.meters_per_tick = (2. 0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vex/odom_raw', 10)
        self.wheel_odom_pub = self. create_publisher(WheelOdom, '/vex/wheel_odom', 10)
        self.status_pub = self.create_publisher(VexStatus, '/vex/status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/vex/cmd_vel', self._cmd_vel_callback, 10)
        
        # Timers
        self.odom_timer = self.create_timer(
            1.0 / self.update_rate, self._update_odometry)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        self.last_update_time = self.get_clock().now()
        
        self.get_logger().info('VEX Mock Node started (SIMULATION MODE)')
        self.get_logger().warn('=' * 50)
        self.get_logger().warn('  THIS IS A MOCK NODE - NO REAL HARDWARE')
        self.get_logger().warn('=' * 50)
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands."""
        # Convert twist to wheel velocities
        linear = msg.linear.x
        angular = msg.angular.z
        
        self.target_left_vel = linear - (angular * self.track_width / 2.0)
        self.target_right_vel = linear + (angular * self.track_width / 2.0)
    
    def _update_odometry(self):
        """Update simulated odometry."""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        
        if dt <= 0 or dt > 1.0:
            return
        
        # Calculate wheel displacements
        left_dist = self.target_left_vel * dt
        right_dist = self.target_right_vel * dt
        
        # Update encoder ticks
        left_delta_ticks = int(left_dist / self.meters_per_tick)
        right_delta_ticks = int(right_dist / self.meters_per_tick)
        self.left_ticks += left_delta_ticks
        self. right_ticks += right_delta_ticks
        
        # Differential drive kinematics
        delta_dist = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.track_width
        
        # Update pose
        mid_theta = self.theta + delta_theta / 2.0
        self.x += delta_dist * math.cos(mid_theta)
        self.y += delta_dist * math.sin(mid_theta)
        self.theta += delta_theta
        
        # Normalize theta
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi
        
        # Calculate velocities
        v_linear = delta_dist / dt if dt > 0 else 0.0
        v_angular = delta_theta / dt if dt > 0 else 0.0
        
        # Create and publish Odometry message
        odom = Odometry()
        odom.header. stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position. x = self.x
        odom.pose.pose.position. y = self.y
        odom.pose.pose.position. z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(self.theta)
        
        # Set covariance
        odom.pose.covariance = self._default_covariance()
        
        odom.twist.twist.linear. x = v_linear
        odom.twist.twist.angular. z = v_angular
        odom.twist.covariance = self._default_covariance()
        
        self.odom_pub.publish(odom)
        
        # Publish wheel odometry
        wheel_odom = WheelOdom()
        wheel_odom.header = odom.header
        wheel_odom.left_ticks = self.left_ticks
        wheel_odom.right_ticks = self.right_ticks
        wheel_odom.left_velocity = self.target_left_vel
        wheel_odom.right_velocity = self.target_right_vel
        wheel_odom.x = self.x
        wheel_odom.y = self.y
        wheel_odom.theta = self.theta
        wheel_odom.vex_timestamp_ms = int((time.time() - self.start_time) * 1000)
        self.wheel_odom_pub.publish(wheel_odom)
    
    def _publish_status(self):
        """Publish mock status."""
        status = VexStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.connected = True
        status.message_count = int((time.time() - self.start_time) * self.update_rate)
        status.error_count = 0
        status.battery_voltage = 12.5  # Simulated full battery
        status.battery_percentage = 90.0
        status.motor_temperatures = [35.0, 35.0, 35.0, 35.0]
        status.update_rate_hz = self.update_rate
        status.checksum_errors = 0
        
        self.status_pub.publish(status)
    
    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw to quaternion."""
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    @staticmethod
    def _default_covariance() -> list:
        """Return default covariance matrix."""
        cov = [0.0] * 36
        diag = [0.01, 0.01, 0.01, 0.01, 0.01, 0.03]
        for i, val in enumerate(diag):
            cov[i * 7] = val
        return cov


def main(args=None):
    rclpy.init(args=args)
    node = VexMockNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()