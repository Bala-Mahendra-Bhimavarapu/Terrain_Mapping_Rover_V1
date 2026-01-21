#!/usr/bin/env python3
"""
Velocity Smoother Node

Smooths velocity commands by applying acceleration limits. 
Prevents jerky motion and protects motors from sudden changes.

Usage:
    ros2 run tmr_teleop velocity_smoother
"""

import rclpy
from rclpy. node import Node
from geometry_msgs.msg import Twist

import time
import math


class VelocitySmootherNode(Node):
    """
    Velocity smoother ROS 2 node. 
    
    Applies acceleration limits to smooth velocity transitions.
    """
    
    def __init__(self):
        super().__init__('velocity_smoother')
        
        # Declare parameters
        self. declare_parameter('max_linear_acceleration', 0.5)
        self.declare_parameter('max_linear_deceleration', 1.0)
        self.declare_parameter('max_angular_acceleration', 1.0)
        self.declare_parameter('max_angular_deceleration', 2.0)
        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('timeout', 0.5)
        
        # Get parameters
        self. max_linear_accel = self.get_parameter('max_linear_acceleration').value
        self.max_linear_decel = self.get_parameter('max_linear_deceleration').value
        self.max_angular_accel = self.get_parameter('max_angular_acceleration').value
        self.max_angular_decel = self.get_parameter('max_angular_deceleration').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.rate = self.get_parameter('rate').value
        self. timeout = self.get_parameter('timeout').value
        
        # Current state
        self.current_linear = 0.0
        self. current_angular = 0.0
        
        # Target velocity
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Timing
        self.last_cmd_time = 0.0
        self.last_update_time = time.time()
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, output_topic, 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, input_topic, self.cmd_vel_callback, 10)
        
        # Timer for smoothing loop
        self.timer = self.create_timer(1.0 / self.rate, self.update_callback)
        
        self.get_logger().info('Velocity Smoother initialized')
        self.get_logger().info(f'  Input:  {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Max Linear Accel: {self.max_linear_accel} m/s²')
        self.get_logger().info(f'  Max Angular Accel: {self.max_angular_accel} rad/s²')
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive target velocity command"""
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        self.last_cmd_time = time.time()
    
    def _smooth_velocity(
        self,
        current:  float,
        target: float,
        max_accel: float,
        max_decel: float,
        dt: float
    ) -> float:
        """
        Smooth velocity towards target with acceleration limits.
        
        Args:
            current: Current velocity
            target: Target velocity
            max_accel: Maximum acceleration
            max_decel: Maximum deceleration
            dt: Time step
        
        Returns:
            New smoothed velocity
        """
        error = target - current
        
        if abs(error) < 0.001:
            return target
        
        # Determine if accelerating or decelerating
        if abs(target) > abs(current):
            # Accelerating (speeding up)
            max_change = max_accel * dt
        else:
            # Decelerating (slowing down)
            max_change = max_decel * dt
        
        # Also consider direction change as deceleration
        if (current > 0 and target < 0) or (current < 0 and target > 0):
            max_change = max_decel * dt
        
        # Apply change limit
        if error > 0:
            return min(current + max_change, target)
        else:
            return max(current - max_change, target)
    
    def update_callback(self):
        """Timer callback for smoothing update"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Check for command timeout
        if current_time - self.last_cmd_time > self.timeout:
            # No recent command, stop
            self.target_linear = 0.0
            self.target_angular = 0.0
        
        # Smooth velocities
        self.current_linear = self._smooth_velocity(
            self.current_linear,
            self.target_linear,
            self.max_linear_accel,
            self.max_linear_decel,
            dt
        )
        
        self.current_angular = self._smooth_velocity(
            self. current_angular,
            self. target_angular,
            self. max_angular_accel,
            self.max_angular_decel,
            dt
        )
        
        # Publish smoothed velocity
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self. current_angular
        self.cmd_vel_pub.publish(twist)


class VelocitySmoother: 
    """
    Standalone velocity smoother class (can be used without ROS).
    """
    
    def __init__(
        self,
        max_linear_accel: float = 0.5,
        max_linear_decel: float = 1.0,
        max_angular_accel: float = 1.0,
        max_angular_decel: float = 2.0
    ):
        self.max_linear_accel = max_linear_accel
        self.max_linear_decel = max_linear_decel
        self. max_angular_accel = max_angular_accel
        self.max_angular_decel = max_angular_decel
        
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_time = time.time()
    
    def smooth(self, target_linear: float, target_angular: float) -> tuple: 
        """
        Smooth velocity towards targets. 
        
        Args:
            target_linear: Target linear velocity
            target_angular: Target angular velocity
        
        Returns:
            Tuple of (smoothed_linear, smoothed_angular)
        """
        current_time = time.time()
        dt = current_time - self. last_time
        self.last_time = current_time
        
        # Smooth linear
        linear_error = target_linear - self.current_linear
        if abs(linear_error) > 0.001:
            if abs(target_linear) > abs(self.current_linear):
                max_change = self.max_linear_accel * dt
            else:
                max_change = self.max_linear_decel * dt
            
            if linear_error > 0:
                self.current_linear = min(self.current_linear + max_change, target_linear)
            else:
                self.current_linear = max(self.current_linear - max_change, target_linear)
        else:
            self.current_linear = target_linear
        
        # Smooth angular
        angular_error = target_angular - self. current_angular
        if abs(angular_error) > 0.001:
            if abs(target_angular) > abs(self.current_angular):
                max_change = self.max_angular_accel * dt
            else: 
                max_change = self. max_angular_decel * dt
            
            if angular_error > 0:
                self. current_angular = min(self. current_angular + max_change, target_angular)
            else:
                self.current_angular = max(self.current_angular - max_change, target_angular)
        else:
            self. current_angular = target_angular
        
        return (self.current_linear, self. current_angular)
    
    def reset(self):
        """Reset smoother state"""
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_time = time.time()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = VelocitySmootherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
