#!/usr/bin/env python3
"""
Gamepad Teleoperation Node

Allows controlling the rover using a gamepad/joystick. 

Default Controls (Xbox-style):
    Left Stick Y  : Forward/Backward
    Left Stick X  : Turn Left/Right
    RB (R1)       : Enable movement (deadman switch)
    LB (L1)       : Turbo mode
    B (Circle)    : Emergency Stop

Usage:
    ros2 run tmr_teleop teleop_gamepad
    ros2 launch tmr_teleop teleop_gamepad.launch.py
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

import time


class TeleopGamepadNode(Node):
    """
    Gamepad teleoperation ROS 2 node.
    """
    
    def __init__(self):
        super().__init__('teleop_gamepad')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # State
        self.last_joy_time = 0.0
        self.estop_active = False
        self.enabled = False
        self.turbo = False
        
        # Current velocity
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Publisher
        self.cmd_vel_pub = self. create_publisher(
            Twist, self.cmd_vel_topic, 10)
        
        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self. joy_callback, 10)
        
        # Emergency stop service client
        self.estop_client = self.create_client(Trigger, '/vex/emergency_stop')
        
        # Timer for publishing
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self. publish_callback)
        
        self.get_logger().info('Gamepad Teleop Node initialized')
        self.get_logger().info(f'  Max Linear:  {self.max_linear} m/s')
        self.get_logger().info(f'  Max Angular: {self.max_angular} rad/s')
        self.get_logger().info(f'  Enable Button: {self.enable_button}')
        self.get_logger().info(f'  Turbo Button: {self.turbo_button}')
        self.get_logger().info('Waiting for joy messages...')
    
    def _declare_parameters(self):
        """Declare all node parameters"""
        # Velocity limits
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('turbo_multiplier', 1.5)
        
        # Axis configuration
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', True)
        self.declare_parameter('deadzone', 0.1)
        
        # Button configuration
        self.declare_parameter('enable_button', 5)
        self.declare_parameter('turbo_button', 4)
        self.declare_parameter('estop_button', 1)
        
        # Topics
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_rate', 20.0)
    
    def _get_parameters(self):
        """Get all parameter values"""
        # Velocity limits
        self.max_linear = self.get_parameter('max_linear_velocity').value
        self.max_angular = self.get_parameter('max_angular_velocity').value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').value
        
        # Axis configuration
        self. linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.invert_linear = self.get_parameter('invert_linear').value
        self.invert_angular = self.get_parameter('invert_angular').value
        self. deadzone = self.get_parameter('deadzone').value
        
        # Button configuration
        self. enable_button = self.get_parameter('enable_button').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self. estop_button = self.get_parameter('estop_button').value
        
        # Topics
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
    
    def _apply_deadzone(self, value:  float) -> float:
        """Apply deadzone to axis value"""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg:  Joy):
        """Process joystick input"""
        self.last_joy_time = time.time()
        
        # Check if we have enough axes and buttons
        if len(msg. axes) <= max(self.linear_axis, self.angular_axis):
            self.get_logger().warn('Not enough axes in joy message')
            return
        
        if len(msg.buttons) <= max(self.enable_button, self.turbo_button, self.estop_button):
            self.get_logger().warn('Not enough buttons in joy message')
            return
        
        # Check emergency stop button
        if msg.buttons[self.estop_button]: 
            self._emergency_stop()
            return
        
        # Check enable button (deadman switch)
        self.enabled = bool(msg.buttons[self.enable_button])
        
        # Check turbo button
        self.turbo = bool(msg. buttons[self.turbo_button])
        
        if not self.enabled:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            return
        
        # Clear e-stop if enabled is pressed
        if self.estop_active:
            self. estop_active = False
            self.get_logger().info('E-Stop cleared')
        
        # Get axis values
        linear_raw = msg.axes[self.linear_axis]
        angular_raw = msg.axes[self.angular_axis]
        
        # Apply inversion
        if self.invert_linear:
            linear_raw = -linear_raw
        if self.invert_angular:
            angular_raw = -angular_raw
        
        # Apply deadzone
        linear = self._apply_deadzone(linear_raw)
        angular = self._apply_deadzone(angular_raw)
        
        # Calculate velocities
        linear_scale = self.max_linear
        angular_scale = self.max_angular
        
        # Apply turbo
        if self.turbo:
            linear_scale *= self.turbo_multiplier
            angular_scale *= self.turbo_multiplier
        
        self.linear_vel = linear * linear_scale
        self.angular_vel = angular * angular_scale
    
    def _emergency_stop(self):
        """Trigger emergency stop"""
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.estop_active = True
        self.enabled = False
        
        # Publish stop command immediately
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Call emergency stop service if available
        if self.estop_client. wait_for_service(timeout_sec=0.1):
            request = Trigger.Request()
            self.estop_client.call_async(request)
        
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')
    
    def publish_callback(self):
        """Timer callback to publish velocity commands"""
        # Check for joy timeout
        if time.time() - self.last_joy_time > 0.5:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.enabled = False
        
        # Create and publish Twist message
        twist = Twist()
        
        if self.enabled and not self.estop_active:
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel
        
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = TeleopGamepadNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        # Send stop command
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()