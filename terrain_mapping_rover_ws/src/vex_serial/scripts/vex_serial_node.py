#!/usr/bin/env python3
"""
VEX Serial Node - ROS 2 interface for VEX V5 Brain communication. 

This node:
- Reads wheel odometry from VEX V5 over serial
- Sends motor commands to VEX V5 over serial
- Publishes nav_msgs/Odometry on /vex/odom_raw
- Subscribes to geometry_msgs/Twist on /vex/cmd_vel
- Handles reconnection and error recovery

Author:  Rover Team
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from rover_interfaces.msg import VexStatus, WheelOdom, TankDriveCmd

import math
import threading
from typing import Optional

# Import our protocol modules
from vex_serial import VexSerialProtocol, OdometryCalculator
from vex_serial.serial_protocol import OdomMessage, StatusMessage


class VexSerialNode(Node):
    """ROS 2 node for VEX V5 serial communication."""
    
    def __init__(self):
        super().__init__('vex_serial_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self. serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self. odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self. publish_tf = self.get_parameter('publish_tf').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self. track_width = self.get_parameter('track_width').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.invert_left = self.get_parameter('invert_left_encoder').value
        self.invert_right = self.get_parameter('invert_right_encoder').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        
        # Initialize serial protocol
        self. serial_protocol = VexSerialProtocol(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=0.1,
            reconnect_delay=1.0,
            max_reconnect_attempts=0  # Infinite reconnection attempts
        )
        
        # Initialize odometry calculator
        self.odom_calculator = OdometryCalculator(
            wheel_radius=self.wheel_radius,
            track_width=self.track_width,
            ticks_per_revolution=self.ticks_per_rev,
            invert_left=self.invert_left,
            invert_right=self.invert_right
        )
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy. BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy. KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, '/vex/odom_raw', sensor_qos)
        self.wheel_odom_pub = self.create_publisher(
            WheelOdom, '/vex/wheel_odom', sensor_qos)
        self.status_pub = self.create_publisher(
            VexStatus, '/vex/status', reliable_qos)
        
        # TF broadcaster (optional, EKF usually handles odom->base_link)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/vex/cmd_vel', self._cmd_vel_callback, 10)
        self.tank_cmd_sub = self.create_subscription(
            TankDriveCmd, '/vex/tank_cmd', self._tank_cmd_callback, 10)
        
        # Command watchdog timer
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)
        
        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        # Statistics
        self.odom_count = 0
        self.cmd_count = 0
        
        # Set up serial callbacks
        self.serial_protocol. set_odom_callback(self._odom_callback)
        self.serial_protocol.set_status_callback(self._status_callback)
        self.serial_protocol.set_error_callback(self._error_callback)
        
        # Connect and start reading
        self.get_logger().info(f'Connecting to VEX V5 on {self.serial_port}.. .')
        if self.serial_protocol.connect():
            self.get_logger().info('Connected to VEX V5!')
            self.serial_protocol.start_reading()
        else:
            self.get_logger().warn('Failed to connect, will retry in background.. .')
            self.serial_protocol.start_reading()
    
    def _declare_parameters(self):
        """Declare all node parameters with defaults."""
        # Serial configuration
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # Frame IDs
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)  # EKF handles TF
        
        # Robot geometry (must match physical robot)
        self.declare_parameter('wheel_radius', 0.0508)  # 5.08 cm
        self.declare_parameter('track_width', 0.2921)   # 29.21 cm
        self.declare_parameter('ticks_per_revolution', 900)  # VEX V5 18: 1
        
        # Motor direction compensation
        # Right motors are physically mirrored, so invert right encoder
        self.declare_parameter('invert_left_encoder', False)
        self.declare_parameter('invert_right_encoder', True)
        
        # Command timeout (stop if no commands received)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        
        # Velocity limits
        self.declare_parameter('max_linear_vel', 0.5)   # m/s
        self. declare_parameter('max_angular_vel', 2.0)  # rad/s
        
        # Covariance values for odometry message
        self.declare_parameter('pose_covariance_diagonal', 
            [0.01, 0.01, 0.01, 0.01, 0.01, 0.03])
        self.declare_parameter('twist_covariance_diagonal',
            [0.01, 0.01, 0.01, 0.01, 0.01, 0.03])
    
    def _odom_callback(self, odom_msg: OdomMessage):
        """Handle odometry message from VEX V5."""
        # Update odometry calculator
        state = self.odom_calculator.update(
            left_ticks=odom_msg.left_ticks,
            right_ticks=odom_msg.right_ticks,
            timestamp_ms=odom_msg. timestamp_ms
        )
        
        # Get current ROS time
        now = self.get_clock().now()
        
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = now. to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Set pose
        odom.pose. pose. position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0.0
        odom. pose.pose.orientation = self._yaw_to_quaternion(state.theta)
        
        # Set pose covariance
        pose_cov = self.get_parameter('pose_covariance_diagonal').value
        odom.pose. covariance = self._diagonal_covariance(pose_cov)
        
        # Set twist (velocity)
        odom.twist.twist.linear.x = state.v_linear
        odom.twist.twist.linear.y = 0.0
        odom. twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular. y = 0.0
        odom. twist.twist.angular.z = state.v_angular
        
        # Set twist covariance
        twist_cov = self.get_parameter('twist_covariance_diagonal').value
        odom.twist.covariance = self._diagonal_covariance(twist_cov)
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Also publish raw wheel odometry for debugging
        wheel_odom = WheelOdom()
        wheel_odom.header = odom.header
        wheel_odom.left_ticks = odom_msg.left_ticks
        wheel_odom.right_ticks = odom_msg.right_ticks
        wheel_odom.left_ticks_per_sec = odom_msg.left_velocity
        wheel_odom.right_ticks_per_sec = odom_msg.right_velocity
        wheel_odom.left_velocity = state.v_left
        wheel_odom.right_velocity = state.v_right
        wheel_odom.x = state.x
        wheel_odom.y = state.y
        wheel_odom.theta = state.theta
        wheel_odom.vex_timestamp_ms = odom_msg.timestamp_ms
        self.wheel_odom_pub. publish(wheel_odom)
        
        # Optionally broadcast TF
        if self.tf_broadcaster:
            self._broadcast_tf(odom, now)
        
        self.odom_count += 1
    
    def _status_callback(self, status_msg: StatusMessage):
        """Handle status message from VEX V5."""
        status = VexStatus()
        status.header. stamp = self.get_clock().now().to_msg()
        status.connected = self.serial_protocol.connected
        status.message_count = self.serial_protocol.message_count
        status.error_count = self.serial_protocol.error_count
        status. battery_voltage = status_msg.battery_voltage
        status.battery_percentage = self._estimate_battery_percentage(
            status_msg.battery_voltage)
        status.motor_temperatures = list(status_msg.motor_temps)
        status.checksum_errors = self.serial_protocol.checksum_errors
        
        # Calculate update rate
        if self.odom_count > 0:
            # Rough estimate based on message count
            status.update_rate_hz = 50.0  # Target rate
        
        self.status_pub.publish(status)
    
    def _error_callback(self, error_msg: str):
        """Handle error from serial protocol."""
        self.get_logger().error(f'VEX Serial Error: {error_msg}')
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle Twist command and convert to wheel velocities."""
        self.last_cmd_time = self.get_clock().now()
        
        # Get velocity limits
        max_linear = self. get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        
        # Clamp velocities
        linear = max(-max_linear, min(max_linear, msg.linear.x))
        angular = max(-max_angular, min(max_angular, msg.angular.z))
        
        # Convert to wheel velocities
        left_vel, right_vel = self. odom_calculator.get_wheel_velocities_for_twist(
            linear, angular)
        
        # Send to VEX V5
        # Note: We apply motor direction compensation on the V5 side
        # The velocities here are the DESIRED velocities for each wheel
        if self.serial_protocol.send_velocity_command(left_vel, right_vel):
            self.cmd_count += 1
        else:
            self.get_logger().warn('Failed to send velocity command')
    
    def _tank_cmd_callback(self, msg: TankDriveCmd):
        """Handle direct tank drive command."""
        self.last_cmd_time = self.get_clock().now()
        
        if msg.emergency_stop:
            self.serial_protocol.send_emergency_stop()
            return
        
        if msg.use_velocity:
            # Direct velocity command
            self.serial_protocol.send_velocity_command(
                msg.left_velocity, msg.right_velocity)
        else:
            # Power mode - would need different V5 handling
            self.get_logger().warn('Power mode not implemented, using velocity')
            max_vel = self.get_parameter('max_linear_vel').value
            left_vel = msg.left_power * max_vel
            right_vel = msg.right_power * max_vel
            self.serial_protocol.send_velocity_command(left_vel, right_vel)
    
    def _watchdog_callback(self):
        """Stop motors if no commands received recently."""
        if not self.serial_protocol.connected:
            return
        
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            # Send stop command
            self.serial_protocol.send_emergency_stop()
    
    def _publish_status(self):
        """Periodically publish status even without V5 status messages."""
        status = VexStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.connected = self.serial_protocol.connected
        status.message_count = self. serial_protocol.message_count
        status.error_count = self.serial_protocol.error_count
        status.checksum_errors = self.serial_protocol.checksum_errors
        
        self.status_pub.publish(status)
    
    def _broadcast_tf(self, odom:  Odometry, stamp):
        """Broadcast odom -> base_link transform."""
        t = TransformStamped()
        t.header.stamp = stamp. to_msg()
        t.header.frame_id = self. odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = odom.pose.pose. position.x
        t.transform.translation.y = odom. pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position. z
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    @staticmethod
    def _yaw_to_quaternion(yaw:  float) -> Quaternion:
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    @staticmethod
    def _diagonal_covariance(diagonal: list) -> list:
        """Create 6x6 covariance matrix from diagonal values."""
        cov = [0.0] * 36
        for i, val in enumerate(diagonal):
            cov[i * 7] = val  # Diagonal elements:  0, 7, 14, 21, 28, 35
        return cov
    
    @staticmethod
    def _estimate_battery_percentage(voltage: float) -> float:
        """Estimate battery percentage from voltage."""
        # VEX V5 battery is typically 12. 8V full, 10V empty
        min_v = 10.0
        max_v = 12.8
        percentage = (voltage - min_v) / (max_v - min_v) * 100.0
        return max(0.0, min(100.0, percentage))
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down VEX serial node...')
        self.serial_protocol.send_emergency_stop()
        self.serial_protocol.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = VexSerialNode()
    
    # Use multi-threaded executor for callbacks during serial reading
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
