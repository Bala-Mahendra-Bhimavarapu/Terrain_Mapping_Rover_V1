#!/usr/bin/env python3
"""
Mock Beacon Scanner Node - Simulates beacon detection for testing. 

Publishes fake beacon readings that simulate a beacon at a fixed
location, with realistic RSSI variation based on distance. 

Usage:
    ros2 run bluetooth_nav beacon_mock_node. py

Parameters:
    beacon_x, beacon_y:  Simulated beacon position in map frame
    
Author:  Rover Team
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rover_interfaces.msg import BeaconSignal, BeaconArray

import math
import random


class BeaconMockNode(Node):
    """Mock beacon scanner for testing."""
    
    def __init__(self):
        super().__init__('beacon_mock_node')
        
        # Parameters for simulated beacon
        self.declare_parameter('beacon_x', 3.0)
        self.declare_parameter('beacon_y', 2.0)
        self.declare_parameter('beacon_uuid', 'mock-beacon-uuid-1234')
        self.declare_parameter('tx_power', -59)
        self.declare_parameter('noise_std', 3.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('detection_range', 10.0)
        self.declare_parameter('dropout_probability', 0.05)
        
        self.beacon_x = self.get_parameter('beacon_x').value
        self.beacon_y = self.get_parameter('beacon_y').value
        self.beacon_uuid = self.get_parameter('beacon_uuid').value
        self.tx_power = self. get_parameter('tx_power').value
        self.noise_std = self.get_parameter('noise_std').value
        self. detection_range = self.get_parameter('detection_range').value
        self.dropout_prob = self.get_parameter('dropout_probability').value
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        
        # Publishers
        self.beacon_pub = self.create_publisher(BeaconArray, '/bluetooth/beacons', 10)
        
        # Timer
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self._publish_beacon)
        
        self.get_logger().info('Mock Beacon Scanner started')
        self.get_logger().info(f'Simulated beacon at ({self.beacon_x}, {self.beacon_y})')
    
    def _odom_callback(self, msg):
        """Update robot position from odometry."""
        self.robot_x = msg. pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
    
    def _publish_beacon(self):
        """Publish simulated beacon reading."""
        # Calculate true distance to beacon
        dx = self.beacon_x - self.robot_x
        dy = self.beacon_y - self. robot_y
        true_distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if beacon is in range
        if true_distance > self.detection_range:
            # Out of range - publish empty array
            msg = BeaconArray()
            msg.header.stamp = self. get_clock().now().to_msg()
            msg.scan_duration_ms = 1000
            self.beacon_pub.publish(msg)
            return
        
        # Simulate random dropout
        if random.random() < self.dropout_prob:
            msg = BeaconArray()
            msg.header.stamp = self. get_clock().now().to_msg()
            msg.scan_duration_ms = 1000
            self.beacon_pub.publish(msg)
            return
        
        # Calculate RSSI from distance using log-distance path loss model
        # RSSI = tx_power - 10 * n * log10(distance)
        # where n is path loss exponent (2.5 for indoor)
        path_loss_exponent = 2.5
        if true_distance < 0.1:
            true_distance = 0.1  # Minimum distance
        
        rssi = self.tx_power - 10 * path_loss_exponent * math.log10(true_distance)
        
        # Add noise
        rssi += random.gauss(0, self.noise_std)
        rssi = int(rssi)
        
        # Calculate estimated distance (with some error)
        estimated_distance = true_distance * random.uniform(0.8, 1.2)
        
        # Create beacon signal message
        signal = BeaconSignal()
        signal.header.stamp = self. get_clock().now().to_msg()
        signal.uuid = self.beacon_uuid
        signal.major = 1
        signal.minor = 1
        signal.rssi = rssi
        signal.tx_power = self.tx_power
        signal.distance = estimated_distance
        signal.accuracy = 0.5
        signal.measurement_count = 1
        
        # Create array message
        msg = BeaconArray()
        msg.header.stamp = self. get_clock().now().to_msg()
        msg.beacons.append(signal)
        msg.scan_duration_ms = 1000
        
        self. beacon_pub.publish(msg)
        
        self.get_logger().debug(
            f'Mock beacon:  dist={true_distance:.2f}m, rssi={rssi}dBm, '
            f'est_dist={estimated_distance:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BeaconMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()