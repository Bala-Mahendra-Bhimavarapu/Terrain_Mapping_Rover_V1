#!/usr/bin/env python3
"""
Teleop Multiplexer Node

Multiplexes multiple velocity command sources with priority. 
Higher priority inputs override lower priority ones.

Priority (default, highest first):
1. Safety controller
2. Gamepad
3. Keyboard
4. Navigation stack

Usage:
    ros2 run tmr_teleop teleop_mux
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import time
from typing import Dict, List, Optional
from dataclasses import dataclass


@dataclass
class InputSource:
    """Container for input source state"""
    name: str
    topic: str
    priority: int
    last_msg_time: float
    last_twist:  Optional[Twist]
    active: bool


class TeleopMuxNode(Node):
    """
    Teleop multiplexer ROS 2 node.
    
    Selects highest priority active input and forwards to output.
    """
    
    def __init__(self):
        super().__init__('teleop_mux')
        
        # Declare parameters
        self. declare_parameter('input_topics', [
            '/cmd_vel_safety',
            '/cmd_vel_gamepad',
            '/cmd_vel_keyboard',
            '/cmd_vel_nav'
        ])
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('rate', 50.0)
        
        # Get parameters
        input_topics = self.get_parameter('input_topics').value
        output_topic = self.get_parameter('output_topic').value
        self.input_timeout = self.get_parameter('input_timeout').value
        rate = self.get_parameter('rate').value
        
        # Initialize input sources (priority = index, lower = higher priority)
        self.inputs: Dict[str, InputSource] = {}
        self.subscribers = []
        
        for priority, topic in enumerate(input_topics):
            name = topic.split('/')[-1]  # Extract name from topic
            
            self.inputs[topic] = InputSource(
                name=name,
                topic=topic,
                priority=priority,
                last_msg_time=0.0,
                last_twist=None,
                active=False
            )
            
            # Create subscriber
            sub = self.create_subscription(
                Twist, topic,
                lambda msg, t=topic: self.input_callback(msg, t),
                10
            )
            self.subscribers. append(sub)
            
            self.get_logger().info(f'  Input {priority}: {topic}')
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, output_topic, 10)
        
        # Active source publisher (for debugging)
        self.active_source_pub = self.create_publisher(
            String, 'teleop_mux/active_source', 10)
        
        # Current active source
        self.active_source:  Optional[str] = None
        
        # Timer for output
        self.timer = self. create_timer(1.0 / rate, self.update_callback)
        
        self.get_logger().info('Teleop Mux initialized')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Timeout: {self.input_timeout}s')
    
    def input_callback(self, msg: Twist, topic: str):
        """Callback for input topics"""
        if topic not in self.inputs:
            return
        
        source = self.inputs[topic]
        source.last_msg_time = time.time()
        source.last_twist = msg
        
        # Check if this is a non-zero command
        is_moving = (abs(msg.linear.x) > 0.001 or 
                    abs(msg.linear.y) > 0.001 or 
                    abs(msg.angular.z) > 0.001)
        
        source.active = is_moving
    
    def _get_active_source(self) -> Optional[InputSource]:
        """
        Get the highest priority active input source.
        
        Returns:
            InputSource with highest priority, or None if no active source
        """
        current_time = time.time()
        
        # Update active status based on timeout
        for source in self.inputs.values():
            if current_time - source.last_msg_time > self.input_timeout:
                source.active = False
        
        # Find highest priority (lowest number) active source
        active_sources = [
            source for source in self.inputs.values()
            if source. active and source.last_twist is not None
        ]
        
        if not active_sources:
            return None
        
        return min(active_sources, key=lambda s: s.priority)
    
    def update_callback(self):
        """Timer callback to select and publish velocity"""
        active_source = self._get_active_source()
        
        if active_source is not None:
            # Publish from active source
            self.cmd_vel_pub.publish(active_source.last_twist)
            
            # Track source changes
            if self.active_source != active_source. topic:
                self.active_source = active_source.topic
                self.get_logger().info(f'Active source:  {active_source.name}')
        else:
            # No active source, publish zero velocity
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            
            if self.active_source is not None:
                self.active_source = None
                self.get_logger().info('No active source, stopped')
        
        # Publish active source info
        source_msg = String()
        source_msg.data = active_source.name if active_source else 'none'
        self.active_source_pub.publish(source_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = TeleopMuxNode()
    
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