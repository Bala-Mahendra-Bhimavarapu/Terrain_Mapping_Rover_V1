#!/usr/bin/env python3
"""
Loop Closure Monitor

Monitors and logs loop closure events from RTAB-MAP. 
Useful for debugging SLAM performance.

Usage:
    ros2 run tmr_slam loop_closure_monitor.py
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32
from visualization_msgs.msg import MarkerArray

import time
from datetime import datetime
from typing import List
from dataclasses import dataclass


@dataclass
class LoopClosureEvent:
    """Record of a loop closure event."""
    timestamp: float
    node_id: int
    matched_id: int
    confidence: float


class LoopClosureMonitorNode(Node):
    """Monitors loop closure events."""
    
    def __init__(self):
        super().__init__('loop_closure_monitor')
        
        # Parameters
        self.declare_parameter('log_to_file', False)
        self.declare_parameter('log_file', 'loop_closures.log')
        
        self.log_to_file = self.get_parameter('log_to_file').value
        self.log_file = self.get_parameter('log_file').value
        
        # State
        self.loop_closure_count = 0
        self.events: List[LoopClosureEvent] = []
        self.start_time = time.time()
        
        # Subscribers
        self.create_subscription(
            Int32, '/rtabmap/info/loop_closures',
            self. loop_closure_count_callback, 10)
        self.create_subscription(
            MarkerArray, '/rtabmap/mapGraph',
            self.map_graph_callback, 10)
        
        # Log file
        if self.log_to_file:
            self. log_handle = open(self.log_file, 'a')
            self.log_handle.write(f"\n--- Session started:  {datetime.now().isoformat()} ---\n")
        
        self.get_logger().info("Loop Closure Monitor started")
    
    def loop_closure_count_callback(self, msg):
        """Handle loop closure count update."""
        new_count = msg.data
        
        if new_count > self.loop_closure_count:
            # New loop closure detected
            elapsed = time.time() - self.start_time
            
            event = LoopClosureEvent(
                timestamp=time.time(),
                node_id=0,  # Would need more info from RTAB-MAP
                matched_id=0,
                confidence=0.0
            )
            self.events.append(event)
            
            log_msg = f"Loop closure #{new_count} at {elapsed:.1f}s"
            self.get_logger().info(log_msg)
            
            if self.log_to_file:
                self.log_handle. write(f"{datetime.now().isoformat()}: {log_msg}\n")
                self.log_handle.flush()
        
        self.loop_closure_count = new_count
    
    def map_graph_callback(self, msg:  MarkerArray):
        """Process map graph for loop closure visualization."""
        # Count link types in markers
        loop_links = 0
        for marker in msg. markers:
            # Loop closure links typically have a specific color/namespace
            if 'loop' in marker.ns. lower():
                loop_links += 1
    
    def get_summary(self) -> str:
        """Get summary of loop closures."""
        elapsed = time.time() - self.start_time
        rate = self.loop_closure_count / elapsed * 60 if elapsed > 0 else 0
        
        summary = f"""
Loop Closure Summary
====================
Total Loop Closures:  {self.loop_closure_count}
Runtime: {elapsed:.1f} seconds
Rate: {rate:.2f} per minute
"""
        return summary
    
    def destroy_node(self):
        """Clean up."""
        if self. log_to_file:
            self.log_handle.write(self.get_summary())
            self.log_handle.close()
        super().destroy_node()


def main():
    rclpy.init()
    
    node = LoopClosureMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        print(node.get_summary())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
