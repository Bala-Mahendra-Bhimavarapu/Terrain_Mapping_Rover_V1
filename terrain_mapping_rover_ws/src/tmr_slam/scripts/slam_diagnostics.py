#!/usr/bin/env python3
"""
SLAM Diagnostics Node

Monitors RTAB-MAP SLAM health and performance.

Usage:
    ros2 run tmr_slam slam_diagnostics.py
"""

import rclpy
from rclpy. node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, Int32
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, PointCloud2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import os
import time
from collections import deque
from typing import Optional
from dataclasses import dataclass


@dataclass
class TopicStats:
    """Statistics for a topic."""
    name: str
    count: int = 0
    last_time: float = 0.0
    rate: float = 0.0
    timestamps: deque = None
    
    def __post_init__(self):
        if self.timestamps is None:
            self.timestamps = deque(maxlen=100)


class SLAMDiagnosticsNode(Node):
    """Monitors SLAM performance."""
    
    def __init__(self):
        super().__init__('slam_diagnostics')
        
        # Parameters
        self.declare_parameter('update_rate', 1.0)
        self.update_rate = self.get_parameter('update_rate').value
        
        # Topic statistics
        self.topics = {
            'rgb': TopicStats('/camera/image_raw'),
            'depth': TopicStats('/tof/depth/image_raw'),
            'odom': TopicStats('/odometry/filtered'),
            'map': TopicStats('/map'),
            'cloud': TopicStats('/rtabmap/cloud_map'),
        }
        
        # SLAM statistics
        self.node_count = 0
        self.loop_closure_count = 0
        self.last_loop_closure_time = 0.0
        self.map_size = (0, 0)  # width, height
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy. BEST_EFFORT,
            depth=1
        )
        
        # Subscribers for monitoring
        self.create_subscription(
            Image, '/camera/image_raw',
            lambda m: self._update_stats('rgb'), sensor_qos)
        self.create_subscription(
            Image, '/tof/depth/image_raw',
            lambda m:  self._update_stats('depth'), sensor_qos)
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda m: self._update_stats('odom'), 10)
        self.create_subscription(
            OccupancyGrid, '/map', self. map_callback, 10)
        self.create_subscription(
            PointCloud2, '/rtabmap/cloud_map',
            lambda m: self._update_stats('cloud'), 10)
        
        # RTAB-MAP info subscribers (if available)
        self.create_subscription(
            Int32, '/rtabmap/info/nodes',
            lambda m: setattr(self, 'node_count', m.data), 10)
        self.create_subscription(
            Int32, '/rtabmap/info/loop_closures',
            self.loop_closure_callback, 10)
        
        # Publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.diagnostics_callback)
        self.display_timer = self.create_timer(0.5, self.display_callback)
        
        self.get_logger().info("SLAM Diagnostics started")
    
    def _update_stats(self, topic_key: str):
        """Update statistics for a topic."""
        stats = self.topics[topic_key]
        stats.count += 1
        stats.timestamps.append(time.time())
        stats.last_time = time.time()
        
        # Calculate rate
        if len(stats.timestamps) >= 2:
            duration = stats.timestamps[-1] - stats.timestamps[0]
            if duration > 0:
                stats.rate = (len(stats.timestamps) - 1) / duration
    
    def map_callback(self, msg: OccupancyGrid):
        """Process map message."""
        self._update_stats('map')
        self.map_size = (msg. info.width, msg.info. height)
    
    def loop_closure_callback(self, msg):
        """Track loop closures."""
        new_count = msg.data
        if new_count > self.loop_closure_count:
            self.last_loop_closure_time = time.time()
            self.get_logger().info(f"Loop closure detected!  Total: {new_count}")
        self.loop_closure_count = new_count
    
    def diagnostics_callback(self):
        """Publish diagnostics."""
        diag_msg = DiagnosticArray()
        diag_msg.header. stamp = self.get_clock().now().to_msg()
        
        # SLAM Status
        slam_status = DiagnosticStatus()
        slam_status.name = "SLAM:  RTAB-MAP"
        slam_status.hardware_id = "tmr_slam"
        
        # Check inputs
        rgb_ok = self.topics['rgb'].rate > 5
        depth_ok = self.topics['depth'].rate > 5
        odom_ok = self. topics['odom'].rate > 20
        
        slam_status.values.append(KeyValue(key='RGB Rate', value=f"{self.topics['rgb'].rate:.1f} Hz"))
        slam_status.values. append(KeyValue(key='Depth Rate', value=f"{self.topics['depth'].rate:.1f} Hz"))
        slam_status.values.append(KeyValue(key='Odom Rate', value=f"{self.topics['odom'].rate:. 1f} Hz"))
        slam_status.values.append(KeyValue(key='Map Nodes', value=str(self.node_count)))
        slam_status.values.append(KeyValue(key='Loop Closures', value=str(self.loop_closure_count)))
        slam_status.values.append(KeyValue(key='Map Size', value=f"{self.map_size[0]}x{self.map_size[1]}"))
        
        if rgb_ok and depth_ok and odom_ok:
            slam_status.level = DiagnosticStatus.OK
            slam_status.message = "SLAM running normally"
        elif odom_ok:
            slam_status.level = DiagnosticStatus. WARN
            slam_status.message = "Sensor input degraded"
        else:
            slam_status.level = DiagnosticStatus.ERROR
            slam_status.message = "SLAM inputs missing"
        
        diag_msg.status. append(slam_status)
        self.diag_pub.publish(diag_msg)
    
    def display_callback(self):
        """Update terminal display."""
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("╔" + "═"*62 + "╗")
        print("║" + "  SLAM DIAGNOSTICS (RTAB-MAP)".center(62) + "║")
        print("╠" + "═"*62 + "╣")
        
        # Input status
        print("║ SENSOR INPUTS: ".ljust(63) + "║")
        for key, stats in self.topics.items():
            if key in ['rgb', 'depth', 'odom']:
                status = "✅" if stats.rate > 5 else "❌"
                print(f"║   {status} {stats.name:30s}:  {stats.rate:6.1f} Hz". ljust(63) + "║")
        
        print("╠" + "═"*62 + "╣")
        
        # SLAM status
        print("║ SLAM STATUS:".ljust(63) + "║")
        print(f"║   Map Nodes:        {self.node_count}".ljust(63) + "║")
        print(f"║   Loop Closures:   {self.loop_closure_count}".ljust(63) + "║")
        print(f"║   Map Size:        {self.map_size[0]} x {self.map_size[1]} cells".ljust(63) + "║")
        
        if self.last_loop_closure_time > 0:
            elapsed = time.time() - self.last_loop_closure_time
            print(f"║   Last Loop:        {elapsed:.1f}s ago".ljust(63) + "║")
        
        print("╠" + "═"*62 + "╣")
        
        # Output status
        print("║ OUTPUTS:".ljust(63) + "║")
        map_rate = self.topics['map'].rate
        cloud_rate = self.topics['cloud'].rate
        map_status = "✅" if map_rate > 0 else "❌"
        cloud_status = "✅" if cloud_rate > 0 else "❌"
        print(f"║   {map_status} 2D Map:    {map_rate:. 2f} Hz".ljust(63) + "║")
        print(f"║   {cloud_status} 3D Cloud:   {cloud_rate:.2f} Hz".ljust(63) + "║")
        
        print("╠" + "═"*62 + "╣")
        
        # Overall status
        all_ok = all(t.rate > 0 for t in self.topics.values())
        if all_ok:
            print("║  ✅ SLAM OPERATIONAL".ljust(63) + "║")
        else:
            print("║  ⚠️  SLAM DEGRADED".ljust(63) + "║")
        
        print("╠" + "═"*62 + "╣")
        print("║  Press Ctrl+C to exit".ljust(63) + "║")
        print("╚" + "═"*62 + "╝")


def main():
    rclpy.init()
    
    node = SLAMDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
