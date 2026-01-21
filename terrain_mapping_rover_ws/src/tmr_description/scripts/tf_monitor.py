#!/usr/bin/env python3
"""
TF Monitor Node

Monitors the TF tree and reports status of all expected transforms. 
Useful for debugging TF issues.

Usage:
    ros2 run tmr_description tf_monitor.py
"""

import rclpy
from rclpy. node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException

import time
from typing import List, Dict, Tuple
from dataclasses import dataclass


@dataclass
class TransformStatus:
    """Status of a single transform."""
    parent:  str
    child: str
    available: bool
    age_ms: float
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]
    error:  str = ""


class TFMonitorNode(Node):
    """Monitors TF tree and reports status."""
    
    def __init__(self):
        super().__init__('tf_monitor')
        
        # Parameters
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('timeout_sec', 1.0)
        
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout_sec = self. get_parameter('timeout_sec').value
        
        # Expected transforms (parent -> child)
        self.expected_transforms = [
            ('map', 'odom'),
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'imu_link'),
            ('base_link', 'camera_link'),
            ('camera_link', 'camera_optical_frame'),
            ('base_link', 'tof_link'),
            ('tof_link', 'tof_optical_frame'),
            ('base_link', 'front_left_wheel'),
            ('base_link', 'front_right_wheel'),
            ('base_link', 'back_left_wheel'),
            ('base_link', 'back_right_wheel'),
        ]
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for status updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.check_transforms)
        
        # Status tracking
        self.transform_status:  Dict[str, TransformStatus] = {}
        
        self.get_logger().info("TF Monitor started")
        self.get_logger().info(f"Monitoring {len(self.expected_transforms)} transforms")
    
    def check_transforms(self):
        """Check all expected transforms."""
        current_time = self.get_clock().now()
        
        all_ok = True
        
        for parent, child in self.expected_transforms:
            key = f"{parent}->{child}"
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.timeout_sec)
                )
                
                # Calculate age
                transform_time = rclpy.time.Time. from_msg(transform.header. stamp)
                age_ns = (current_time - transform_time).nanoseconds
                age_ms = age_ns / 1e6
                
                # Extract transform data
                t = transform.transform.translation
                r = transform.transform.rotation
                
                self.transform_status[key] = TransformStatus(
                    parent=parent,
                    child=child,
                    available=True,
                    age_ms=age_ms,
                    translation=(t.x, t.y, t.z),
                    rotation=(r.x, r.y, r.z, r.w)
                )
                
            except TransformException as e: 
                all_ok = False
                self. transform_status[key] = TransformStatus(
                    parent=parent,
                    child=child,
                    available=False,
                    age_ms=-1,
                    translation=(0, 0, 0),
                    rotation=(0, 0, 0, 1),
                    error=str(e)
                )
        
        # Print status
        self.print_status(all_ok)
    
    def print_status(self, all_ok: bool):
        """Print transform status."""
        # Clear screen
        print("\033[2J\033[H", end="")
        
        print("=" * 70)
        print("  TF MONITOR - Terrain Mapping Rover")
        print("=" * 70)
        print()
        
        # Group by status
        available = []
        missing = []
        
        for key, status in self.transform_status.items():
            if status. available:
                available.append((key, status))
            else:
                missing.append((key, status))
        
        # Print available transforms
        print(f"✅ Available Transforms ({len(available)}):")
        print("-" * 70)
        
        for key, status in available:
            t = status.translation
            age_str = f"{status.age_ms:.0f}ms" if status.age_ms < 10000 else "static"
            print(f"  {key: 40s} | xyz=({t[0]: +.3f}, {t[1]:+.3f}, {t[2]:+.3f}) | {age_str}")
        
        print()
        
        # Print missing transforms
        if missing:
            print(f"❌ Missing Transforms ({len(missing)}):")
            print("-" * 70)
            
            for key, status in missing:
                print(f"  {key:40s} | {status.error[: 40]}")
            
            print()
        
        # Overall status
        print("=" * 70)
        if all_ok:
            print("  ✅ All transforms available - TF tree is complete")
        else:
            print(f"  ⚠️  {len(missing)} transforms missing - TF tree incomplete")
        print("=" * 70)
        print("\nPress Ctrl+C to exit")


def main():
    rclpy.init()
    
    node = TFMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
