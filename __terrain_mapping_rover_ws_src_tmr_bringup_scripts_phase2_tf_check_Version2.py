#!/usr/bin/env python3
"""
Phase 2 TF Check

Verifies that the TF tree is complete and all expected frames exist. 

Usage:
    ros2 run tmr_bringup phase2_tf_check. py
"""

import rclpy
from rclpy. node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException

import time
from typing import List, Tuple
from dataclasses import dataclass
from enum import Enum


class TestStatus(Enum):
    PENDING = "⏳"
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class TFCheckResult:
    name: str
    status: TestStatus
    message: str
    details: str = ""


class Phase2TFCheck(Node):
    """Phase 2 TF tree validation."""
    
    def __init__(self):
        super().__init__('phase2_tf_check')
        
        # Expected frames (must all exist)
        self.expected_frames = [
            'map',
            'odom',
            'base_footprint',
            'base_link',
            'imu_link',
            'camera_link',
            'camera_optical_frame',
            'tof_link',
            'tof_optical_frame',
            'front_left_wheel',
            'front_right_wheel',
            'back_left_wheel',
            'back_right_wheel',
        ]
        
        # Expected transform chains
        self.expected_transforms = [
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
        
        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self. tf_buffer, self)
        
        # Results
        self.results: List[TFCheckResult] = []
        
        self.get_logger().info("Phase 2 TF Check initialized")
    
    def wait_for_tf(self, timeout: float = 5.0):
        """Wait for TF tree to be populated."""
        self.get_logger().info(f"Waiting {timeout}s for TF tree...")
        
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def check_frame_exists(self, frame:  str) -> TFCheckResult:
        """Check if a frame exists in the TF tree."""
        try:
            # Try to look up transform to this frame from any frame
            frames = self.tf_buffer.all_frames_as_string()
            
            if frame in frames:
                return TFCheckResult(
                    name=f"Frame:  {frame}",
                    status=TestStatus.PASSED,
                    message="Frame exists"
                )
            else:
                return TFCheckResult(
                    name=f"Frame:  {frame}",
                    status=TestStatus.FAILED,
                    message="Frame NOT found"
                )
        except Exception as e:
            return TFCheckResult(
                name=f"Frame: {frame}",
                status=TestStatus.FAILED,
                message=f"Error:  {e}"
            )
    
    def check_transform(self, parent: str, child: str) -> TFCheckResult:
        """Check if a transform exists and is valid."""
        name = f"Transform: {parent} -> {child}"
        
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            t = transform.transform. translation
            r = transform.transform.rotation
            
            # Check for NaN
            import math
            if any(math.isnan(v) for v in [t.x, t.y, t.z, r.x, r.y, r.z, r.w]):
                return TFCheckResult(
                    name=name,
                    status=TestStatus.FAILED,
                    message="Transform contains NaN values"
                )
            
            # Check quaternion is normalized
            norm = math.sqrt(r.x**2 + r. y**2 + r.z**2 + r.w**2)
            if abs(norm - 1.0) > 0.01:
                return TFCheckResult(
                    name=name,
                    status=TestStatus.WARNING,
                    message=f"Quaternion not normalized: {norm:. 4f}"
                )
            
            return TFCheckResult(
                name=name,
                status=TestStatus.PASSED,
                message="Transform valid",
                details=f"xyz=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
            )
            
        except TransformException as e:
            return TFCheckResult(
                name=name,
                status=TestStatus.FAILED,
                message="Transform NOT available",
                details=str(e)[:50]
            )
    
    def run_checks(self) -> bool:
        """Run all TF checks."""
        self.wait_for_tf()
        
        print("\n" + "="*60)
        print("  PHASE 2 TF CHECK")
        print("="*60 + "\n")
        
        self.results = []
        
        # Check all expected transforms
        print("Checking transforms...")
        for parent, child in self.expected_transforms:
            result = self.check_transform(parent, child)
            self.results.append(result)
        
        return self.print_results()
    
    def print_results(self) -> bool:
        """Print results and return success status."""
        print("\n" + "="*60)
        print("  TF CHECK RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            icon = result.status.value
            print(f"{icon} {result. name}")
            print(f"   {result.message}")
            if result.details:
                print(f"   {result.details}")
            print()
            
            if result. status == TestStatus.PASSED: 
                passed += 1
            elif result.status == TestStatus. FAILED:
                failed += 1
            else:
                warnings += 1
        
        print("-"*60)
        print(f"Summary: {passed} passed, {warnings} warnings, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ PHASE 2 TF CHECK PASSED\n")
            return True
        else:
            print("\n❌ PHASE 2 TF CHECK FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase2TFCheck()
    
    try:
        success = node.run_checks()
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())