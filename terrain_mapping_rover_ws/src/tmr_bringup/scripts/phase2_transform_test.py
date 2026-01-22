#!/usr/bin/env python3
"""
Phase 2 Transform Test

Tests specific transform values against expected measurements.

Usage:
    ros2 run tmr_bringup phase2_transform_test.py
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException

import time
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class TestStatus(Enum):
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class TransformTestResult:
    name: str
    status: TestStatus
    expected:  str
    actual: str
    message: str


class Phase2TransformTest(Node):
    """Tests transform values against expected."""
    
    def __init__(self):
        super().__init__('phase2_transform_test')
        
        # Position tolerance (meters)
        self.position_tolerance = 0.01
        
        # Expected transform values (parent, child, expected_xyz)
        self.expected_transforms = [
            ('base_footprint', 'base_link', (0.0, 0.0, 0.0508)),  # 2 inches up
            ('base_link', 'imu_link', (0.0, 0.0, 0.046)),
            ('base_link', 'camera_link', (0.13, -0.0275, 0.078)),
            ('base_link', 'tof_link', (0.13, 0.014, 0.078)),
        ]
        
        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.results: List[TransformTestResult] = []
        
        self. get_logger().info("Phase 2 Transform Test initialized")
    
    def wait_for_tf(self, timeout: float = 5.0):
        """Wait for TF tree to be populated."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def test_transform_value(
        self, 
        parent:  str, 
        child: str, 
        expected_xyz: Tuple[float, float, float]
    ) -> TransformTestResult:
        """Test a transform against expected value."""
        name = f"{parent} -> {child}"
        
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )
            
            t = transform.transform. translation
            actual = (t.x, t.y, t.z)
            
            # Check each component
            errors = []
            for i, (a, e, axis) in enumerate(zip(actual, expected_xyz, ['X', 'Y', 'Z'])):
                if abs(a - e) > self.position_tolerance:
                    errors.append(f"{axis}:  {a:.4f} vs {e:.4f}")
            
            expected_str = f"({expected_xyz[0]:.4f}, {expected_xyz[1]:.4f}, {expected_xyz[2]:.4f})"
            actual_str = f"({actual[0]:.4f}, {actual[1]:.4f}, {actual[2]:.4f})"
            
            if not errors:
                return TransformTestResult(
                    name=name,
                    status=TestStatus.PASSED,
                    expected=expected_str,
                    actual=actual_str,
                    message="Transform value correct"
                )
            else:
                return TransformTestResult(
                    name=name,
                    status=TestStatus.WARNING,
                    expected=expected_str,
                    actual=actual_str,
                    message=f"Differs: {', '.join(errors)}"
                )
                
        except TransformException as e:
            return TransformTestResult(
                name=name,
                status=TestStatus.FAILED,
                expected=str(expected_xyz),
                actual="N/A",
                message=f"Transform not available: {e}"
            )
    
    def run_tests(self) -> bool:
        """Run all transform tests."""
        print("\n" + "="*60)
        print("  PHASE 2 TRANSFORM VALUE TEST")
        print("="*60)
        
        self.wait_for_tf()
        
        self.results = []
        
        print("\nTesting transform values.. .\n")
        
        for parent, child, expected in self.expected_transforms:
            result = self.test_transform_value(parent, child, expected)
            self.results.append(result)
        
        return self.print_results()
    
    def print_results(self) -> bool:
        """Print results."""
        print("\n" + "-"*60)
        print("RESULTS:")
        print("-"*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            icon = result.status.value
            print(f"{icon} {result.name}")
            print(f"   Expected: {result.expected}")
            print(f"   Actual:   {result.actual}")
            print(f"   {result.message}")
            print()
            
            if result.status == TestStatus.PASSED: 
                passed += 1
            elif result.status == TestStatus. FAILED:
                failed += 1
            else:
                warnings += 1
        
        print("-"*60)
        print(f"Summary: {passed} passed, {warnings} warnings, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ TRANSFORM VALUE TEST PASSED\n")
            return True
        else: 
            print("\n❌ TRANSFORM VALUE TEST FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = Phase2TransformTest()
    
    try:
        success = node. run_tests()
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
