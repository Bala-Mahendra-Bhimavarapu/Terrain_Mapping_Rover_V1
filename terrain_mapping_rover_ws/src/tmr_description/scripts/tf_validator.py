#!/usr/bin/env python3
"""
TF Validator Node

Validates the TF tree against expected structure and reports errors.
Runs once and exits with success/failure code.

Usage:
    ros2 run tmr_description tf_validator.py
"""

import rclpy
from rclpy. node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException

import sys
import time
import math
from typing import List, Tuple
from dataclasses import dataclass
from enum import Enum


class ValidationResult(Enum):
    PASS = "✅"
    FAIL = "❌"
    WARN = "⚠️"


@dataclass
class ValidationCheck:
    name: str
    result: ValidationResult
    message: str
    details: str = ""


class TFValidatorNode(Node):
    """Validates TF tree structure."""
    
    def __init__(self):
        super().__init__('tf_validator')
        
        # Parameters
        self. declare_parameter('wait_time', 5.0)
        self.wait_time = self.get_parameter('wait_time').value
        
        # TF buffer and listener
        self. tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Validation results
        self.results: List[ValidationCheck] = []
        
        self.get_logger().info("TF Validator starting...")
    
    def wait_for_tf(self):
        """Wait for TF tree to be populated."""
        self.get_logger().info(f"Waiting {self.wait_time}s for TF tree...")
        
        start = time.time()
        while time.time() - start < self.wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def check_transform_exists(self, parent: str, child: str) -> ValidationCheck:
        """Check if a transform exists."""
        name = f"Transform {parent} -> {child}"
        
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child,
                rclpy.time.Time(),
                timeout=Duration(seconds=1. 0)
            )
            
            t = transform.transform.translation
            r = transform.transform.rotation
            
            return ValidationCheck(
                name=name,
                result=ValidationResult.PASS,
                message="Transform available",
                details=f"xyz=({t.x:. 4f}, {t.y:.4f}, {t.z:.4f})"
            )
            
        except TransformException as e:
            return ValidationCheck(
                name=name,
                result=ValidationResult.FAIL,
                message="Transform NOT available",
                details=str(e)[:50]
            )
    
    def check_transform_value(
        self, 
        parent: str, 
        child: str,
        expected_xyz: Tuple[float, float, float],
        tolerance: float = 0.01
    ) -> ValidationCheck:
        """Check if transform has expected value."""
        name = f"Value {parent} -> {child}"
        
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            t = transform.transform.translation
            actual = (t.x, t.y, t.z)
            
            # Check each component
            errors = []
            for i, (a, e, axis) in enumerate(zip(actual, expected_xyz, ['X', 'Y', 'Z'])):
                if abs(a - e) > tolerance:
                    errors.append(f"{axis}:  {a:.4f} vs {e:.4f}")
            
            if not errors:
                return ValidationCheck(
                    name=name,
                    result=ValidationResult. PASS,
                    message="Transform value correct",
                    details=f"xyz=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
                )
            else:
                return ValidationCheck(
                    name=name,
                    result=ValidationResult. WARN,
                    message="Transform value differs",
                    details=", ".join(errors)
                )
                
        except TransformException as e:
            return ValidationCheck(
                name=name,
                result=ValidationResult.FAIL,
                message="Could not check value",
                details=str(e)[:50]
            )
    
    def check_optical_frame_orientation(
        self, 
        camera_frame: str, 
        optical_frame: str
    ) -> ValidationCheck:
        """Check optical frame has correct orientation."""
        name = f"Optical frame {optical_frame}"
        
        try:
            transform = self.tf_buffer. lookup_transform(
                camera_frame, optical_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            r = transform.transform.rotation
            
            # Expected:  -90° about Z, then -90° about X
            # Quaternion should be approximately (0.5, -0.5, 0.5, -0.5) or similar
            
            # Simple check: translation should be zero
            t = transform.transform.translation
            if abs(t.x) > 0.001 or abs(t.y) > 0.001 or abs(t.z) > 0.001:
                return ValidationCheck(
                    name=name,
                    result=ValidationResult. WARN,
                    message="Optical frame has non-zero translation",
                    details=f"xyz=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
                )
            
            return ValidationCheck(
                name=name,
                result=ValidationResult. PASS,
                message="Optical frame orientation OK",
                details=f"quat=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})"
            )
            
        except TransformException as e:
            return ValidationCheck(
                name=name,
                result=ValidationResult.FAIL,
                message="Could not check optical frame",
                details=str(e)[:50]
            )
    
    def run_validation(self) -> bool:
        """Run all validation checks."""
        self.wait_for_tf()
        
        print("\n" + "=" * 60)
        print("  TF VALIDATION - Terrain Mapping Rover")
        print("=" * 60 + "\n")
        
        # =====================================================================
        # Check 1: Core transforms exist
        # =====================================================================
        print("1. Checking core transforms...")
        
        core_transforms = [
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'imu_link'),
            ('base_link', 'camera_link'),
            ('base_link', 'tof_link'),
        ]
        
        for parent, child in core_transforms:
            self.results.append(self.check_transform_exists(parent, child))
        
        # =====================================================================
        # Check 2: Optical frames exist
        # =====================================================================
        print("2. Checking optical frames...")
        
        self.results.append(self.check_transform_exists('camera_link', 'camera_optical_frame'))
        self.results.append(self.check_transform_exists('tof_link', 'tof_optical_frame'))
        
        # =====================================================================
        # Check 3: Wheel transforms exist
        # =====================================================================
        print("3. Checking wheel transforms...")
        
        wheels = ['front_left_wheel', 'front_right_wheel', 'back_left_wheel', 'back_right_wheel']
        for wheel in wheels:
            self.results.append(self.check_transform_exists('base_link', wheel))
        
        # =====================================================================
        # Check 4: Transform values
        # =====================================================================
        print("4. Checking transform values...")
        
        # base_footprint -> base_link should be 2" up
        self.results.append(self.check_transform_value(
            'base_footprint', 'base_link',
            expected_xyz=(0.0, 0.0, 0.0508),
            tolerance=0.005
        ))
        
        # IMU should be centered
        self.results.append(self.check_transform_value(
            'base_link', 'imu_link',
            expected_xyz=(0.0, 0.0, 0.046),
            tolerance=0.01
        ))
        
        # =====================================================================
        # Check 5: Optical frame orientations
        # =====================================================================
        print("5. Checking optical frame orientations...")
        
        self.results.append(self.check_optical_frame_orientation(
            'camera_link', 'camera_optical_frame'))
        self.results.append(self.check_optical_frame_orientation(
            'tof_link', 'tof_optical_frame'))
        
        # =====================================================================
        # Print results
        # =====================================================================
        
        print("\n" + "-" * 60)
        print("RESULTS:")
        print("-" * 60 + "\n")
        
        passed = 0
        failed = 0
        warned = 0
        
        for check in self.results:
            icon = check.result.value
            print(f"{icon} {check. name}")
            print(f"   {check.message}")
            if check.details:
                print(f"   {check.details}")
            print()
            
            if check. result == ValidationResult.PASS: 
                passed += 1
            elif check.result == ValidationResult. FAIL:
                failed += 1
            else:
                warned += 1
        
        print("=" * 60)
        print(f"Summary: {passed} passed, {warned} warnings, {failed} failed")
        print("=" * 60)
        
        if failed == 0:
            print("\n✅ TF VALIDATION PASSED\n")
            return True
        else:
            print("\n❌ TF VALIDATION FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = TFValidatorNode()
    
    try:
        success = node.run_validation()
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
