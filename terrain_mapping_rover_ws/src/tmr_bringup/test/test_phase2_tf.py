#!/usr/bin/env python3
"""
Phase 2 TF Unit Tests

Unit tests for TF tree validation. 

Usage:
    pytest test_phase2_tf.py -v
"""

import unittest
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException


class TestPhase2TF(unittest.TestCase):
    """Unit tests for Phase 2 TF tree."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()
        cls.node = rclpy.create_node('test_phase2_tf')
        
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        
        # Wait for TF tree to populate
        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def _lookup_transform(self, parent:  str, child: str):
        """Helper to lookup transform."""
        try:
            return self.tf_buffer.lookup_transform(
                parent, child,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )
        except TransformException: 
            return None
    
    def test_base_footprint_to_base_link(self):
        """Test base_footprint -> base_link transform exists."""
        transform = self._lookup_transform('base_footprint', 'base_link')
        self.assertIsNotNone(transform, "Transform base_footprint->base_link not found")
        
        # Check Z offset (2 inches = 0.0508m)
        z = transform.transform.translation.z
        self.assertAlmostEqual(z, 0.0508, delta=0.01, 
            msg=f"base_link Z offset wrong: {z}")
    
    def test_base_link_to_imu_link(self):
        """Test base_link -> imu_link transform exists."""
        transform = self._lookup_transform('base_link', 'imu_link')
        self.assertIsNotNone(transform, "Transform base_link->imu_link not found")
    
    def test_base_link_to_camera_link(self):
        """Test base_link -> camera_link transform exists."""
        transform = self._lookup_transform('base_link', 'camera_link')
        self.assertIsNotNone(transform, "Transform base_link->camera_link not found")
    
    def test_camera_optical_frame(self):
        """Test camera_link -> camera_optical_frame exists."""
        transform = self._lookup_transform('camera_link', 'camera_optical_frame')
        self.assertIsNotNone(transform, "Transform camera_link->camera_optical_frame not found")
    
    def test_base_link_to_tof_link(self):
        """Test base_link -> tof_link transform exists."""
        transform = self._lookup_transform('base_link', 'tof_link')
        self.assertIsNotNone(transform, "Transform base_link->tof_link not found")
    
    def test_tof_optical_frame(self):
        """Test tof_link -> tof_optical_frame exists."""
        transform = self._lookup_transform('tof_link', 'tof_optical_frame')
        self.assertIsNotNone(transform, "Transform tof_link->tof_optical_frame not found")
    
    def test_wheel_frames(self):
        """Test all wheel frames exist."""
        wheels = ['front_left_wheel', 'front_right_wheel', 
                  'back_left_wheel', 'back_right_wheel']
        
        for wheel in wheels:
            transform = self._lookup_transform('base_link', wheel)
            self.assertIsNotNone(transform, 
                f"Transform base_link->{wheel} not found")
    
    def test_transform_values_valid(self):
        """Test transform values are not NaN."""
        transform = self._lookup_transform('base_link', 'imu_link')
        if transform is None:
            self.skipTest("Transform not available")
        
        t = transform.transform.translation
        r = transform.transform.rotation
        
        self.assertFalse(math.isnan(t.x), "Translation X is NaN")
        self.assertFalse(math.isnan(t.y), "Translation Y is NaN")
        self.assertFalse(math.isnan(t.z), "Translation Z is NaN")
        self.assertFalse(math.isnan(r.x), "Rotation X is NaN")
        self.assertFalse(math.isnan(r.y), "Rotation Y is NaN")
        self.assertFalse(math.isnan(r.z), "Rotation Z is NaN")
        self.assertFalse(math.isnan(r.w), "Rotation W is NaN")
    
    def test_quaternion_normalized(self):
        """Test quaternions are normalized."""
        transform = self._lookup_transform('base_link', 'camera_link')
        if transform is None: 
            self.skipTest("Transform not available")
        
        r = transform.transform.rotation
        norm = math.sqrt(r.x**2 + r.y**2 + r.z**2 + r.w**2)
        
        self.assertAlmostEqual(norm, 1.0, delta=0.01,
            msg=f"Quaternion not normalized: {norm}")


if __name__ == '__main__':
    unittest.main()
