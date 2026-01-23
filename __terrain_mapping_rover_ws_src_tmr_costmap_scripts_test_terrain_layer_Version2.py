#!/usr/bin/env python3
"""
Test Terrain Layer

Tests the terrain classification layer with various scenarios.

Usage:
    ros2 run tmr_costmap test_terrain_layer.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

import numpy as np
import struct
import time
from typing import List, Tuple
from dataclasses import dataclass
from enum import Enum


class TestStatus(Enum):
    PASSED = "✅"
    FAILED = "❌"
    WARNING = "⚠️"


@dataclass
class TestResult:
    name: str
    status: TestStatus
    message: str
    details: str = ""


class TerrainLayerTestNode(Node):
    """Tests terrain layer functionality."""
    
    def __init__(self):
        super().__init__('test_terrain_layer')
        
        # State
        self.costmap_received = False
        self.viz_received = False
        self.latest_costmap = None
        self.latest_viz = None
        
        # Results
        self.results: List[TestResult] = []
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self.costmap_callback, 10)
        self.viz_sub = self.create_subscription(
            MarkerArray, '/terrain_layer/visualization',
            self.viz_callback, 10)
        
        # Publisher for test point clouds
        self.pc_pub = self.create_publisher(
            PointCloud2, '/tof/points_test', sensor_qos)
        
        self.get_logger().info("Terrain Layer Test Node initialized")
    
    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_received = True
        self.latest_costmap = msg
    
    def viz_callback(self, msg: MarkerArray):
        self.viz_received = True
        self.latest_viz = msg
    
    def create_test_point_cloud(
        self,
        points: List[Tuple[float, float, float]],
        frame_id: str = "base_link"
    ) -> PointCloud2:
        """Create a PointCloud2 message from a list of (x, y, z) points."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.height = 1
        msg.width = len(points)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack data
        data = []
        for x, y, z in points:
            data.extend(struct.pack('fff', x, y, z))
        msg.data = bytes(data)
        
        return msg
    
    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """Wait for costmap data."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.costmap_received:
                return True
        return False
    
    def test_costmap_receiving(self) -> TestResult:
        """Test that costmap is being received."""
        if self.wait_for_data(timeout=5.0):
            return TestResult(
                name="Costmap Reception",
                status=TestStatus.PASSED,
                message="Costmap data received"
            )
        else:
            return TestResult(
                name="Costmap Reception",
                status=TestStatus.FAILED,
                message="No costmap data received",
                details="Check that costmap node is running"
            )
    
    def test_costmap_size(self) -> TestResult:
        """Test costmap has reasonable size."""
        if self.latest_costmap is None:
            return TestResult(
                name="Costmap Size",
                status=TestStatus.FAILED,
                message="No costmap available"
            )
        
        width = self.latest_costmap.info.width
        height = self.latest_costmap.info.height
        resolution = self.latest_costmap.info.resolution
        
        if width > 0 and height > 0:
            return TestResult(
                name="Costmap Size",
                status=TestStatus.PASSED,
                message=f"Size: {width}x{height} @ {resolution}m resolution",
                details=f"Physical size: {width*resolution:.1f}m x {height*resolution:.1f}m"
            )
        else:
            return TestResult(
                name="Costmap Size",
                status=TestStatus.FAILED,
                message="Invalid costmap size"
            )
    
    def test_costmap_contains_obstacles(self) -> TestResult:
        """Test that costmap contains obstacle information."""
        if self.latest_costmap is None:
            return TestResult(
                name="Obstacle Detection",
                status=TestStatus.FAILED,
                message="No costmap available"
            )
        
        # Count different cost values
        costs = list(self.latest_costmap.data)
        
        free_count = costs.count(0)
        lethal_count = costs.count(254)
        unknown_count = costs.count(255)
        other_count = len(costs) - free_count - lethal_count - unknown_count
        
        total = len(costs)
        
        if other_count > 0 or lethal_count > 0:
            return TestResult(
                name="Obstacle Detection",
                status=TestStatus.PASSED,
                message=f"Obstacles detected",
                details=f"Free: {free_count}, Obstacles: {other_count + lethal_count}, Unknown: {unknown_count}"
            )
        elif total > 0:
            return TestResult(
                name="Obstacle Detection",
                status=TestStatus.WARNING,
                message="No obstacles in costmap",
                details="May be normal if environment is clear"
            )
        else:
            return TestResult(
                name="Obstacle Detection",
                status=TestStatus.FAILED,
                message="Empty costmap"
            )
    
    def test_visualization_markers(self) -> TestResult:
        """Test that visualization markers are published."""
        # Wait for visualization
        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.viz_received:
                break
        
        if self.viz_received and self.latest_viz:
            marker_count = len(self.latest_viz.markers)
            return TestResult(
                name="Visualization",
                status=TestStatus.PASSED,
                message=f"Received {marker_count} visualization markers"
            )
        else:
            return TestResult(
                name="Visualization",
                status=TestStatus.WARNING,
                message="No visualization markers received",
                details="Visualization may be disabled"
            )
    
    def test_costmap_frame(self) -> TestResult:
        """Test costmap frame ID is correct."""
        if self.latest_costmap is None:
            return TestResult(
                name="Costmap Frame",
                status=TestStatus.FAILED,
                message="No costmap available"
            )
        
        frame_id = self.latest_costmap.header.frame_id
        expected_frames = ['odom', 'map', 'base_link']
        
        if frame_id in expected_frames:
            return TestResult(
                name="Costmap Frame",
                status=TestStatus.PASSED,
                message=f"Frame ID: {frame_id}"
            )
        else:
            return TestResult(
                name="Costmap Frame",
                status=TestStatus.WARNING,
                message=f"Unexpected frame: {frame_id}",
                details=f"Expected one of: {expected_frames}"
            )
    
    def run_all_tests(self) -> bool:
        """Run all tests."""
        print("\n" + "="*60)
        print("  TERRAIN LAYER TESTS")
        print("="*60 + "\n")
        
        print("Waiting for costmap data...")
        
        self.results = [
            self.test_costmap_receiving(),
            self.test_costmap_size(),
            self.test_costmap_frame(),
            self.test_costmap_contains_obstacles(),
            self.test_visualization_markers(),
        ]
        
        return self.print_results()
    
    def print_results(self) -> bool:
        """Print test results."""
        print("\n" + "="*60)
        print("  TEST RESULTS")
        print("="*60 + "\n")
        
        passed = 0
        failed = 0
        warnings = 0
        
        for result in self.results:
            icon = result.status.value
            print(f"{icon} {result.name}")
            print(f"   {result.message}")
            if result.details:
                print(f"   {result.details}")
            print()
            
            if result.status == TestStatus.PASSED:
                passed += 1
            elif result.status == TestStatus.FAILED:
                failed += 1
            else:
                warnings += 1
        
        print("-"*60)
        print(f"Summary: {passed} passed, {warnings} warnings, {failed} failed")
        print("-"*60)
        
        if failed == 0:
            print("\n✅ TERRAIN LAYER TESTS PASSED\n")
            return True
        else:
            print("\n❌ TERRAIN LAYER TESTS FAILED\n")
            return False


def main():
    rclpy.init()
    
    node = TerrainLayerTestNode()
    
    try:
        success = node.run_all_tests()
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())