#!/usr/bin/env python3
"""
Terrain Visualizer

Visualizes terrain classification results in RViz.

Usage:
    ros2 run tmr_costmap terrain_visualizer.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid

import struct
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class TerrainStats:
    """Statistics about terrain classification."""
    total_cells: int = 0
    free_cells: int = 0
    low_cost_cells: int = 0
    medium_cost_cells: int = 0
    high_cost_cells: int = 0
    lethal_cells: int = 0
    unknown_cells: int = 0


class TerrainVisualizerNode(Node):
    """Visualizes terrain data."""
    
    def __init__(self):
        super().__init__('terrain_visualizer')
        
        # Parameters
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('point_cloud_topic', '/tof/points')
        self.declare_parameter('publish_rate', 2.0)
        
        costmap_topic = self.get_parameter('costmap_topic').value
        pc_topic = self.get_parameter('point_cloud_topic').value
        rate = self.get_parameter('publish_rate').value
        
        # State
        self.latest_costmap = None
        self.latest_cloud = None
        self.stats = TerrainStats()
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, costmap_topic, self.costmap_callback, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, pc_topic, self.cloud_callback, sensor_qos)
        
        # Publishers
        self.stats_pub = self.create_publisher(
            MarkerArray, 'terrain_stats_markers', 10)
        self.height_pub = self.create_publisher(
            MarkerArray, 'terrain_height_markers', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_visualization)
        
        self.get_logger().info("Terrain Visualizer started")
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Process costmap for statistics."""
        self.latest_costmap = msg
        self.compute_stats(msg)
    
    def cloud_callback(self, msg: PointCloud2):
        """Store latest point cloud."""
        self.latest_cloud = msg
    
    def compute_stats(self, costmap: OccupancyGrid):
        """Compute terrain statistics from costmap."""
        self.stats = TerrainStats()
        
        for cost in costmap.data:
            self.stats.total_cells += 1
            
            if cost == 0:
                self.stats.free_cells += 1
            elif cost < 50:
                self.stats.free_cells += 1
            elif cost < 128:
                self.stats.low_cost_cells += 1
            elif cost < 200:
                self.stats.medium_cost_cells += 1
            elif cost < 254:
                self.stats.high_cost_cells += 1
            elif cost == 254:
                self.stats.lethal_cells += 1
            else:
                self.stats.unknown_cells += 1
    
    def publish_visualization(self):
        """Publish visualization markers."""
        if self.stats.total_cells == 0:
            return
        
        # Publish stats as text marker
        marker_array = MarkerArray()
        
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "terrain_stats"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.5
        text_marker.scale.z = 0.15
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        # Format stats text
        total = self.stats.total_cells
        if total > 0:
            text_marker.text = (
                f"Terrain Stats:\n"
                f"Free: {self.stats.free_cells} ({100*self.stats.free_cells/total:.1f}%)\n"
                f"Low: {self.stats.low_cost_cells} ({100*self.stats.low_cost_cells/total:.1f}%)\n"
                f"Med: {self.stats.medium_cost_cells} ({100*self.stats.medium_cost_cells/total:.1f}%)\n"
                f"High: {self.stats.high_cost_cells} ({100*self.stats.high_cost_cells/total:.1f}%)\n"
                f"Lethal: {self.stats.lethal_cells} ({100*self.stats.lethal_cells/total:.1f}%)"
            )
        
        marker_array.markers.append(text_marker)
        self.stats_pub.publish(marker_array)


def main():
    rclpy.init()
    node = TerrainVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()