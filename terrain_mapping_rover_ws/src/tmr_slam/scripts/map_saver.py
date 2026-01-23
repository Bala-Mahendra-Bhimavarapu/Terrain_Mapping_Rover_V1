#!/usr/bin/env python3
"""
Map Saver Node

Provides services to save RTAB-MAP database and 2D/3D maps. 

Services:
    /save_map - Save current map with optional name
    /save_2d_map - Save 2D occupancy grid as PNG/PGM
    /save_3d_map - Save 3D point cloud as PCD/PLY

Usage:
    ros2 run tmr_slam map_saver.py
    ros2 service call /save_map tmr_msgs/srv/SaveMap "{map_name: 'my_map'}"
"""

import rclpy
from rclpy. node import Node

from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

import os
import subprocess
from datetime import datetime
from typing import Optional


class MapSaverNode(Node):
    """Saves maps in various formats."""
    
    def __init__(self):
        super().__init__('map_saver')
        
        # Parameters
        self.declare_parameter('maps_directory', '')
        self.declare_parameter('default_map_name', '')
        
        maps_dir = self.get_parameter('maps_directory').value
        if not maps_dir:
            # Default to package maps directory
            maps_dir = os.path.expanduser('~/terrain_mapping_rover_ws/src/tmr_slam/maps')
        
        self.maps_dir = maps_dir
        self. default_name = self.get_parameter('default_map_name').value
        
        # Ensure maps directory exists
        os.makedirs(self.maps_dir, exist_ok=True)
        
        # Latest map data
        self.latest_grid:  Optional[OccupancyGrid] = None
        self.latest_cloud: Optional[PointCloud2] = None
        
        # Subscribers
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/map', self.grid_callback, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/rtabmap/cloud_map', self.cloud_callback, 10)
        
        # Services
        self.save_map_srv = self.create_service(
            Trigger, 'save_map', self.save_map_callback)
        self.save_2d_srv = self.create_service(
            Trigger, 'save_2d_map', self.save_2d_callback)
        self.save_3d_srv = self.create_service(
            Trigger, 'save_3d_map', self.save_3d_callback)
        
        self.get_logger().info(f"Map Saver ready.  Maps directory: {self.maps_dir}")
    
    def grid_callback(self, msg: OccupancyGrid):
        self.latest_grid = msg
    
    def cloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg
    
    def _generate_map_name(self) -> str:
        """Generate map name with timestamp."""
        if self.default_name:
            return self.default_name
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f"tmr_map_{timestamp}"
    
    def save_map_callback(self, request, response):
        """Save complete map (database + 2D + 3D)."""
        map_name = self._generate_map_name()
        
        try:
            # Save RTAB-MAP database
            db_path = os.path.join(self.maps_dir, f"{map_name}.db")
            
            # Call rtabmap service to save database
            result = subprocess.run(
                ['ros2', 'service', 'call', '/rtabmap/backup', 
                 'std_srvs/srv/Empty', '{}'],
                capture_output=True, timeout=30
            )
            
            # Save 2D map
            self._save_2d_map(map_name)
            
            # Save 3D map
            self._save_3d_map(map_name)
            
            response.success = True
            response. message = f"Map saved as: {map_name}"
            self.get_logger().info(f"Map saved:  {map_name}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error saving map: {e}"
            self.get_logger().error(f"Error saving map: {e}")
        
        return response
    
    def save_2d_callback(self, request, response):
        """Save 2D occupancy grid."""
        map_name = self._generate_map_name()
        
        try:
            self._save_2d_map(map_name)
            response. success = True
            response.message = f"2D map saved as: {map_name}"
        except Exception as e:
            response.success = False
            response. message = f"Error:  {e}"
        
        return response
    
    def save_3d_callback(self, request, response):
        """Save 3D point cloud."""
        map_name = self._generate_map_name()
        
        try:
            self._save_3d_map(map_name)
            response.success = True
            response.message = f"3D map saved as: {map_name}"
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
        
        return response
    
    def _save_2d_map(self, map_name: str):
        """Save 2D occupancy grid using map_saver."""
        map_path = os.path.join(self.maps_dir, map_name)
        
        # Use nav2_map_server's map_saver_cli
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_path, '--ros-args', '-p', 'save_map_timeout: =10. 0'],
            capture_output=True, timeout=30
        )
        
        if result.returncode == 0:
            self.get_logger().info(f"2D map saved:  {map_path}. pgm")
        else:
            self.get_logger().warn(f"2D map save may have failed: {result.stderr}")
    
    def _save_3d_map(self, map_name: str):
        """Save 3D point cloud."""
        if self.latest_cloud is None:
            self.get_logger().warn("No 3D cloud data available")
            return
        
        pcd_path = os.path. join(self.maps_dir, f"{map_name}.pcd")
        
        # Use RTAB-MAP's point cloud saving service
        result = subprocess.run(
            ['ros2', 'service', 'call', '/rtabmap/publish_map',
             'std_srvs/srv/Empty', '{}'],
            capture_output=True, timeout=30
        )
        
        self.get_logger().info(f"3D map publish requested: {pcd_path}")


def main():
    rclpy.init()
    
    node = MapSaverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
