"""
Obstacle Detector - Detects obstacles from ToF point cloud or depth image.

Provides simple obstacle detection for reactive avoidance.
"""

import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass


@dataclass
class ObstacleInfo:
    """Information about detected obstacles."""
    front_blocked: bool
    front_distance: float  # Meters to nearest front obstacle
    left_blocked: bool
    left_distance: float
    right_blocked: bool
    right_distance: float
    best_direction: float  # Suggested direction to turn (radians)
    obstacle_count: int


class ObstacleDetector: 
    """
    Detects obstacles from depth data. 
    
    Divides the field of view into sectors and reports
    which directions are blocked.
    """
    
    def __init__(
        self,
        min_range: float = 0.1,
        max_range: float = 2.0,
        obstacle_threshold: float = 0.5,
        robot_width: float = 0.36,
        fov_horizontal: float = 70.0  # Degrees
    ):
        """
        Initialize obstacle detector. 
        
        Args:
            min_range: Minimum valid range (meters)
            max_range:  Maximum detection range (meters)
            obstacle_threshold: Distance to consider blocked (meters)
            robot_width: Robot width for collision checking (meters)
            fov_horizontal: Horizontal field of view (degrees)
        """
        self.min_range = min_range
        self.max_range = max_range
        self.obstacle_threshold = obstacle_threshold
        self.robot_width = robot_width
        self.fov_rad = np.radians(fov_horizontal)
    
    def detect_from_depth_image(
        self,
        depth_image: np.ndarray,
        depth_scale: float = 0.001
    ) -> ObstacleInfo:
        """
        Detect obstacles from depth image.
        
        Args:
            depth_image: Depth image (uint16)
            depth_scale: Scale to convert to meters
            
        Returns: 
            ObstacleInfo with obstacle status
        """
        h, w = depth_image.shape[: 2]
        
        # Convert to meters
        depth_m = depth_image. astype(np.float32) * depth_scale
        
        # Mask invalid readings
        valid_mask = (depth_m > self.min_range) & (depth_m < self.max_range)
        
        # Divide into three sectors:  left, center, right
        third = w // 3
        
        left_sector = depth_m[: , :third]
        center_sector = depth_m[:, third: 2*third]
        right_sector = depth_m[:, 2*third:]
        
        left_valid = left_sector[valid_mask[: , : third]]
        center_valid = center_sector[valid_mask[:, third:2*third]]
        right_valid = right_sector[valid_mask[:, 2*third:]]
        
        # Calculate minimum distance in each sector
        left_dist = np.min(left_valid) if len(left_valid) > 0 else self.max_range
        center_dist = np.min(center_valid) if len(center_valid) > 0 else self.max_range
        right_dist = np.min(right_valid) if len(right_valid) > 0 else self.max_range
        
        # Determine if blocked
        left_blocked = left_dist < self.obstacle_threshold
        front_blocked = center_dist < self.obstacle_threshold
        right_blocked = right_dist < self. obstacle_threshold
        
        # Count obstacles
        obstacle_count = sum([left_blocked, front_blocked, right_blocked])
        
        # Suggest best direction
        best_direction = self._calculate_best_direction(
            left_dist, center_dist, right_dist,
            left_blocked, front_blocked, right_blocked
        )
        
        return ObstacleInfo(
            front_blocked=front_blocked,
            front_distance=center_dist,
            left_blocked=left_blocked,
            left_distance=left_dist,
            right_blocked=right_blocked,
            right_distance=right_dist,
            best_direction=best_direction,
            obstacle_count=obstacle_count
        )
    
    def detect_from_pointcloud(
        self,
        points: np.ndarray  # Nx3 array of x,y,z points
    ) -> ObstacleInfo:
        """
        Detect obstacles from point cloud.
        
        Args:
            points:  Nx3 array of points in robot frame
            
        Returns: 
            ObstacleInfo with obstacle status
        """
        if len(points) == 0:
            return ObstacleInfo(
                front_blocked=False, front_distance=self.max_range,
                left_blocked=False, left_distance=self.max_range,
                right_blocked=False, right_distance=self.max_range,
                best_direction=0.0, obstacle_count=0
            )
        
        x = points[:, 0]  # Forward
        y = points[:, 1]  # Left
        z = points[:, 2]  # Up
        
        # Filter by height (ignore ground and high obstacles)
        height_mask = (z > 0. 05) & (z < 0.5)
        
        # Filter by range
        distances = np.sqrt(x**2 + y**2)
        range_mask = (distances > self.min_range) & (distances < self.max_range)
        
        valid_mask = height_mask & range_mask
        
        if not np.any(valid_mask):
            return ObstacleInfo(
                front_blocked=False, front_distance=self.max_range,
                left_blocked=False, left_distance=self. max_range,
                right_blocked=False, right_distance=self.max_range,
                best_direction=0.0, obstacle_count=0
            )
        
        x_valid = x[valid_mask]
        y_valid = y[valid_mask]
        dist_valid = distances[valid_mask]
        
        # Calculate angles
        angles = np.arctan2(y_valid, x_valid)
        
        # Sector boundaries (in front of robot)
        left_mask = (angles > 0.2) & (angles < self.fov_rad / 2)
        center_mask = (angles >= -0.2) & (angles <= 0.2)
        right_mask = (angles < -0.2) & (angles > -self.fov_rad / 2)
        
        # Minimum distances per sector
        left_dist = np.min(dist_valid[left_mask]) if np.any(left_mask) else self.max_range
        center_dist = np.min(dist_valid[center_mask]) if np.any(center_mask) else self.max_range
        right_dist = np.min(dist_valid[right_mask]) if np.any(right_mask) else self.max_range
        
        # Determine if blocked
        left_blocked = left_dist < self.obstacle_threshold
        front_blocked = center_dist < self.obstacle_threshold
        right_blocked = right_dist < self.obstacle_threshold
        
        obstacle_count = sum([left_blocked, front_blocked, right_blocked])
        
        best_direction = self._calculate_best_direction(
            left_dist, center_dist, right_dist,
            left_blocked, front_blocked, right_blocked
        )
        
        return ObstacleInfo(
            front_blocked=front_blocked,
            front_distance=center_dist,
            left_blocked=left_blocked,
            left_distance=left_dist,
            right_blocked=right_blocked,
            right_distance=right_dist,
            best_direction=best_direction,
            obstacle_count=obstacle_count
        )
    
    def _calculate_best_direction(
        self,
        left_dist: float,
        center_dist: float,
        right_dist: float,
        left_blocked: bool,
        front_blocked: bool,
        right_blocked: bool
    ) -> float:
        """Calculate best direction to avoid obstacles."""
        
        if not front_blocked: 
            # Front is clear, go straight
            return 0.0
        
        if not left_blocked and not right_blocked:
            # Both sides clear, turn toward more open side
            if left_dist > right_dist:
                return 0.5  # Turn left
            else:
                return -0.5  # Turn right
        
        if not left_blocked: 
            return 0.7  # Turn left
        
        if not right_blocked:
            return -0.7  # Turn right
        
        # All blocked - turn around
        return 3.14 if left_dist > right_dist else -3.14