"""
Point cloud generator from ToF depth images. 

Converts depth images to 3D point clouds using camera intrinsics.
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class Intrinsics:
    """Camera intrinsics for point cloud generation."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    depth_scale: float = 0.001


class PointCloudGenerator:
    """
    Generates 3D point clouds from depth images.
    
    Uses camera intrinsics to back-project depth pixels to 3D points.
    """
    
    def __init__(self, intrinsics: Optional[Intrinsics] = None):
        """
        Initialize point cloud generator.
        
        Args:
            intrinsics: Camera intrinsics. Can be set later.
        """
        self. intrinsics = intrinsics
        self._pixel_coords:  Optional[np.ndarray] = None
    
    def set_intrinsics(self, intrinsics: Intrinsics):
        """Set camera intrinsics and precompute pixel coordinates."""
        self. intrinsics = intrinsics
        self._precompute_pixel_coords()
    
    def _precompute_pixel_coords(self):
        """Precompute normalized pixel coordinates for efficiency."""
        if self.intrinsics is None:
            return
        
        # Create meshgrid of pixel coordinates
        u = np. arange(self.intrinsics. width)
        v = np.arange(self.intrinsics.height)
        u, v = np.meshgrid(u, v)
        
        # Normalize pixel coordinates
        # x = (u - cx) / fx
        # y = (v - cy) / fy
        self._pixel_coords = np.stack([
            (u - self.intrinsics. cx) / self.intrinsics.fx,
            (v - self.intrinsics.cy) / self.intrinsics. fy,
            np.ones_like(u)
        ], axis=-1).astype(np.float32)
    
    def generate(
        self,
        depth_image: np.ndarray,
        max_depth_m: float = 3.0,
        min_depth_m: float = 0.1
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate point cloud from depth image. 
        
        Args:
            depth_image: Depth image (uint16, raw sensor values)
            max_depth_m: Maximum depth to include (meters)
            min_depth_m: Minimum depth to include (meters)
            
        Returns: 
            Tuple of (points, valid_mask):
            - points: Nx3 array of XYZ points in meters
            - valid_mask:  Boolean mask of valid points
        """
        if self. intrinsics is None or self._pixel_coords is None:
            raise RuntimeError("Intrinsics not set")
        
        # Convert depth to meters
        depth_m = depth_image. astype(np.float32) * self.intrinsics.depth_scale
        
        # Create valid mask
        valid_mask = (depth_m > min_depth_m) & (depth_m < max_depth_m)
        
        # Compute 3D points
        # X = x * Z
        # Y = y * Z
        # Z = depth
        points = self._pixel_coords * depth_m[: , :, np.newaxis]
        
        return points, valid_mask
    
    def generate_flat(
        self,
        depth_image: np.ndarray,
        max_depth_m: float = 3.0,
        min_depth_m: float = 0.1
    ) -> np.ndarray:
        """
        Generate flattened point cloud (only valid points).
        
        Args:
            depth_image: Depth image
            max_depth_m: Maximum depth
            min_depth_m:  Minimum depth
            
        Returns: 
            Nx3 array of valid XYZ points
        """
        points, valid_mask = self.generate(depth_image, max_depth_m, min_depth_m)
        return points[valid_mask]
    
    def to_ros_pointcloud2(
        self,
        depth_image: np.ndarray,
        header,
        max_depth_m: float = 3.0,
        min_depth_m: float = 0.1
    ):
        """
        Convert depth image to ROS PointCloud2 message.
        
        Args:
            depth_image: Depth image
            header: ROS header for the message
            max_depth_m:  Maximum depth
            min_depth_m: Minimum depth
            
        Returns:
            sensor_msgs. msg.PointCloud2
        """
        from sensor_msgs.msg import PointCloud2, PointField
        import struct
        
        points, valid_mask = self.generate(depth_image, max_depth_m, min_depth_m)
        valid_points = points[valid_mask]
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(valid_points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField. FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack point data
        msg.data = valid_points.astype(np.float32).tobytes()
        
        return msg
