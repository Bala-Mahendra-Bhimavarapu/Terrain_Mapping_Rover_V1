"""
Point Cloud Generator

Converts ToF depth images to 3D point clouds. 

Uses the pinhole camera model to project depth pixels to 3D points: 
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = depth
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

try:
    from sensor_msgs.msg import PointCloud2, PointField
    import sensor_msgs_py.point_cloud2 as pc2
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters"""
    fx: float  # Focal length x
    fy: float  # Focal length y
    cx:  float  # Principal point x
    cy: float  # Principal point y
    width: int
    height: int


class PointCloudGenerator:
    """
    Generates 3D point clouds from depth images.
    """
    
    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        min_depth: float = 0.1,
        max_depth: float = 4.0,
        decimation: int = 1
    ):
        """
        Initialize point cloud generator.
        
        Args:
            intrinsics:  Camera intrinsic parameters
            min_depth:  Minimum valid depth in meters
            max_depth: Maximum valid depth in meters
            decimation: Decimation factor (1 = full res, 2 = half, etc.)
        """
        self.intrinsics = intrinsics
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.decimation = max(1, decimation)
        
        # Precompute pixel coordinates for efficiency
        self._precompute_coordinates()
    
    def _precompute_coordinates(self):
        """Precompute normalized pixel coordinates"""
        width = self.intrinsics.width // self.decimation
        height = self.intrinsics.height // self.decimation
        
        # Create meshgrid of pixel coordinates
        u = np.arange(0, self.intrinsics.width, self.decimation)
        v = np.arange(0, self.intrinsics.height, self.decimation)
        u, v = np.meshgrid(u, v)
        
        # Compute normalized coordinates
        # (u - cx) / fx and (v - cy) / fy
        self.x_factor = (u. astype(np.float32) - self.intrinsics.cx) / self.intrinsics.fx
        self.y_factor = (v.astype(np.float32) - self.intrinsics.cy) / self.intrinsics.fy
    
    def generate(
        self,
        depth_image: np.ndarray,
        depth_scale: float = 0.001,
        confidence_image: Optional[np.ndarray] = None,
        confidence_threshold: int = 30,
        intensity_image: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]: 
        """
        Generate point cloud from depth image.
        
        Args:
            depth_image:  Depth image in millimeters (uint16) or meters (float32)
            depth_scale: Scale factor to convert depth to meters
            confidence_image:  Optional confidence image for filtering
            confidence_threshold:  Minimum confidence value
            intensity_image: Optional intensity/amplitude image
        
        Returns:
            Tuple of (points_xyz, valid_mask, intensities)
            - points_xyz:  Nx3 array of 3D points
            - valid_mask: Boolean mask of valid points
            - intensities:  Optional Nx1 array of intensities
        """
        # Apply decimation if needed
        if self.decimation > 1:
            depth = depth_image[::self.decimation, ::self.decimation]
            if confidence_image is not None:
                confidence_image = confidence_image[::self.decimation, ::self.decimation]
            if intensity_image is not None: 
                intensity_image = intensity_image[::self.decimation, ::self.decimation]
        else:
            depth = depth_image
        
        # Convert to meters
        depth_m = depth. astype(np.float32) * depth_scale
        
        # Create validity mask
        valid = (depth_m >= self.min_depth) & (depth_m <= self.max_depth)
        
        # Apply confidence filter if available
        if confidence_image is not None:
            valid &= (confidence_image >= confidence_threshold)
        
        # Filter out invalid depths
        valid &= ~np.isnan(depth_m) & ~np.isinf(depth_m)
        
        # Compute 3D coordinates
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        x = self.x_factor * depth_m
        y = self.y_factor * depth_m
        z = depth_m
        
        # Stack into Nx3 array
        points = np.stack([x, y, z], axis=-1)
        
        # Get intensities if available
        intensities = None
        if intensity_image is not None:
            intensities = intensity_image. astype(np.float32)
        
        return points, valid, intensities
    
    def generate_organized(
        self,
        depth_image: np.ndarray,
        depth_scale: float = 0.001,
        confidence_image: Optional[np.ndarray] = None,
        confidence_threshold: int = 30
    ) -> np.ndarray:
        """
        Generate organized point cloud (maintains image structure).
        
        Invalid points are set to NaN. 
        
        Args:
            depth_image: Depth image
            depth_scale: Scale factor
            confidence_image: Optional confidence image
            confidence_threshold: Minimum confidence
        
        Returns: 
            HxWx3 array of 3D points
        """
        points, valid, _ = self.generate(
            depth_image, depth_scale, confidence_image, confidence_threshold
        )
        
        # Set invalid points to NaN
        points[~valid] = np.nan
        
        return points
    
    def to_pointcloud2(
        self,
        depth_image: np.ndarray,
        depth_scale: float,
        frame_id: str,
        stamp,
        confidence_image: Optional[np.ndarray] = None,
        confidence_threshold: int = 30,
        intensity_image: Optional[np.ndarray] = None,
        include_intensity: bool = True
    ) -> Optional['PointCloud2']:
        """
        Generate ROS PointCloud2 message.
        
        Args:
            depth_image: Depth image
            depth_scale: Scale factor
            frame_id: TF frame ID
            stamp: ROS timestamp
            confidence_image: Optional confidence image
            confidence_threshold:  Minimum confidence
            intensity_image:  Optional intensity image
            include_intensity: Whether to include intensity field
        
        Returns:
            PointCloud2 message
        """
        if not ROS_AVAILABLE: 
            return None
        
        # Generate point cloud
        points, valid, intensities = self.generate(
            depth_image, depth_scale,
            confidence_image, confidence_threshold,
            intensity_image if include_intensity else None
        )
        
        # Flatten and filter valid points
        points_flat = points.reshape(-1, 3)
        valid_flat = valid.flatten()
        valid_points = points_flat[valid_flat]
        
        if len(valid_points) == 0:
            return None
        
        # Create point cloud message
        if include_intensity and intensities is not None:
            intensities_flat = intensities.flatten()[valid_flat]
            
            # Create structured array with intensity
            cloud_data = np.zeros(len(valid_points), dtype=[
                ('x', np. float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32)
            ])
            cloud_data['x'] = valid_points[:, 0]
            cloud_data['y'] = valid_points[:, 1]
            cloud_data['z'] = valid_points[: , 2]
            cloud_data['intensity'] = intensities_flat
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField. FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 16
        else:
            cloud_data = np.zeros(len(valid_points), dtype=[
                ('x', np.float32),
                ('y', np. float32),
                ('z', np.float32)
            ])
            cloud_data['x'] = valid_points[:, 0]
            cloud_data['y'] = valid_points[:, 1]
            cloud_data['z'] = valid_points[:, 2]
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField. FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 12
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        
        msg.height = 1
        msg.width = len(valid_points)
        msg.fields = fields
        msg. is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(valid_points)
        msg.data = cloud_data. tobytes()
        msg.is_dense = True
        
        return msg


def create_intrinsics_from_params(
    fx: float, fy: float,
    cx: float, cy:  float,
    width: int, height: int
) -> CameraIntrinsics:
    """
    Create CameraIntrinsics from individual parameters.
    
    Args:
        fx, fy: Focal lengths
        cx, cy: Principal point
        width, height: Image dimensions
    
    Returns:
        CameraIntrinsics object
    """
    return CameraIntrinsics(
        fx=fx, fy=fy,
        cx=cx, cy=cy,
        width=width, height=height
    )