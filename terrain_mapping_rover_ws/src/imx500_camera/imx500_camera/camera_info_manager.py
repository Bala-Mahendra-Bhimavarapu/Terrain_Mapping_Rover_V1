"""
Camera Info Manager - handles camera intrinsics. 

Supports loading from YAML or direct parameter input.
"""

import yaml
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass


@dataclass
class CameraInfo:
    """Camera calibration information."""
    width: int
    height: int
    
    # Intrinsic matrix elements
    fx: float
    fy: float
    cx: float
    cy: float
    
    # Distortion coefficients [k1, k2, p1, p2, k3]
    distortion_model: str = "plumb_bob"
    D:  Tuple[float, ... ] = (0.0, 0.0, 0.0, 0.0, 0.0)
    
    # Full matrices (computed from above)
    K: np.ndarray = None  # 3x3 intrinsic matrix
    R: np. ndarray = None  # 3x3 rectification matrix (identity for monocular)
    P: np.ndarray = None  # 3x4 projection matrix
    
    def __post_init__(self):
        """Compute full matrices."""
        # Intrinsic matrix K
        self.K = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self. fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np. float64)
        
        # Rectification matrix (identity for monocular)
        self.R = np.eye(3, dtype=np.float64)
        
        # Projection matrix P = K * [I | 0] for monocular
        self.P = np.array([
            [self.fx, 0.0, self. cx, 0.0],
            [0.0, self. fy, self.cy, 0.0],
            [0.0, 0.0, 1.0, 0.0]
        ], dtype=np. float64)


class CameraInfoManager:
    """Manages camera intrinsic parameters."""
    
    def __init__(self):
        self._info: Optional[CameraInfo] = None
    
    def load_from_yaml(self, filepath: str) -> bool:
        """
        Load camera info from YAML file.
        
        Expected format (OpenCV/ROS standard):
        ```yaml
        image_width: 1280
        image_height: 720
        camera_matrix:
          data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        distortion_coefficients:
          data: [k1, k2, p1, p2, k3]
        ```
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml. safe_load(f)
            
            width = data['image_width']
            height = data['image_height']
            
            K_data = data['camera_matrix']['data']
            D_data = data. get('distortion_coefficients', {}).get('data', [0, 0, 0, 0, 0])
            
            self._info = CameraInfo(
                width=width,
                height=height,
                fx=K_data[0],
                fy=K_data[4],
                cx=K_data[2],
                cy=K_data[5],
                D=tuple(D_data[: 5])
            )
            return True
            
        except Exception as e:
            print(f"Failed to load camera info: {e}")
            return False
    
    def set_from_parameters(
        self,
        width: int,
        height: int,
        fx:  float,
        fy: float,
        cx: float,
        cy: float,
        distortion:  Optional[List[float]] = None
    ):
        """
        Set camera info directly from parameters.
        
        This allows manual input without a YAML file.
        """
        D = tuple(distortion) if distortion else (0.0, 0.0, 0.0, 0.0, 0.0)
        
        self._info = CameraInfo(
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            D=D
        )
    
    def get_info(self) -> Optional[CameraInfo]:
        """Get current camera info."""
        return self._info
    
    def to_ros_msg(self):
        """
        Convert to sensor_msgs/CameraInfo message.
        
        Returns:
            sensor_msgs. msg.CameraInfo
        """
        from sensor_msgs.msg import CameraInfo as CameraInfoMsg
        
        if self._info is None:
            return None
        
        msg = CameraInfoMsg()
        msg.width = self._info. width
        msg.height = self._info.height
        msg. distortion_model = self._info. distortion_model
        msg. d = list(self._info.D)
        msg.k = self._info.K. flatten().tolist()
        msg.r = self._info.R.flatten().tolist()
        msg.p = self._info.P.flatten().tolist()
        
        return msg
