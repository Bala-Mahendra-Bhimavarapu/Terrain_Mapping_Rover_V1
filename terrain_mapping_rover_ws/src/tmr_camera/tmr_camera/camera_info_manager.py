"""
Camera Info Manager

Manages camera intrinsic calibration data and provides CameraInfo messages. 

Supports loading calibration from: 
- YAML files (OpenCV/ROS format)
- Parameters
- Default values
"""

import os
import yaml
import numpy as np
from typing import Optional, List, Tuple
from dataclasses import dataclass

from sensor_msgs.msg import CameraInfo


@dataclass
class CalibrationData:
    """Container for camera calibration data"""
    width: int
    height: int
    camera_name: str
    
    # Intrinsic matrix (3x3)
    camera_matrix: np.ndarray
    
    # Distortion coefficients
    distortion_model: str
    distortion_coefficients: np.ndarray
    
    # Rectification matrix (3x3) - for stereo
    rectification_matrix: np. ndarray
    
    # Projection matrix (3x4)
    projection_matrix: np.ndarray


class CameraInfoManager:
    """
    Manages camera calibration info and generates CameraInfo messages.
    """
    
    def __init__(
        self,
        camera_name: str = "camera",
        frame_id: str = "camera_optical_frame",
        width: int = 1280,
        height: int = 720
    ):
        """
        Initialize camera info manager.
        
        Args:
            camera_name:  Camera identifier
            frame_id: TF frame ID
            width: Image width
            height: Image height
        """
        self.camera_name = camera_name
        self. frame_id = frame_id
        self.width = width
        self.height = height
        
        # Default calibration (approximate values)
        self.calibration = self._create_default_calibration()
        
        self.calibrated = False
    
    def _create_default_calibration(self) -> CalibrationData:
        """Create default calibration with approximate values"""
        # Approximate focal length based on typical IMX500 lens
        # f ≈ width for ~60° horizontal FOV
        fx = float(self.width) * 0.7
        fy = fx
        cx = float(self.width) / 2.0
        cy = float(self.height) / 2.0
        
        camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ])
        
        distortion = np.zeros(5)
        
        rectification = np.eye(3)
        
        projection = np.array([
            [fx, 0. 0, cx, 0.0],
            [0.0, fy, cy, 0.0],
            [0.0, 0.0, 1.0, 0.0]
        ])
        
        return CalibrationData(
            width=self.width,
            height=self.height,
            camera_name=self.camera_name,
            camera_matrix=camera_matrix,
            distortion_model="plumb_bob",
            distortion_coefficients=distortion,
            rectification_matrix=rectification,
            projection_matrix=projection
        )
    
    def load_calibration_file(self, filepath: str) -> bool:
        """
        Load calibration from YAML file.
        
        Args:
            filepath: Path to calibration YAML file
        
        Returns:
            True if loaded successfully
        """
        if not os.path.exists(filepath):
            print(f"Calibration file not found: {filepath}")
            return False
        
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            # Parse calibration data
            width = data.get('image_width', self.width)
            height = data.get('image_height', self.height)
            camera_name = data.get('camera_name', self.camera_name)
            
            # Camera matrix
            cm_data = data.get('camera_matrix', {})
            if 'data' in cm_data:
                camera_matrix = np.array(cm_data['data']).reshape(3, 3)
            else:
                camera_matrix = self. calibration.camera_matrix
            
            # Distortion
            dist_model = data.get('distortion_model', 'plumb_bob')
            dist_data = data.get('distortion_coefficients', {})
            if 'data' in dist_data:
                distortion = np. array(dist_data['data'])
            else:
                distortion = self.calibration.distortion_coefficients
            
            # Rectification
            rect_data = data.get('rectification_matrix', {})
            if 'data' in rect_data: 
                rectification = np.array(rect_data['data']).reshape(3, 3)
            else:
                rectification = np.eye(3)
            
            # Projection
            proj_data = data.get('projection_matrix', {})
            if 'data' in proj_data: 
                projection = np.array(proj_data['data']).reshape(3, 4)
            else:
                projection = np. hstack([camera_matrix, np.zeros((3, 1))])
            
            self.calibration = CalibrationData(
                width=width,
                height=height,
                camera_name=camera_name,
                camera_matrix=camera_matrix,
                distortion_model=dist_model,
                distortion_coefficients=distortion,
                rectification_matrix=rectification,
                projection_matrix=projection
            )
            
            self.calibrated = True
            print(f"Loaded calibration from:  {filepath}")
            return True
            
        except Exception as e: 
            print(f"Failed to load calibration: {e}")
            return False
    
    def set_calibration_from_params(
        self,
        camera_matrix: List[float],
        distortion_coefficients: List[float],
        distortion_model: str = "plumb_bob"
    ):
        """
        Set calibration from parameter values.
        
        Args:
            camera_matrix: 9-element list [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            distortion_coefficients: 5-element list [k1, k2, p1, p2, k3]
            distortion_model: Distortion model name
        """
        if len(camera_matrix) == 9:
            cm = np.array(camera_matrix).reshape(3, 3)
        else:
            cm = self.calibration.camera_matrix
        
        if len(distortion_coefficients) >= 4:
            dist = np.array(distortion_coefficients)
        else:
            dist = self.calibration.distortion_coefficients
        
        # Create projection matrix from camera matrix
        proj = np.hstack([cm, np.zeros((3, 1))])
        
        self.calibration = CalibrationData(
            width=self.width,
            height=self.height,
            camera_name=self.camera_name,
            camera_matrix=cm,
            distortion_model=distortion_model,
            distortion_coefficients=dist,
            rectification_matrix=np.eye(3),
            projection_matrix=proj
        )
        
        self.calibrated = True
    
    def get_camera_info(self, stamp=None) -> CameraInfo:
        """
        Create CameraInfo message with current calibration.
        
        Args:
            stamp: Timestamp for message header
        
        Returns:
            CameraInfo message
        """
        msg = CameraInfo()
        
        # Header
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        # Image dimensions
        msg.width = self.calibration.width
        msg.height = self.calibration.height
        
        # Distortion model
        msg.distortion_model = self.calibration.distortion_model
        
        # Distortion coefficients (D)
        msg.d = self.calibration.distortion_coefficients. flatten().tolist()
        
        # Intrinsic camera matrix (K)
        msg.k = self.calibration.camera_matrix.flatten().tolist()
        
        # Rectification matrix (R)
        msg.r = self.calibration. rectification_matrix.flatten().tolist()
        
        # Projection matrix (P)
        msg.p = self.calibration. projection_matrix.flatten().tolist()
        
        # Binning (not used)
        msg.binning_x = 0
        msg.binning_y = 0
        
        # ROI (full image)
        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi. height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = False
        
        return msg
    
    def get_intrinsics(self) -> Tuple[float, float, float, float]: 
        """
        Get camera intrinsic parameters.
        
        Returns:
            Tuple of (fx, fy, cx, cy)
        """
        K = self.calibration.camera_matrix
        return (K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    
    def save_calibration(self, filepath:  str):
        """
        Save current calibration to YAML file. 
        
        Args:
            filepath: Output file path
        """
        data = {
            'image_width': int(self.calibration.width),
            'image_height': int(self.calibration.height),
            'camera_name': self.calibration.camera_name,
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': self.calibration.camera_matrix. flatten().tolist()
            },
            'distortion_model':  self.calibration.distortion_model,
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(self.calibration.distortion_coefficients),
                'data': self.calibration.distortion_coefficients.tolist()
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': self.calibration. rectification_matrix.flatten().tolist()
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': self.calibration.projection_matrix.flatten().tolist()
            }
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        print(f"Saved calibration to: {filepath}")
