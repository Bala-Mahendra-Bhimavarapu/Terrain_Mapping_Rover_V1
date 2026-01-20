"""
TMR ToF Camera Package

Arducam ToF Camera (B0410) driver for Terrain Mapping Rover. 

Features:
- Depth image capture (240x180 resolution)
- Confidence and amplitude images
- Point cloud generation
- Depth filtering and processing
- Multiple depth modes (near, middle, far)

Hardware: 
- Sensor: Sony IMX316
- Resolution: 240 x 180
- Depth Range: 0.2m - 4m
- Interface:  MIPI CSI-2
"""

from .arducam_tof_driver import ArducamToFDriver
from .pointcloud_generator import PointCloudGenerator
from .depth_processor import DepthProcessor

__all__ = ['ArducamToFDriver', 'PointCloudGenerator', 'DepthProcessor']
__version__ = '1.0.0'
