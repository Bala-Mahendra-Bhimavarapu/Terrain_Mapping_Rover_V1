"""
TMR Camera Package

IMX500 (Raspberry Pi AI Camera) driver for Terrain Mapping Rover. 

Features:
- High-resolution image capture (up to 12.3MP)
- Configurable frame rate and resolution
- Auto/manual exposure and white balance
- Camera calibration support
- On-chip AI inference (when model loaded)
"""

from .imx500_driver import IMX500Driver
from .camera_info_manager import CameraInfoManager
from .image_processor import ImageProcessor

__all__ = ['IMX500Driver', 'CameraInfoManager', 'ImageProcessor']
__version__ = '1.0.0'
