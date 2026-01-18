"""
Arducam ToF Camera Driver (B0410).

This driver interfaces with the Arducam ToF camera using the official SDK.
The camera provides depth images up to 3 meters. 

Author: Rover Team
"""

import time
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class ToFConfig:
    """ToF camera configuration."""
    # Range mode
    max_range_m: float = 3.0      # Maximum range in meters
    
    # Frame rate
    framerate: int = 15           # Target frame rate
    
    # Confidence threshold
    confidence_threshold: int = 30  # Minimum confidence (0-255)


@dataclass
class ToFIntrinsics:
    """ToF camera intrinsic parameters."""
    width: int = 240
    height: int = 180
    
    # Intrinsic parameters
    fx: float = 240.0   # Focal length X
    fy: float = 180.0   # Focal length Y
    cx: float = 120.0   # Principal point X
    cy: float = 90.0    # Principal point Y
    
    # Depth scale (depth_value * scale = meters)
    depth_scale: float = 0.001  # mm to meters


@dataclass
class ToFFrame:
    """Captured ToF frame data."""
    depth_image: np.ndarray       # Depth image (uint16, mm)
    confidence_image: np.ndarray  # Confidence image (uint8)
    amplitude_image: np.ndarray   # Amplitude/IR image (uint16)
    timestamp: float              # Capture timestamp


class ArducamToFDriver: 
    """
    Driver for Arducam ToF Camera B0410.
    
    Uses the Arducam ToF SDK for camera access.
    The camera provides: 
    - Depth image (distance in mm)
    - Confidence image (reliability of depth measurement)
    - Amplitude/IR image (infrared reflectance)
    """
    
    def __init__(self, config: Optional[ToFConfig] = None):
        """
        Initialize ToF driver.
        
        Args:
            config: Camera configuration.  Uses defaults if None.
        """
        self.config = config or ToFConfig()
        self._camera = None
        self._initialized = False
        self. intrinsics = ToFIntrinsics()
    
    def initialize(self) -> bool:
        """
        Initialize the ToF camera.
        
        Returns:
            True if initialization successful. 
        """
        try:
            # Import Arducam ToF SDK
            # The SDK should be installed from: 
            # https://github.com/ArduCAM/Arducam_tof_camera
            import ArducamDepthCamera as ac
            
            self._camera = ac.ArducamCamera()
            
            # Open camera
            ret = self._camera.open(ac.TOFConnect. CSI, 0)
            if ret != 0:
                print(f"Failed to open ToF camera: error {ret}")
                return False
            
            # Start camera
            ret = self._camera. start(ac.TOFOutput.DEPTH)
            if ret != 0:
                print(f"Failed to start ToF camera: error {ret}")
                self._camera.close()
                return False
            
            # Get camera info
            info = self._camera.getCameraInfo()
            self.intrinsics.width = info.width
            self.intrinsics.height = info.height
            
            # Set range mode
            if self.config.max_range_m <= 2.0:
                self._camera.setControl(ac.TOFControl. RANG, 0)  # Short range
            else:
                self._camera.setControl(ac. TOFControl.RANG, 1)  # Long range
            
            self._initialized = True
            print(f"ToF camera initialized:  {self.intrinsics.width}x{self.intrinsics.height}")
            return True
            
        except ImportError:
            print("Arducam ToF SDK not found.")
            print("Install from:  https://github.com/ArduCAM/Arducam_tof_camera")
            return False
        except Exception as e: 
            print(f"Failed to initialize ToF camera: {e}")
            return False
    
    def capture(self) -> Optional[ToFFrame]:
        """
        Capture a ToF frame.
        
        Returns:
            ToFFrame containing depth, confidence, and amplitude data.
        """
        if not self._initialized or not self._camera:
            return None
        
        try: 
            import ArducamDepthCamera as ac
            
            # Request frame
            frame = self._camera.requestFrame(200)  # 200ms timeout
            
            if frame is None:
                return None
            
            # Get depth data
            depth_buf = frame.getDepthData()
            confidence_buf = frame.getConfidenceData()
            amplitude_buf = frame.getAmplitudeData()
            
            # Convert to numpy arrays
            depth_image = np.array(depth_buf, dtype=np.uint16).reshape(
                self.intrinsics.height, self.intrinsics.width)
            
            confidence_image = np.array(confidence_buf, dtype=np.uint8).reshape(
                self.intrinsics.height, self.intrinsics.width)
            
            amplitude_image = np.array(amplitude_buf, dtype=np. uint16).reshape(
                self.intrinsics.height, self.intrinsics.width)
            
            # Release frame
            self._camera.releaseFrame(frame)
            
            # Apply confidence threshold
            mask = confidence_image < self.config.confidence_threshold
            depth_image[mask] = 0
            
            return ToFFrame(
                depth_image=depth_image,
                confidence_image=confidence_image,
                amplitude_image=amplitude_image,
                timestamp=time. time()
            )
            
        except Exception as e:
            print(f"ToF capture failed: {e}")
            return None
    
    def set_intrinsics(
        self,
        fx: float,
        fy:  float,
        cx: float,
        cy: float,
        depth_scale: float = 0.001
    ):
        """
        Set camera intrinsics manually.
        
        Args:
            fx, fy:  Focal lengths in pixels
            cx, cy:  Principal point in pixels
            depth_scale: Scale factor to convert depth values to meters
        """
        self.intrinsics.fx = fx
        self.intrinsics. fy = fy
        self.intrinsics.cx = cx
        self.intrinsics. cy = cy
        self.intrinsics.depth_scale = depth_scale
    
    def close(self):
        """Close the camera."""
        if self._camera:
            try:
                self._camera.stop()
                self._camera.close()
            except:
                pass
        self._camera = None
        self._initialized = False
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized