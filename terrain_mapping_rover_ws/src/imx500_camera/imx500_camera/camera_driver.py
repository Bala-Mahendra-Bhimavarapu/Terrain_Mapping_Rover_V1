"""
IMX500 Camera Driver for Raspberry Pi 5. 

Uses libcamera/picamera2 to interface with the Pi AI Camera. 
Supports both RGB streaming and AI accelerator features. 

Author: Rover Team
"""

import time
import numpy as np
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass, field
from threading import Lock


@dataclass
class CameraConfig:
    """Camera configuration parameters."""
    width: int = 1280
    height: int = 720
    framerate: int = 30
    
    # Exposure control
    ae_enable: bool = True
    exposure_time_us: int = 20000  # 20ms
    analogue_gain: float = 1.0
    digital_gain: float = 1.0
    
    # ROI for metering (normalized 0-1)
    roi_x: float = 0.0
    roi_y: float = 0.0
    roi_width: float = 1.0
    roi_height: float = 1.0
    
    # Limits
    min_exposure_us: int = 100
    max_exposure_us: int = 100000
    min_gain: float = 1.0
    max_gain: float = 16.0


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters."""
    fx: float = 600.0  # Focal length X (pixels)
    fy: float = 600.0  # Focal length Y (pixels)
    cx: float = 640.0  # Principal point X
    cy: float = 360.0  # Principal point Y
    
    # Distortion coefficients (k1, k2, p1, p2, k3)
    distortion:  Tuple[float, ... ] = (0.0, 0.0, 0.0, 0.0, 0.0)
    
    # Image dimensions
    width: int = 1280
    height: int = 720


@dataclass 
class CaptureResult:
    """Result of a camera capture."""
    image: np.ndarray
    timestamp: float
    exposure_time_us: int = 0
    analogue_gain: float = 1.0
    digital_gain: float = 1.0
    brightness: float = 0.0  # Average brightness 0-255


class IMX500Driver:
    """
    Driver for IMX500-based Pi AI Camera.
    
    Uses picamera2 library for camera access on Raspberry Pi 5.
    """
    
    def __init__(self, config: Optional[CameraConfig] = None):
        """
        Initialize the camera driver.
        
        Args:
            config: Camera configuration.  Uses defaults if None.
        """
        self.config = config or CameraConfig()
        self._camera = None
        self._initialized = False
        self._lock = Lock()
        
        # Store last capture metadata
        self._last_metadata:  Dict[str, Any] = {}
    
    def initialize(self) -> bool:
        """
        Initialize the camera.
        
        Returns:
            True if initialization successful. 
        """
        try:
            from picamera2 import Picamera2
            from libcamera import controls
            
            self._camera = Picamera2()
            
            # Configure camera
            camera_config = self._camera.create_still_configuration(
                main={"size": (self.config.width, self.config.height), "format": "RGB888"},
                buffer_count=4
            )
            
            self._camera.configure(camera_config)
            
            # Set initial controls
            self._apply_controls()
            
            # Start camera
            self._camera.start()
            
            # Wait for camera to stabilize
            time.sleep(0.5)
            
            self._initialized = True
            return True
            
        except ImportError as e:
            print(f"Failed to import picamera2: {e}")
            print("Install with: sudo apt install python3-picamera2")
            return False
        except Exception as e:
            print(f"Failed to initialize camera: {e}")
            return False
    
    def _apply_controls(self):
        """Apply current configuration to camera controls."""
        if not self._camera:
            return
        
        try:
            from libcamera import controls
            
            ctrl = {}
            
            if self.config.ae_enable:
                ctrl[controls.AeEnable] = True
            else:
                ctrl[controls.AeEnable] = False
                ctrl[controls.ExposureTime] = self.config.exposure_time_us
                ctrl[controls.AnalogueGain] = self.config. analogue_gain
            
            # Set metering ROI if supported
            # Note: Not all controls may be available on all cameras
            
            self._camera.set_controls(ctrl)
            
        except Exception as e:
            print(f"Warning: Could not apply all controls: {e}")
    
    def capture(self) -> Optional[CaptureResult]:
        """
        Capture a single frame.
        
        Returns:
            CaptureResult containing image and metadata, or None on failure.
        """
        if not self._initialized or not self._camera:
            return None
        
        try: 
            with self._lock:
                # Capture with metadata
                result = self._camera.capture_array("main")
                metadata = self._camera.capture_metadata()
                
                self._last_metadata = metadata
                
                # Extract metadata
                exposure = metadata.get('ExposureTime', 0)
                again = metadata.get('AnalogueGain', 1.0)
                dgain = metadata.get('DigitalGain', 1.0)
                
                # Calculate average brightness
                if result is not None:
                    gray = np.mean(result, axis=2) if len(result.shape) == 3 else result
                    brightness = np.mean(gray)
                else:
                    brightness = 0.0
                
                return CaptureResult(
                    image=result,
                    timestamp=time.time(),
                    exposure_time_us=exposure,
                    analogue_gain=again,
                    digital_gain=dgain,
                    brightness=brightness
                )
                
        except Exception as e:
            print(f"Capture failed: {e}")
            return None
    
    def set_exposure(self, exposure_us: int):
        """Set manual exposure time."""
        self.config.exposure_time_us = max(
            self.config.min_exposure_us,
            min(self.config.max_exposure_us, exposure_us)
        )
        self. config.ae_enable = False
        self._apply_controls()
    
    def set_gain(self, analogue_gain: float, digital_gain: float = 1.0):
        """Set manual gain values."""
        self.config.analogue_gain = max(
            self.config.min_gain,
            min(self.config. max_gain, analogue_gain)
        )
        self.config.digital_gain = digital_gain
        self. config.ae_enable = False
        self._apply_controls()
    
    def set_auto_exposure(self, enable: bool):
        """Enable or disable auto exposure."""
        self. config.ae_enable = enable
        self._apply_controls()
    
    def get_metadata(self) -> Dict[str, Any]:
        """Get last capture metadata."""
        return self._last_metadata. copy()
    
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
        """Check if camera is initialized."""
        return self._initialized
