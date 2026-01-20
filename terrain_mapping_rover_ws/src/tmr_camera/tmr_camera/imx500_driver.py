"""
IMX500 Camera Driver

Low-level driver for the Raspberry Pi AI Camera (IMX500) using libcamera/Picamera2.

The IMX500 is a 12.3MP stacked CMOS image sensor with integrated AI processor
capable of running neural network models directly on the sensor.

Hardware:
- Sensor: Sony IMX500
- Resolution: 4056 x 3040 (12.3MP)
- Interface:  MIPI CSI-2
- AI Processor: Integrated edge AI accelerator

Requirements:
- Raspberry Pi 5 (or Pi 4 with reduced performance)
- libcamera installed
- picamera2 Python library
"""

import time
import threading
import numpy as np
from typing import Optional, Tuple, Callable, Dict, Any
from dataclasses import dataclass
from enum import Enum

# Try to import picamera2
try:
    from picamera2 import Picamera2
    from libcamera import controls, Transform
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False
    print("WARNING: picamera2 not available.  Using mock camera.")

# Try to import OpenCV for fallback
try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError: 
    OPENCV_AVAILABLE = False


class ExposureMode(Enum):
    AUTO = "auto"
    MANUAL = "manual"


class AWBMode(Enum):
    AUTO = "auto"
    MANUAL = "manual"
    DAYLIGHT = "daylight"
    CLOUDY = "cloudy"
    TUNGSTEN = "tungsten"
    FLUORESCENT = "fluorescent"


class DenoiseMode(Enum):
    OFF = "off"
    FAST = "fast"
    HIGH_QUALITY = "high_quality"


@dataclass
class CameraConfig:
    """Camera configuration parameters"""
    width: int = 1280
    height: int = 720
    frame_rate: float = 30.0
    encoding: str = "bgr8"
    
    # Exposure
    exposure_mode: ExposureMode = ExposureMode.AUTO
    exposure_time_us:  int = 20000
    exposure_compensation: float = 0.0
    
    # Gain
    gain_mode: ExposureMode = ExposureMode.AUTO
    analog_gain: float = 1.0
    
    # White balance
    awb_mode: AWBMode = AWBMode.AUTO
    awb_gain_red: float = 1.5
    awb_gain_blue: float = 1.5
    
    # Image quality
    sharpness:  float = 0.0
    contrast: float = 1.0
    brightness: float = 0.0
    saturation: float = 1.0
    denoise_mode: DenoiseMode = DenoiseMode. FAST
    
    # Hardware
    buffer_count: int = 4
    use_hardware_acceleration: bool = True


@dataclass
class CameraFrame:
    """Container for captured frame data"""
    image: np. ndarray
    timestamp: float
    sequence:  int
    exposure_time_us: int
    analog_gain: float
    metadata: Dict[str, Any]


class IMX500Driver:
    """
    Driver for IMX500 camera using Picamera2/libcamera. 
    
    Provides interface for: 
    - Image capture with configurable resolution and frame rate
    - Exposure and gain control (auto/manual)
    - White balance control
    - Image quality adjustments
    - Continuous capture with callback
    """
    
    # Supported resolutions
    SUPPORTED_RESOLUTIONS = [
        (4056, 3040),  # Full resolution
        (2028, 1520),  # Half resolution
        (1920, 1080),  # 1080p
        (1280, 720),   # 720p
        (640, 480),    # VGA
    ]
    
    def __init__(self, camera_index: int = 0, config: Optional[CameraConfig] = None):
        """
        Initialize IMX500 driver.
        
        Args:
            camera_index: Camera index (0 for first camera)
            config: Camera configuration (uses defaults if None)
        """
        self.camera_index = camera_index
        self.config = config or CameraConfig()
        
        self.camera: Optional[Picamera2] = None
        self.connected = False
        self.capturing = False
        
        # Capture state
        self.frame_count = 0
        self.last_frame_time = 0.0
        self.actual_frame_rate = 0.0
        
        # Continuous capture
        self.capture_thread:  Optional[threading.Thread] = None
        self.capture_callback: Optional[Callable[[CameraFrame], None]] = None
        self.stop_capture = threading.Event()
        
        # Lock for thread safety
        self.lock = threading.Lock()
    
    def connect(self) -> bool:
        """
        Connect to the camera and initialize. 
        
        Returns:
            True if connection successful
        """
        if not PICAMERA2_AVAILABLE: 
            print("WARNING: picamera2 not available, using mock mode")
            self.connected = True
            return True
        
        try:
            # Create Picamera2 instance
            self.camera = Picamera2(self.camera_index)
            
            # Create configuration
            camera_config = self.camera.create_still_configuration(
                main={
                    "size": (self.config.width, self. config.height),
                    "format": "RGB888" if self.config.encoding in ["rgb8", "bgr8"] else "YUV420"
                },
                buffer_count=self.config.buffer_count
            )
            
            # Configure camera
            self.camera.configure(camera_config)
            
            # Apply settings
            self._apply_controls()
            
            self.connected = True
            return True
            
        except Exception as e:
            print(f"Failed to connect to camera: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from camera"""
        self.stop_continuous_capture()
        
        if self.camera is not None:
            try:
                self.camera.stop()
                self.camera.close()
            except Exception: 
                pass
            self.camera = None
        
        self.connected = False
    
    def _apply_controls(self):
        """Apply camera controls from config"""
        if self.camera is None:
            return
        
        controls_dict = {}
        
        # Exposure
        if self.config.exposure_mode == ExposureMode.AUTO: 
            controls_dict["AeEnable"] = True
            controls_dict["ExposureValue"] = self.config.exposure_compensation
        else:
            controls_dict["AeEnable"] = False
            controls_dict["ExposureTime"] = self.config.exposure_time_us
            controls_dict["AnalogueGain"] = self.config.analog_gain
        
        # White balance
        awb_mode_map = {
            AWBMode. AUTO: controls. AwbModeEnum.Auto,
            AWBMode. DAYLIGHT: controls.AwbModeEnum.Daylight,
            AWBMode.CLOUDY: controls.AwbModeEnum.Cloudy,
            AWBMode.TUNGSTEN:  controls.AwbModeEnum.Tungsten,
            AWBMode.FLUORESCENT: controls.AwbModeEnum. Fluorescent,
        }
        
        if self.config. awb_mode == AWBMode. MANUAL:
            controls_dict["AwbEnable"] = False
            controls_dict["ColourGains"] = (
                self.config.awb_gain_red,
                self.config. awb_gain_blue
            )
        elif self.config.awb_mode in awb_mode_map: 
            controls_dict["AwbEnable"] = True
            controls_dict["AwbMode"] = awb_mode_map[self.config.awb_mode]
        
        # Image quality
        controls_dict["Sharpness"] = self.config.sharpness + 1.0  # libcamera uses 0-2 range
        controls_dict["Contrast"] = self.config. contrast
        controls_dict["Brightness"] = self.config.brightness
        controls_dict["Saturation"] = self.config.saturation
        
        # Denoise
        denoise_map = {
            DenoiseMode.OFF: controls.draft.NoiseReductionModeEnum. Off,
            DenoiseMode.FAST: controls.draft. NoiseReductionModeEnum.Fast,
            DenoiseMode.HIGH_QUALITY: controls.draft.NoiseReductionModeEnum.HighQuality,
        }
        if self.config.denoise_mode in denoise_map:
            controls_dict["NoiseReductionMode"] = denoise_map[self.config. denoise_mode]
        
        # Frame rate
        frame_duration = int(1_000_000 / self.config.frame_rate)  # microseconds
        controls_dict["FrameDurationLimits"] = (frame_duration, frame_duration)
        
        try:
            self.camera.set_controls(controls_dict)
        except Exception as e:
            print(f"Warning: Failed to set some controls: {e}")
    
    def start(self) -> bool:
        """
        Start camera streaming.
        
        Returns:
            True if started successfully
        """
        if not self.connected:
            return False
        
        if not PICAMERA2_AVAILABLE: 
            self.capturing = True
            return True
        
        try:
            self.camera.start()
            self.capturing = True
            self.frame_count = 0
            self.last_frame_time = time.time()
            return True
        except Exception as e: 
            print(f"Failed to start camera: {e}")
            return False
    
    def stop(self):
        """Stop camera streaming"""
        if self.camera is not None and PICAMERA2_AVAILABLE: 
            try:
                self.camera.stop()
            except Exception:
                pass
        self.capturing = False
    
    def capture(self) -> Optional[CameraFrame]:
        """
        Capture a single frame.
        
        Returns:
            CameraFrame with image data, or None on failure
        """
        if not self.connected or not self.capturing:
            return None
        
        try:
            with self.lock:
                if not PICAMERA2_AVAILABLE: 
                    # Return mock frame
                    return self._create_mock_frame()
                
                # Capture frame with metadata
                request = self.camera.capture_request()
                
                # Get image data
                image = request.make_array("main")
                
                # Convert RGB to BGR if needed
                if self.config. encoding == "bgr8" and len(image. shape) == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                # Get metadata
                metadata = request.get_metadata()
                
                # Create frame object
                frame = CameraFrame(
                    image=image,
                    timestamp=time.time(),
                    sequence=self.frame_count,
                    exposure_time_us=metadata.get("ExposureTime", 0),
                    analog_gain=metadata. get("AnalogueGain", 1.0),
                    metadata=metadata
                )
                
                # Release request
                request.release()
                
                # Update stats
                self.frame_count += 1
                current_time = time.time()
                if self.last_frame_time > 0:
                    dt = current_time - self.last_frame_time
                    if dt > 0:
                        self. actual_frame_rate = 0.9 * self.actual_frame_rate + 0.1 * (1. 0 / dt)
                self.last_frame_time = current_time
                
                return frame
                
        except Exception as e:
            print(f"Capture failed: {e}")
            return None
    
    def _create_mock_frame(self) -> CameraFrame:
        """Create a mock frame for testing without hardware"""
        # Create gradient test pattern
        image = np.zeros((self.config.height, self.config.width, 3), dtype=np.uint8)
        
        # Add color gradient
        for i in range(self.config.height):
            image[i, :, 0] = int(255 * i / self.config.height)  # Blue gradient
            image[i, :, 2] = int(255 * (1 - i / self.config. height))  # Red gradient
        
        # Add timestamp text
        if OPENCV_AVAILABLE:
            timestamp_str = f"MOCK {time.strftime('%H:%M:%S')}"
            cv2.putText(image, timestamp_str, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(image, f"Frame:  {self.frame_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        self.frame_count += 1
        
        return CameraFrame(
            image=image,
            timestamp=time.time(),
            sequence=self.frame_count,
            exposure_time_us=20000,
            analog_gain=1.0,
            metadata={"mock": True}
        )
    
    def start_continuous_capture(self, callback: Callable[[CameraFrame], None]):
        """
        Start continuous capture with callback.
        
        Args:
            callback: Function called for each captured frame
        """
        if self.capture_thread is not None and self.capture_thread.is_alive():
            return
        
        self.capture_callback = callback
        self.stop_capture.clear()
        
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
    
    def stop_continuous_capture(self):
        """Stop continuous capture"""
        self.stop_capture.set()
        
        if self.capture_thread is not None: 
            self.capture_thread. join(timeout=2.0)
            self.capture_thread = None
    
    def _capture_loop(self):
        """Internal capture loop for continuous capture"""
        frame_interval = 1.0 / self.config.frame_rate
        
        while not self.stop_capture.is_set():
            start_time = time.time()
            
            frame = self.capture()
            
            if frame is not None and self.capture_callback is not None:
                try:
                    self.capture_callback(frame)
                except Exception as e:
                    print(f"Capture callback error: {e}")
            
            # Maintain frame rate
            elapsed = time.time() - start_time
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def set_exposure(self, mode: ExposureMode, exposure_us: int = 20000, gain:  float = 1.0):
        """
        Set exposure mode and parameters.
        
        Args:
            mode: AUTO or MANUAL
            exposure_us:  Exposure time in microseconds (for manual mode)
            gain: Analog gain (for manual mode)
        """
        self.config.exposure_mode = mode
        self.config.exposure_time_us = exposure_us
        self.config.analog_gain = gain
        
        if self.connected and self.camera is not None:
            self._apply_controls()
    
    def set_white_balance(self, mode: AWBMode, red_gain: float = 1.5, blue_gain: float = 1.5):
        """
        Set white balance mode and parameters.
        
        Args:
            mode: AWB mode
            red_gain: Red channel gain (for manual mode)
            blue_gain: Blue channel gain (for manual mode)
        """
        self. config.awb_mode = mode
        self.config.awb_gain_red = red_gain
        self.config.awb_gain_blue = blue_gain
        
        if self.connected and self.camera is not None:
            self._apply_controls()
    
    def get_frame_rate(self) -> float:
        """Get actual measured frame rate"""
        return self.actual_frame_rate
    
    def get_frame_count(self) -> int:
        """Get total frames captured"""
        return self.frame_count
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self. stop()
        self.disconnect()
        return False
