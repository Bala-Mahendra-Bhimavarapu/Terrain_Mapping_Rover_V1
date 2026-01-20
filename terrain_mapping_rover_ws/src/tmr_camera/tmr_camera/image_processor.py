"""
Image Processor

Provides image processing utilities for the camera node: 
- Image format conversion
- Undistortion
- Compression
- Basic enhancements
"""

import numpy as np
from typing import Optional, Tuple

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False
    print("WARNING: OpenCV not available.  Image processing disabled.")


class ImageProcessor:
    """
    Image processing utilities for camera images.
    """
    
    def __init__(self):
        """Initialize image processor"""
        self.undistort_map1:  Optional[np.ndarray] = None
        self.undistort_map2: Optional[np. ndarray] = None
        self.camera_matrix: Optional[np.ndarray] = None
        self. distortion_coeffs: Optional[np.ndarray] = None
    
    def setup_undistortion(
        self,
        camera_matrix: np.ndarray,
        distortion_coeffs:  np.ndarray,
        width: int,
        height: int,
        alpha: float = 0.0
    ):
        """
        Setup undistortion maps for fast image correction.
        
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            distortion_coeffs:  Distortion coefficients
            width:  Image width
            height: Image height
            alpha: Free scaling parameter (0=crop, 1=keep all)
        """
        if not OPENCV_AVAILABLE:
            return
        
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        
        # Get optimal new camera matrix
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, distortion_coeffs,
            (width, height), alpha, (width, height)
        )
        
        # Create undistortion maps
        self. undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
            camera_matrix, distortion_coeffs, None,
            new_camera_matrix, (width, height), cv2.CV_32FC1
        )
    
    def undistort(self, image: np.ndarray) -> np.ndarray:
        """
        Undistort an image using precomputed maps.
        
        Args:
            image: Input distorted image
        
        Returns:
            Undistorted image
        """
        if not OPENCV_AVAILABLE:
            return image
        
        if self.undistort_map1 is None or self.undistort_map2 is None: 
            return image
        
        return cv2.remap(image, self.undistort_map1, self.undistort_map2,
                        cv2.INTER_LINEAR)
    
    @staticmethod
    def convert_encoding(image: np.ndarray, from_encoding: str, to_encoding: str) -> np.ndarray:
        """
        Convert image between color encodings.
        
        Args:
            image: Input image
            from_encoding: Source encoding (bgr8, rgb8, mono8)
            to_encoding: Target encoding
        
        Returns:
            Converted image
        """
        if not OPENCV_AVAILABLE: 
            return image
        
        if from_encoding == to_encoding:
            return image
        
        # Handle conversions
        if from_encoding == "bgr8" and to_encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif from_encoding == "rgb8" and to_encoding == "bgr8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif from_encoding in ["bgr8", "rgb8"] and to_encoding == "mono8": 
            if from_encoding == "bgr8":
                return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        elif from_encoding == "mono8" and to_encoding == "bgr8":
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif from_encoding == "mono8" and to_encoding == "rgb8": 
            return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        
        return image
    
    @staticmethod
    def compress_jpeg(image: np.ndarray, quality: int = 80) -> bytes:
        """
        Compress image to JPEG. 
        
        Args:
            image: Input image (BGR format)
            quality: JPEG quality (0-100)
        
        Returns: 
            Compressed JPEG bytes
        """
        if not OPENCV_AVAILABLE:
            return b''
        
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
        success, encoded = cv2.imencode('.jpg', image, encode_params)
        
        if success:
            return encoded.tobytes()
        return b''
    
    @staticmethod
    def resize(image: np.ndarray, width: int, height: int) -> np.ndarray:
        """
        Resize image. 
        
        Args:
            image: Input image
            width:  Target width
            height: Target height
        
        Returns:
            Resized image
        """
        if not OPENCV_AVAILABLE: 
            return image
        
        return cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
    
    @staticmethod
    def apply_clahe(image: np.ndarray, clip_limit: float = 2.0, grid_size: int = 8) -> np.ndarray:
        """
        Apply CLAHE (Contrast Limited Adaptive Histogram Equalization).
        
        Improves local contrast, useful for varying lighting conditions.
        
        Args:
            image: Input image (BGR or grayscale)
            clip_limit: Threshold for contrast limiting
            grid_size: Size of grid for histogram equalization
        
        Returns: 
            Enhanced image
        """
        if not OPENCV_AVAILABLE:
            return image
        
        # Convert to LAB color space if color image
        if len(image.shape) == 3:
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
        else:
            l = image
        
        # Apply CLAHE to L channel
        clahe = cv2.createCLAHE(clipLimit=clip_limit, 
                                tileGridSize=(grid_size, grid_size))
        l_enhanced = clahe.apply(l)
        
        # Merge back
        if len(image.shape) == 3:
            lab_enhanced = cv2.merge([l_enhanced, a, b])
            return cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)
        else:
            return l_enhanced
    
    @staticmethod
    def adjust_brightness_contrast(
        image: np.ndarray,
        brightness: float = 0.0,
        contrast: float = 1.0
    ) -> np.ndarray:
        """
        Adjust image brightness and contrast.
        
        Args:
            image: Input image
            brightness: Brightness adjustment (-1.0 to 1.0)
            contrast: Contrast multiplier (0.0 to 2.0)
        
        Returns:
            Adjusted image
        """
        if not OPENCV_AVAILABLE:
            return image
        
        # Convert brightness from -1,1 to -127,127
        beta = int(brightness * 127)
        
        return cv2.convertScaleAbs(image, alpha=contrast, beta=beta)
    
    @staticmethod
    def denoise(image: np.ndarray, strength: int = 5) -> np.ndarray:
        """
        Apply denoising to image.
        
        Args:
            image: Input image
            strength: Denoising strength (higher = more denoising)
        
        Returns:
            Denoised image
        """
        if not OPENCV_AVAILABLE: 
            return image
        
        if len(image.shape) == 3:
            return cv2.fastNlMeansDenoisingColored(image, None, strength, strength, 7, 21)
        else:
            return cv2.fastNlMeansDenoising(image, None, strength, 7, 21)
    
    @staticmethod
    def calculate_brightness(image: np.ndarray) -> float:
        """
        Calculate average brightness of image.
        
        Args:
            image: Input image
        
        Returns:
            Average brightness (0-255)
        """
        if not OPENCV_AVAILABLE: 
            return 128. 0
        
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        return float(np.mean(gray))
    
    @staticmethod
    def calculate_sharpness(image: np.ndarray) -> float:
        """
        Calculate image sharpness using Laplacian variance.
        
        Args:
            image: Input image
        
        Returns:
            Sharpness score (higher = sharper)
        """
        if not OPENCV_AVAILABLE: 
            return 0.0
        
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        return float(laplacian.var())
