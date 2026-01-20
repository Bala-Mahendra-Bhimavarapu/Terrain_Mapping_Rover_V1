"""
Depth Processor

Provides depth image processing and filtering utilities: 
- Temporal filtering (noise reduction over time)
- Spatial filtering (noise reduction in space)
- Edge-preserving filtering
- Hole filling
- Depth colorization for visualization
"""

import numpy as np
from typing import Optional, Tuple
from collections import deque
from enum import Enum

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError: 
    OPENCV_AVAILABLE = False


class HoleFillMode(Enum):
    """Hole filling modes"""
    NONE = "none"
    NEAREST = "nearest"
    LINEAR = "linear"


class DepthProcessor:
    """
    Depth image processing and filtering. 
    """
    
    def __init__(
        self,
        temporal_filter_alpha: float = 0.4,
        spatial_filter_size: int = 3,
        min_depth: float = 0.1,
        max_depth:  float = 4.0
    ):
        """
        Initialize depth processor.
        
        Args:
            temporal_filter_alpha: Temporal filter coefficient (0-1)
            spatial_filter_size: Spatial filter kernel size
            min_depth:  Minimum valid depth in meters
            max_depth: Maximum valid depth in meters
        """
        self.temporal_alpha = temporal_filter_alpha
        self.spatial_size = spatial_filter_size
        self.min_depth = min_depth
        self.max_depth = max_depth
        
        # Temporal filter state
        self.prev_depth:  Optional[np.ndarray] = None
        
        # History for advanced temporal filtering
        self.depth_history = deque(maxlen=5)
    
    def reset(self):
        """Reset filter state"""
        self.prev_depth = None
        self.depth_history.clear()
    
    def apply_temporal_filter(
        self,
        depth: np.ndarray,
        alpha: Optional[float] = None
    ) -> np.ndarray:
        """
        Apply temporal filtering (IIR filter).
        
        Smooths depth over time to reduce temporal noise.
        
        filtered = alpha * current + (1 - alpha) * previous
        
        Args: 
            depth: Current depth image
            alpha: Filter coefficient (uses default if None)
        
        Returns:
            Temporally filtered depth
        """
        if alpha is None:
            alpha = self.temporal_alpha
        
        if self.prev_depth is None:
            self.prev_depth = depth.copy()
            return depth
        
        # Only filter where both frames are valid
        valid_current = (depth > 0) & ~np.isnan(depth)
        valid_previous = (self. prev_depth > 0) & ~np.isnan(self. prev_depth)
        both_valid = valid_current & valid_previous
        
        # Apply filter
        filtered = depth. copy()
        filtered[both_valid] = (
            alpha * depth[both_valid] +
            (1 - alpha) * self.prev_depth[both_valid]
        )
        
        # Use current where previous was invalid
        filtered[valid_current & ~valid_previous] = depth[valid_current & ~valid_previous]
        
        # Update state
        self.prev_depth = filtered. copy()
        
        return filtered
    
    def apply_spatial_filter(
        self,
        depth: np.ndarray,
        kernel_size: Optional[int] = None
    ) -> np.ndarray:
        """
        Apply spatial filtering (median or bilateral).
        
        Args:
            depth:  Depth image
            kernel_size:  Filter kernel size (must be odd)
        
        Returns:
            Spatially filtered depth
        """
        if not OPENCV_AVAILABLE:
            return depth
        
        if kernel_size is None:
            kernel_size = self.spatial_size
        
        # Ensure odd kernel size
        kernel_size = kernel_size | 1
        
        # Create mask of valid depths
        valid = (depth > 0) & ~np.isnan(depth)
        
        # Apply median filter
        filtered = cv2.medianBlur(depth. astype(np.float32), kernel_size)
        
        # Restore invalid regions
        filtered[~valid] = 0
        
        return filtered
    
    def apply_bilateral_filter(
        self,
        depth: np.ndarray,
        d:  int = 5,
        sigma_color: float = 50,
        sigma_space: float = 50
    ) -> np.ndarray:
        """
        Apply edge-preserving bilateral filter.
        
        Smooths while preserving edges (depth discontinuities).
        
        Args:
            depth: Depth image
            d: Diameter of pixel neighborhood
            sigma_color: Filter sigma in the color space
            sigma_space:  Filter sigma in the coordinate space
        
        Returns: 
            Filtered depth
        """
        if not OPENCV_AVAILABLE:
            return depth
        
        # Normalize depth for filtering
        valid = (depth > 0) & ~np.isnan(depth)
        depth_norm = depth.astype(np.float32)
        
        if valid.any():
            depth_min = depth_norm[valid].min()
            depth_max = depth_norm[valid]. max()
            if depth_max > depth_min:
                depth_norm = (depth_norm - depth_min) / (depth_max - depth_min) * 255
        
        # Apply bilateral filter
        filtered_norm = cv2.bilateralFilter(
            depth_norm. astype(np.float32), d, sigma_color, sigma_space
        )
        
        # Denormalize
        if valid.any() and depth_max > depth_min:
            filtered = filtered_norm / 255 * (depth_max - depth_min) + depth_min
        else:
            filtered = depth
        
        # Restore invalid regions
        filtered[~valid] = 0
        
        return filtered. astype(depth.dtype)
    
    def fill_holes(
        self,
        depth: np.ndarray,
        mode: HoleFillMode = HoleFillMode.NEAREST,
        max_hole_size: int = 5
    ) -> np.ndarray:
        """
        Fill holes (invalid depth values) in depth image.
        
        Args:
            depth: Depth image
            mode: Hole filling mode
            max_hole_size: Maximum hole size to fill
        
        Returns: 
            Depth image with holes filled
        """
        if mode == HoleFillMode. NONE or not OPENCV_AVAILABLE: 
            return depth
        
        filled = depth.copy()
        invalid = (depth <= 0) | np.isnan(depth)
        
        if not invalid.any():
            return filled
        
        if mode == HoleFillMode. NEAREST:
            # Use morphological closing to fill small holes
            kernel = np.ones((max_hole_size, max_hole_size), np.uint8)
            
            # Create valid depth image
            valid_depth = filled.copy()
            valid_depth[invalid] = 0
            
            # Dilate valid regions
            dilated = cv2.dilate(valid_depth. astype(np.float32), kernel)
            
            # Fill holes with dilated values
            filled[invalid] = dilated[invalid]
            
        elif mode == HoleFillMode.LINEAR:
            # Use inpainting for linear interpolation
            mask = invalid.astype(np. uint8) * 255
            
            # Normalize depth for inpainting
            valid = ~invalid
            if valid.any():
                depth_min = filled[valid].min()
                depth_max = filled[valid].max()
                if depth_max > depth_min:
                    depth_norm = ((filled - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)
                    
                    # Inpaint
                    inpainted = cv2.inpaint(depth_norm, mask, max_hole_size, cv2.INPAINT_NS)
                    
                    # Denormalize
                    filled = inpainted. astype(np.float32) / 255 * (depth_max - depth_min) + depth_min
        
        return filled
    
    def apply_confidence_filter(
        self,
        depth: np.ndarray,
        confidence:  np.ndarray,
        threshold: int = 30
    ) -> np.ndarray:
        """
        Filter depth based on confidence values.
        
        Args:
            depth: Depth image
            confidence: Confidence image (0-255)
            threshold: Minimum confidence threshold
        
        Returns: 
            Filtered depth (low confidence set to 0)
        """
        filtered = depth.copy()
        filtered[confidence < threshold] = 0
        return filtered
    
    def clip_depth(
        self,
        depth: np.ndarray,
        min_depth: Optional[float] = None,
        max_depth: Optional[float] = None
    ) -> np.ndarray:
        """
        Clip depth to valid range.
        
        Args:
            depth: Depth image in meters
            min_depth:  Minimum valid depth
            max_depth: Maximum valid depth
        
        Returns:
            Clipped depth
        """
        if min_depth is None:
            min_depth = self.min_depth
        if max_depth is None: 
            max_depth = self. max_depth
        
        clipped = depth.copy()
        clipped[(depth < min_depth) | (depth > max_depth)] = 0
        return clipped
    
    def process(
        self,
        depth:  np.ndarray,
        confidence: Optional[np.ndarray] = None,
        confidence_threshold: int = 30,
        enable_temporal:  bool = True,
        enable_spatial: bool = True,
        hole_fill_mode: HoleFillMode = HoleFillMode. NONE
    ) -> np.ndarray:
        """
        Apply full processing pipeline.
        
        Args:
            depth: Raw depth image
            confidence: Confidence image
            confidence_threshold: Confidence threshold
            enable_temporal: Enable temporal filtering
            enable_spatial:  Enable spatial filtering
            hole_fill_mode: Hole filling mode
        
        Returns:
            Processed depth image
        """
        processed = depth.copy().astype(np.float32)
        
        # Apply confidence filter
        if confidence is not None:
            processed = self.apply_confidence_filter(
                processed, confidence, confidence_threshold
            )
        
        # Apply temporal filter
        if enable_temporal: 
            processed = self.apply_temporal_filter(processed)
        
        # Apply spatial filter
        if enable_spatial: 
            processed = self.apply_spatial_filter(processed)
        
        # Fill holes
        if hole_fill_mode != HoleFillMode.NONE:
            processed = self.fill_holes(processed, hole_fill_mode)
        
        return processed
    
    @staticmethod
    def colorize_depth(
        depth:  np.ndarray,
        min_depth: float = 0.1,
        max_depth:  float = 4.0,
        colormap: int = None
    ) -> np.ndarray:
        """
        Colorize depth image for visualization.
        
        Args:
            depth: Depth image in meters
            min_depth:  Minimum depth for colormap
            max_depth: Maximum depth for colormap
            colormap:  OpenCV colormap (default:  COLORMAP_JET)
        
        Returns:
            Colorized BGR image
        """
        if not OPENCV_AVAILABLE:
            # Return grayscale if OpenCV not available
            normalized = ((depth - min_depth) / (max_depth - min_depth) * 255)
            normalized = np.clip(normalized, 0, 255).astype(np.uint8)
            return np.stack([normalized] * 3, axis=-1)
        
        if colormap is None:
            colormap = cv2.COLORMAP_JET
        
        # Normalize depth
        valid = (depth > 0) & ~np.isnan(depth)
        normalized = np.zeros_like(depth, dtype=np.float32)
        normalized[valid] = (depth[valid] - min_depth) / (max_depth - min_depth)
        normalized = np.clip(normalized, 0, 1)
        
        # Convert to uint8
        depth_uint8 = (normalized * 255).astype(np.uint8)
        
        # Apply colormap
        colored = cv2.applyColorMap(depth_uint8, colormap)
        
        # Set invalid to black
        colored[~valid] = [0, 0, 0]
        
        return colored
    
    @staticmethod
    def depth_to_uint16(depth_m: np.ndarray, scale: float = 1000.0) -> np.ndarray:
        """
        Convert depth in meters to uint16 millimeters.
        
        Args:
            depth_m:  Depth in meters
            scale: Scale factor (1000 = millimeters)
        
        Returns:
            Depth in uint16
        """
        depth_scaled = depth_m * scale
        depth_scaled = np.clip(depth_scaled, 0, 65535)
        return depth_scaled.astype(np.uint16)
    
    @staticmethod
    def uint16_to_depth(depth_uint16: np.ndarray, scale: float = 0.001) -> np.ndarray:
        """
        Convert uint16 depth to meters.
        
        Args:
            depth_uint16: Depth in uint16 (typically mm)
            scale: Scale factor (0.001 = mm to m)
        
        Returns: 
            Depth in meters
        """
        return depth_uint16.astype(np.float32) * scale
