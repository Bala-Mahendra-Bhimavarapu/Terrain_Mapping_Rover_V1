"""
Brightness analyzer for exposure control. 

Analyzes image brightness using various methods suitable
for harsh lunar lighting conditions.
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class BrightnessResult:
    """Result of brightness analysis."""
    mean_brightness: float          # 0-255
    median_brightness: float        # 0-255
    weighted_brightness: float      # 0-255 (ROI weighted)
    histogram_peak: float           # 0-255
    overexposed_ratio: float        # 0-1
    underexposed_ratio:  float       # 0-1
    dynamic_range: float            # 0-255
    recommended_brightness: float   # Final recommendation


class BrightnessAnalyzer:
    """
    Analyzes image brightness for exposure control.
    
    Uses multiple methods to handle challenging lunar terrain:
    - Mean/median for general brightness
    - ROI weighting for center-focused metering
    - Histogram analysis for high-contrast scenes
    - Over/underexposure detection
    """
    
    def __init__(
        self,
        overexposed_threshold: int = 250,
        underexposed_threshold: int = 5,
        roi_weight: float = 0.7
    ):
        """
        Initialize brightness analyzer.
        
        Args:
            overexposed_threshold: Pixel value above this is overexposed
            underexposed_threshold: Pixel value below this is underexposed
            roi_weight: Weight given to ROI vs full image (0-1)
        """
        self.overexposed_threshold = overexposed_threshold
        self. underexposed_threshold = underexposed_threshold
        self.roi_weight = roi_weight
    
    def analyze(
        self,
        image: np.ndarray,
        roi:  Optional[Tuple[float, float, float, float]] = None
    ) -> BrightnessResult:
        """
        Analyze image brightness. 
        
        Args:
            image: Input image (BGR or grayscale)
            roi: Region of interest (x, y, width, height) normalized 0-1
                 Default is center 50%
        
        Returns:
            BrightnessResult with all metrics
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = np.mean(image, axis=2).astype(np.uint8)
        else:
            gray = image
        
        h, w = gray.shape
        
        # Default ROI is center 50%
        if roi is None:
            roi = (0.25, 0.25, 0.5, 0.5)
        
        # Extract ROI
        roi_x = int(roi[0] * w)
        roi_y = int(roi[1] * h)
        roi_w = int(roi[2] * w)
        roi_h = int(roi[3] * h)
        roi_region = gray[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        
        # Calculate metrics
        mean_brightness = float(np.mean(gray))
        median_brightness = float(np.median(gray))
        roi_brightness = float(np.mean(roi_region))
        
        # Weighted brightness (emphasize ROI)
        weighted_brightness = (
            self.roi_weight * roi_brightness +
            (1 - self.roi_weight) * mean_brightness
        )
        
        # Histogram analysis
        histogram, _ = np.histogram(gray. flatten(), bins=256, range=(0, 256))
        histogram_peak = float(np.argmax(histogram))
        
        # Over/underexposure ratios
        total_pixels = gray.size
        overexposed_ratio = float(np.sum(gray > self.overexposed_threshold)) / total_pixels
        underexposed_ratio = float(np.sum(gray < self.underexposed_threshold)) / total_pixels
        
        # Dynamic range
        dynamic_range = float(np. percentile(gray, 95) - np.percentile(gray, 5))
        
        # Recommended brightness (balance multiple factors)
        if overexposed_ratio > 0.1:
            # Too bright, use lower estimate
            recommended = min(weighted_brightness, median_brightness)
        elif underexposed_ratio > 0.3:
            # Too dark, use higher estimate
            recommended = max(weighted_brightness, median_brightness)
        else:
            # Normal conditions
            recommended = weighted_brightness
        
        return BrightnessResult(
            mean_brightness=mean_brightness,
            median_brightness=median_brightness,
            weighted_brightness=weighted_brightness,
            histogram_peak=histogram_peak,
            overexposed_ratio=overexposed_ratio,
            underexposed_ratio=underexposed_ratio,
            dynamic_range=dynamic_range,
            recommended_brightness=recommended
        )