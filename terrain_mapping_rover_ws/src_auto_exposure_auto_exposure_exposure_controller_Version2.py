"""
Exposure controller for camera. 

Implements a PID-like controller to adjust exposure and gain
to maintain target brightness.
"""

import time
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class ExposureSettings:
    """Current exposure settings."""
    exposure_time_us: int
    analogue_gain: float
    digital_gain: float


@dataclass
class ExposureLimits:
    """Exposure parameter limits."""
    min_exposure_us: int = 100
    max_exposure_us:  int = 100000
    min_gain: float = 1.0
    max_gain: float = 16.0
    min_digital_gain: float = 1.0
    max_digital_gain: float = 4.0


class ExposureController:
    """
    PID-style exposure controller.
    
    Adjusts exposure time first, then gain if needed.
    Includes motion detection to lock exposure during movement.
    """
    
    def __init__(
        self,
        target_brightness: float = 128.0,
        tolerance: float = 10.0,
        limits: Optional[ExposureLimits] = None,
        kp: float = 0.5,
        ki: float = 0.1,
        kd: float = 0.05
    ):
        """
        Initialize exposure controller.
        
        Args:
            target_brightness: Target brightness (0-255)
            tolerance:  Acceptable deviation from target
            limits: Exposure parameter limits
            kp, ki, kd: PID gains
        """
        self.target_brightness = target_brightness
        self.tolerance = tolerance
        self.limits = limits or ExposureLimits()
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # State
        self._current = ExposureSettings(
            exposure_time_us=20000,
            analogue_gain=1.0,
            digital_gain=1.0
        )
        
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()
        self._locked = False
        self._stable_count = 0
    
    @property
    def current_settings(self) -> ExposureSettings:
        """Get current exposure settings."""
        return self._current
    
    @property
    def is_locked(self) -> bool:
        """Check if exposure is locked (stable)."""
        return self._locked
    
    def set_target(self, brightness: float):
        """Set target brightness."""
        self.target_brightness = max(0, min(255, brightness))
        self._integral = 0.0
        self._locked = False
    
    def lock(self):
        """Lock current exposure settings."""
        self._locked = True
    
    def unlock(self):
        """Unlock exposure settings."""
        self._locked = False
        self._integral = 0.0
    
    def update(self, current_brightness: float) -> Tuple[ExposureSettings, bool]:
        """
        Update exposure based on current brightness.
        
        Args:
            current_brightness:  Measured brightness (0-255)
            
        Returns:
            Tuple of (new_settings, changed)
        """
        if self._locked:
            return self._current, False
        
        # Calculate error
        error = self. target_brightness - current_brightness
        
        # Check if within tolerance
        if abs(error) < self.tolerance:
            self._stable_count += 1
            if self._stable_count > 10: 
                self._locked = True
            return self._current, False
        else:
            self._stable_count = 0
        
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self._last_time
        self._last_time = current_time
        
        if dt <= 0 or dt > 1.0:
            dt = 0.1
        
        # PID calculation
        self._integral += error * dt
        self._integral = max(-1000, min(1000, self._integral))  # Anti-windup
        
        derivative = (error - self._last_error) / dt
        self._last_error = error
        
        adjustment = self.kp * error + self.ki * self._integral + self.kd * derivative
        
        # Apply adjustment (exposure first, then gain)
        new_settings = self._apply_adjustment(adjustment)
        
        changed = (
            new_settings.exposure_time_us != self._current.exposure_time_us or
            new_settings.analogue_gain != self._current.analogue_gain or
            new_settings.digital_gain != self._current.digital_gain
        )
        
        self._current = new_settings
        return new_settings, changed
    
    def _apply_adjustment(self, adjustment: float) -> ExposureSettings:
        """Apply brightness adjustment to exposure settings."""
        # Normalize adjustment to exposure multiplier
        # Positive adjustment = need more light = increase exposure
        multiplier = 1.0 + (adjustment / 255.0)
        multiplier = max(0.5, min(2.0, multiplier))
        
        # Start with current settings
        new_exposure = self._current.exposure_time_us
        new_again = self._current.analogue_gain
        new_dgain = self._current.digital_gain
        
        # Adjust exposure time first
        new_exposure = int(new_exposure * multiplier)
        new_exposure = max(self.limits.min_exposure_us,
                          min(self.limits.max_exposure_us, new_exposure))
        
        # If exposure is at limit, adjust gain
        if new_exposure >= self.limits.max_exposure_us and multiplier > 1.0:
            # Need more light, increase gain
            gain_multiplier = multiplier
            new_again = new_again * gain_multiplier
            new_again = max(self.limits.min_gain,
                           min(self.limits.max_gain, new_again))
            
            # If analog gain maxed, use digital gain
            if new_again >= self.limits.max_gain:
                new_dgain = new_dgain * gain_multiplier
                new_dgain = max(self.limits.min_digital_gain,
                               min(self.limits.max_digital_gain, new_dgain))
        
        elif new_exposure <= self.limits. min_exposure_us and multiplier < 1.0:
            # Too much light, decrease gain first
            gain_multiplier = multiplier
            new_dgain = new_dgain * gain_multiplier
            new_dgain = max(self.limits. min_digital_gain, new_dgain)
            
            if new_dgain <= self.limits.min_digital_gain:
                new_again = new_again * gain_multiplier
                new_again = max(self.limits.min_gain, new_again)
        
        return ExposureSettings(
            exposure_time_us=new_exposure,
            analogue_gain=round(new_again, 2),
            digital_gain=round(new_dgain, 2)
        )
    
    def reset(self):
        """Reset controller state."""
        self._integral = 0.0
        self._last_error = 0.0
        self._locked = False
        self._stable_count = 0
        self._current = ExposureSettings(
            exposure_time_us=20000,
            analogue_gain=1.0,
            digital_gain=1.0
        )