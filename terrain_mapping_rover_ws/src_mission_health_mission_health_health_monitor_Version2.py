"""
Core health monitoring logic. 

Aggregates health status from all subsystems and determines
overall mission health. 
"""

from enum import Enum
from typing import Dict, List, Optional
from dataclasses import dataclass, field
import time


class HealthStatus(Enum):
    """Health status levels."""
    OK = "OK"
    DEGRADED = "DEGRADED"
    CRITICAL = "CRITICAL"
    OFFLINE = "OFFLINE"


@dataclass
class SubsystemHealth:
    """Health state of a single subsystem."""
    name: str
    status: HealthStatus = HealthStatus. OFFLINE
    last_update: float = 0.0
    message: str = ""
    metrics: Dict[str, float] = field(default_factory=dict)


@dataclass
class MissionHealthState:
    """Overall mission health state."""
    status: HealthStatus = HealthStatus.OK
    slam_status: HealthStatus = HealthStatus. OFFLINE
    localization_status: HealthStatus = HealthStatus.OFFLINE
    navigation_status: HealthStatus = HealthStatus.OFFLINE
    perception_status: HealthStatus = HealthStatus.OFFLINE
    motor_status: HealthStatus = HealthStatus.OFFLINE
    
    slam_quality: float = 0.0
    localization_confidence: float = 0.0
    vo_feature_count: float = 0.0
    loop_closure_count: float = 0.0
    position_uncertainty: float = 0.0
    orientation_uncertainty: float = 0.0


class HealthMonitor:
    """
    Monitors overall mission health.
    
    Aggregates health from multiple subsystems and determines
    overall mission status.
    """
    
    def __init__(
        self,
        timeout_sec: float = 2.0,
        degraded_threshold:  int = 1,
        critical_threshold: int = 2
    ):
        """
        Initialize health monitor.
        
        Args:
            timeout_sec: Time before subsystem is considered offline
            degraded_threshold: Number of degraded subsystems for DEGRADED status
            critical_threshold:  Number of critical/offline subsystems for CRITICAL
        """
        self.timeout = timeout_sec
        self.degraded_threshold = degraded_threshold
        self.critical_threshold = critical_threshold
        
        self. subsystems: Dict[str, SubsystemHealth] = {}
        self._state = MissionHealthState()
    
    def register_subsystem(self, name:  str):
        """Register a subsystem to monitor."""
        self.subsystems[name] = SubsystemHealth(name=name)
    
    def update_subsystem(
        self,
        name: str,
        status: HealthStatus,
        message: str = "",
        metrics: Optional[Dict[str, float]] = None
    ):
        """Update a subsystem's health status."""
        if name not in self.subsystems:
            self.register_subsystem(name)
        
        self.subsystems[name].status = status
        self. subsystems[name].last_update = time.time()
        self.subsystems[name]. message = message
        if metrics:
            self.subsystems[name].metrics = metrics
    
    def check_timeouts(self):
        """Check for subsystem timeouts."""
        current_time = time.time()
        
        for name, subsystem in self.subsystems.items():
            if current_time - subsystem.last_update > self.timeout:
                subsystem.status = HealthStatus.OFFLINE
                subsystem. message = "Timeout - no updates received"
    
    def compute_overall_health(self) -> MissionHealthState:
        """Compute overall mission health from subsystems."""
        self.check_timeouts()
        
        # Count statuses
        critical_count = 0
        degraded_count = 0
        offline_count = 0
        
        for subsystem in self.subsystems.values():
            if subsystem.status == HealthStatus.CRITICAL:
                critical_count += 1
            elif subsystem.status == HealthStatus.DEGRADED:
                degraded_count += 1
            elif subsystem.status == HealthStatus.OFFLINE:
                offline_count += 1
        
        # Determine overall status
        if critical_count > 0 or offline_count >= self.critical_threshold:
            self._state.status = HealthStatus. CRITICAL
        elif degraded_count >= self.degraded_threshold or offline_count > 0:
            self._state.status = HealthStatus. DEGRADED
        else:
            self._state.status = HealthStatus.OK
        
        # Update individual subsystem statuses
        self._state.slam_status = self.subsystems. get(
            'slam', SubsystemHealth('slam')).status
        self._state. localization_status = self.subsystems.get(
            'localization', SubsystemHealth('localization')).status
        self._state.navigation_status = self. subsystems.get(
            'navigation', SubsystemHealth('navigation')).status
        self._state.perception_status = self.subsystems.get(
            'perception', SubsystemHealth('perception')).status
        self._state. motor_status = self.subsystems.get(
            'motors', SubsystemHealth('motors')).status
        
        # Aggregate metrics
        slam = self.subsystems.get('slam', SubsystemHealth('slam'))
        loc = self.subsystems.get('localization', SubsystemHealth('localization'))
        
        self._state.slam_quality = slam.metrics.get('quality', 0.0)
        self._state.vo_feature_count = slam.metrics.get('feature_count', 0.0)
        self._state.loop_closure_count = slam.metrics.get('loop_closures', 0.0)
        self._state.localization_confidence = loc.metrics. get('confidence', 0.0)
        self._state.position_uncertainty = loc.metrics.get('position_uncertainty', 0.0)
        self._state.orientation_uncertainty = loc.metrics.get('orientation_uncertainty', 0.0)
        
        return self._state
    
    @property
    def state(self) -> MissionHealthState:
        """Get current health state."""
        return self._state