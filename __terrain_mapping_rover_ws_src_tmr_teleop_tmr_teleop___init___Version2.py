"""
TMR Teleop Package

Teleoperation package for the Terrain Mapping Rover. 

Features:
- Keyboard teleoperation (WASD + arrow keys)
- Gamepad/joystick teleoperation
- Velocity smoothing (acceleration limiting)
- Input multiplexing (priority-based input selection)

Supported Controllers:
- Keyboard (WASD + arrow keys)
- Xbox controllers
- PlayStation controllers
- Logitech gamepads
- Most HID-compliant game controllers
"""

# Velocity smoother (standalone class + ROS node)
from .velocity_smoother import VelocitySmoother, VelocitySmootherNode

# Input multiplexer
from .teleop_mux import TeleopMuxNode, InputSource

# Teleop nodes (typically run as executables, but available for import)
from .teleop_keyboard import TeleopKeyboardNode, KeyboardReader
from .teleop_gamepad import TeleopGamepadNode

__all__ = [
    # Velocity smoothing
    'VelocitySmoother',
    'VelocitySmootherNode',
    
    # Input multiplexing
    'TeleopMuxNode',
    'InputSource',
    
    # Keyboard teleop
    'TeleopKeyboardNode',
    'KeyboardReader',
    
    # Gamepad teleop
    'TeleopGamepadNode',
]

__version__ = '1.0.0'