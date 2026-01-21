#!/usr/bin/env python3
"""
Keyboard Teleoperation Node

Allows controlling the rover using keyboard input.

Key Bindings:
    Movement:
        w/↑ :  Forward
        s/↓ : Backward
        a/← : Turn Left
        d/→ : Turn Right
        q   : Forward + Left
        e   : Forward + Right
        z   : Backward + Left
        c   : Backward + Right
    
    Speed Control:
        i :  Increase linear speed
        k :  Decrease linear speed
        o :  Increase angular speed
        l :  Decrease angular speed
    
    Stop:
        Space : Emergency Stop (immediate)
        x     : Gradual Stop

Usage:
    ros2 run tmr_teleop teleop_keyboard
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# Terminal input handling
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


# Key bindings
MOVE_BINDINGS = {
    # WASD keys
    'w': (1. 0, 0.0),    # Forward
    's': (-1.0, 0.0),   # Backward
    'a': (0.0, 1.0),    # Turn left
    'd': (0.0, -1.0),   # Turn right
    'q': (1.0, 1.0),    # Forward + left
    'e': (1.0, -1.0),   # Forward + right
    'z': (-1.0, 1.0),   # Backward + left
    'c': (-1.0, -1.0),  # Backward + right
    
    # Arrow keys (using escape sequences)
    '\x1b[A': (1.0, 0.0),   # Up arrow
    '\x1b[B': (-1.0, 0.0),  # Down arrow
    '\x1b[C': (0.0, -1.0),  # Right arrow
    '\x1b[D': (0.0, 1.0),   # Left arrow
}

SPEED_BINDINGS = {
    'i': (0.1, 0.0),    # Increase linear
    'k': (-0.1, 0.0),   # Decrease linear
    'o': (0.0, 0.1),    # Increase angular
    'l':  (0.0, -0.1),   # Decrease angular
}

HELP_TEXT = """
╔══════════════════════════════════════════════════════════════╗
║           TMR Keyboard Teleoperation                         ║
╠══════════════════════════════════════════════════════════════╣
║  Movement:                    Speed Control:                 ║
║     q   w   e                   i :  Increase linear speed    ║
║     a   s   d                   k : Decrease linear speed    ║
║     z   x   c                   o : Increase angular speed   ║
║                                 l : Decrease angular speed   ║
║  Arrow keys also work                                        ║
║                                                              ║
║  Stop:                                                        ║
║     SPACE : Emergency Stop                                   ║
║     x     :  Gradual Stop                                     ║
║                                                              ║
║  Press CTRL+C to exit                                        ║
╚══════════════════════════════════════════════════════════════╝

Current Settings:
"""


class KeyboardReader: 
    """
    Cross-platform keyboard input reader.
    """
    
    def __init__(self):
        self.settings = None
        if sys.platform != 'win32':
            self.settings = termios.tcgetattr(sys.stdin)
    
    def read_key(self, timeout=0.1):
        """
        Read a single key press.
        
        Args:
            timeout: Read timeout in seconds
        
        Returns:
            Key character or None if timeout
        """
        if sys. platform == 'win32':
            if msvcrt.kbhit():
                return msvcrt.getwch()
            return None
        else:
            tty.setraw(sys.stdin.fileno())
            try:
                # Set up non-blocking read with timeout
                import select
                rlist, _, _ = select.select([sys.stdin], [], [], timeout)
                if rlist:
                    key = sys.stdin.read(1)
                    
                    # Handle escape sequences (arrow keys)
                    if key == '\x1b':
                        # Check for more characters
                        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
                        if rlist: 
                            key += sys.stdin.read(2)
                    
                    return key
                return None
            finally:
                termios.tcsetattr(sys. stdin, termios.TCSADRAIN, self.settings)
    
    def restore(self):
        """Restore terminal settings"""
        if sys.platform != 'win32' and self.settings:
            termios.tcsetattr(sys. stdin, termios.TCSADRAIN, self.settings)


class TeleopKeyboardNode(Node):
    """
    Keyboard teleoperation ROS 2 node.
    """
    
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Declare parameters
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('linear_velocity_step', 0.05)
        self.declare_parameter('angular_velocity_step', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('enable_estop', True)
        self.declare_parameter('idle_timeout', 0.5)
        
        # Get parameters
        self.max_linear = self.get_parameter('max_linear_velocity').value
        self.max_angular = self.get_parameter('max_angular_velocity').value
        self.linear_step = self.get_parameter('linear_velocity_step').value
        self.angular_step = self. get_parameter('angular_velocity_step').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_estop = self.get_parameter('enable_estop').value
        self.idle_timeout = self.get_parameter('idle_timeout').value
        
        # Current velocity settings
        self.linear_speed = self.max_linear * 0.5  # Start at 50%
        self.angular_speed = self.max_angular * 0.5
        
        # Current command
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # State
        self.running = True
        self.last_key_time = time.time()
        self.estop_active = False
        
        # Publisher
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Emergency stop service client (optional)
        self.estop_client = None
        if self.enable_estop:
            self.estop_client = self.create_client(Trigger, '/vex/emergency_stop')
        
        # Keyboard reader
        self.keyboard = KeyboardReader()
        
        # Print help
        self._print_status()
        
        # Start keyboard reading thread
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        
        # Timer for publishing velocity
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_callback)
    
    def _print_status(self):
        """Print current status"""
        print(HELP_TEXT)
        print(f"  Linear Speed:   {self.linear_speed:. 2f} m/s (max: {self.max_linear:. 2f})")
        print(f"  Angular Speed: {self.angular_speed:.2f} rad/s (max: {self.max_angular:.2f})")
        print(f"  E-Stop:  {'ACTIVE' if self.estop_active else 'Ready'}")
        print("")
    
    def _update_status_line(self):
        """Update the status line without reprinting everything"""
        status = f"\rVel: Linear={self.linear_vel:+.2f} m/s, Angular={self.angular_vel:+.2f} rad/s | "
        status += f"Speed:  Lin={self.linear_speed:. 2f}, Ang={self.angular_speed:.2f} | "
        status += f"E-Stop: {'ACTIVE' if self.estop_active else 'Off'}    "
        print(status, end='', flush=True)
    
    def _keyboard_loop(self):
        """Background thread for reading keyboard input"""
        while self. running:
            key = self. keyboard.read_key(timeout=0.1)
            
            if key is None:
                continue
            
            self.last_key_time = time.time()
            
            # Check for quit
            if key == '\x03':  # Ctrl+C
                self.running = False
                break
            
            # Emergency stop
            if key == ' ': 
                self._emergency_stop()
                continue
            
            # Movement keys
            if key. lower() in MOVE_BINDINGS or key in MOVE_BINDINGS: 
                lookup_key = key.lower() if key.lower() in MOVE_BINDINGS else key
                linear_mult, angular_mult = MOVE_BINDINGS[lookup_key]
                self.linear_vel = linear_mult * self.linear_speed
                self.angular_vel = angular_mult * self.angular_speed
                self. estop_active = False
                self._update_status_line()
                continue
            
            # Speed adjustment keys
            if key.lower() in SPEED_BINDINGS: 
                linear_delta, angular_delta = SPEED_BINDINGS[key.lower()]
                
                self.linear_speed += linear_delta
                self. linear_speed = max(0.05, min(self.max_linear, self.linear_speed))
                
                self.angular_speed += angular_delta
                self.angular_speed = max(0.1, min(self.max_angular, self.angular_speed))
                
                self._update_status_line()
                continue
            
            # Gradual stop
            if key. lower() == 'x':
                self. linear_vel = 0.0
                self.angular_vel = 0.0
                self._update_status_line()
                continue
    
    def _emergency_stop(self):
        """Trigger emergency stop"""
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.estop_active = True
        
        # Publish stop command immediately
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Call emergency stop service if available
        if self.estop_client is not None and self.estop_client.wait_for_service(timeout_sec=0.1):
            request = Trigger.Request()
            future = self.estop_client. call_async(request)
        
        self._update_status_line()
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')
    
    def _publish_callback(self):
        """Timer callback to publish velocity commands"""
        # Check for idle timeout
        if self.idle_timeout > 0:
            idle_time = time.time() - self.last_key_time
            if idle_time > self. idle_timeout:
                self. linear_vel = 0.0
                self.angular_vel = 0.0
        
        # Create and publish Twist message
        twist = Twist()
        
        if not self.estop_active:
            twist.linear.x = self.linear_vel
            twist. angular.z = self.angular_vel
        
        self.cmd_vel_pub.publish(twist)
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        
        # Send stop command
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Restore terminal
        self.keyboard.restore()
        
        print("\n\nKeyboard teleop shutting down...")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = TeleopKeyboardNode()
    
    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt: 
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
