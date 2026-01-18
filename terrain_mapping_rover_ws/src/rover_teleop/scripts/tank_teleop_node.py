#!/usr/bin/env python3
"""
Tank Teleop Node - Direct left/right wheel control.

Uses keyboard to control left and right wheels independently: 
- I/K: Left wheels forward/backward
- O/L:  Right wheels forward/backward
- T: Both forward
- G: Both backward
- Space: Stop

Author: Rover Team
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from rover_interfaces.msg import TankDriveCmd


HELP_MSG = """
╔════════════════════════════════════════════════════════╗
║           TANK DRIVE TELEOP (Direct Wheels)            ║
╠════════════════════════════════════════════════════════╣
║                                                        ║
║   Left Wheels:        Right Wheels:                    ║
║   ┌─────────┐         ┌─────────┐                      ║
║   │    I    │         │    O    │  Forward             ║
║   │    K    │         │    L    │  Backward            ║
║   └─────────┘         └─────────┘                      ║
║                                                        ║
║   Both Wheels:                                         ║
║   ┌─────────────────────��───────┐                      ║
║   │   T = Both Forward          │                      ║
║   │   G = Both Backward         │                      ║
║   │   F = Spin Left             │                      ║
║   │   H = Spin Right            │                      ║
║   │   Space = STOP              │                      ║
║   └─────────────────────────────┘                      ║
║                                                        ║
║   Speed:  +/- to adjust                                 ║
║   Ctrl+C to quit                                       ║
║                                                        ║
╚════════════════════════════════════════════════════════╝
"""


class TankTeleopNode(Node):
    """Tank drive teleop with individual wheel control."""
    
    def __init__(self):
        super().__init__('tank_teleop_node')
        
        self.declare_parameter('wheel_speed', 0.2)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('speed_increment', 0.05)
        
        self.wheel_speed = self.get_parameter('wheel_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        
        self.cmd_pub = self.create_publisher(TankDriveCmd, '/vex/tank_cmd', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Tank Teleop started')
        print(HELP_MSG)
        self._print_speed()
    
    def _print_speed(self):
        """Print current speed."""
        print(f'\rWheel Speed: {self.wheel_speed:.2f} m/s    ', end='')
        sys.stdout.flush()
    
    def _get_key(self):
        """Get keypress."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            return sys.stdin.read(1)
        return None
    
    def run(self):
        """Main loop."""
        left_vel = 0.0
        right_vel = 0.0
        
        try:
            while rclpy.ok():
                key = self._get_key()
                
                if key is not None:
                    key_lower = key.lower()
                    
                    if key == '\x03': 
                        break
                    
                    # Individual wheel control
                    if key_lower == 'i':
                        left_vel = self.wheel_speed
                    elif key_lower == 'k':
                        left_vel = -self.wheel_speed
                    elif key_lower == 'o':
                        right_vel = self.wheel_speed
                    elif key_lower == 'l':
                        right_vel = -self.wheel_speed
                    
                    # Both wheels
                    elif key_lower == 't':
                        left_vel = self.wheel_speed
                        right_vel = self.wheel_speed
                    elif key_lower == 'g': 
                        left_vel = -self.wheel_speed
                        right_vel = -self.wheel_speed
                    elif key_lower == 'f':
                        left_vel = -self.wheel_speed
                        right_vel = self.wheel_speed
                    elif key_lower == 'h': 
                        left_vel = self.wheel_speed
                        right_vel = -self.wheel_speed
                    
                    # Stop
                    elif key == ' ':
                        left_vel = 0.0
                        right_vel = 0.0
                    
                    # Speed control
                    elif key in ['+', '=']:
                        self.wheel_speed = min(self.max_speed, 
                                              self.wheel_speed + self. speed_increment)
                        self._print_speed()
                    elif key in ['-', '_']:
                        self.wheel_speed = max(0.05, 
                                              self.wheel_speed - self.speed_increment)
                        self._print_speed()
                else:
                    # No key - apply decay to stop gradually
                    left_vel *= 0.9
                    right_vel *= 0.9
                    if abs(left_vel) < 0.01: 
                        left_vel = 0.0
                    if abs(right_vel) < 0.01:
                        right_vel = 0.0
                
                # Publish command
                cmd = TankDriveCmd()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.left_velocity = left_vel
                cmd. right_velocity = right_vel
                cmd.use_velocity = True
                cmd.emergency_stop = False
                self.cmd_pub.publish(cmd)
                
                rclpy.spin_once(self, timeout_sec=0)
                
        finally:
            termios.tcsetattr(sys. stdin, termios.TCSADRAIN, self.settings)
            
            # Stop
            cmd = TankDriveCmd()
            cmd.header. stamp = self.get_clock().now().to_msg()
            cmd.emergency_stop = True
            self.cmd_pub.publish(cmd)
            
            print('\nTank Teleop stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = TankTeleopNode()
    try:
        node.run()
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
