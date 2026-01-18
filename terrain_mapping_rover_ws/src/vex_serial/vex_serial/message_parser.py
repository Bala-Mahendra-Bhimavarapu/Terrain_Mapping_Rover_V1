"""
Message parsing utilities for VEX serial protocol.
"""

from typing import Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class ParseResult:
    """Result of parsing a serial message."""
    success: bool
    message_type: str = ""
    data: Dict[str, Any] = None
    error: str = ""
    
    def __post_init__(self):
        if self.data is None:
            self.data = {}


class MessageParser: 
    """
    Parses ASCII messages from VEX V5.
    
    Message Formats:
    ----------------
    ODOM: "ODOM,<l_ticks>,<r_ticks>,<l_vel>,<r_vel>,<timestamp>,<checksum>"
    STATUS: "STATUS,<battery>,<t1>,<t2>,<t3>,<t4>,<checksum>"
    ERROR: "ERROR,<code>,<message>,<checksum>"
    ACK: "ACK,<cmd_id>,<checksum>"
    """
    
    CHECKSUM_MODULO = 256
    
    @classmethod
    def parse(cls, message: str) -> ParseResult:
        """
        Parse a message string. 
        
        Args:
            message: Raw message string (without newline)
            
        Returns: 
            ParseResult with parsed data or error
        """
        if not message: 
            return ParseResult(success=False, error="Empty message")
        
        parts = message.split(',')
        if len(parts) < 2:
            return ParseResult(success=False, error="Invalid message format")
        
        msg_type = parts[0]. upper()
        
        # Validate checksum
        if not cls._validate_checksum(parts):
            return ParseResult(
                success=False,
                message_type=msg_type,
                error="Checksum validation failed"
            )
        
        # Parse based on message type
        if msg_type == "ODOM": 
            return cls._parse_odom(parts)
        elif msg_type == "STATUS":
            return cls._parse_status(parts)
        elif msg_type == "ERROR":
            return cls._parse_error(parts)
        elif msg_type == "ACK": 
            return cls._parse_ack(parts)
        else:
            return ParseResult(
                success=False,
                message_type=msg_type,
                error=f"Unknown message type: {msg_type}"
            )
    
    @classmethod
    def _validate_checksum(cls, parts: list) -> bool:
        """Validate message checksum."""
        try:
            received_checksum = int(parts[-1])
            body = ','.join(parts[:-1])
            expected = sum(ord(c) for c in body) % cls.CHECKSUM_MODULO
            return received_checksum == expected
        except (ValueError, IndexError):
            return False
    
    @classmethod
    def _parse_odom(cls, parts: list) -> ParseResult:
        """Parse odometry message."""
        # ODOM,l_ticks,r_ticks,l_vel,r_vel,timestamp,checksum
        if len(parts) != 7:
            return ParseResult(
                success=False,
                message_type="ODOM",
                error=f"Expected 7 fields, got {len(parts)}"
            )
        
        try:
            data = {
                'left_ticks': int(parts[1]),
                'right_ticks': int(parts[2]),
                'left_velocity': float(parts[3]),
                'right_velocity': float(parts[4]),
                'timestamp_ms': int(parts[5])
            }
            return ParseResult(success=True, message_type="ODOM", data=data)
        except ValueError as e:
            return ParseResult(
                success=False,
                message_type="ODOM",
                error=f"Parse error: {e}"
            )
    
    @classmethod
    def _parse_status(cls, parts: list) -> ParseResult:
        """Parse status message."""
        # STATUS,battery,t1,t2,t3,t4,checksum
        if len(parts) != 7:
            return ParseResult(
                success=False,
                message_type="STATUS",
                error=f"Expected 7 fields, got {len(parts)}"
            )
        
        try:
            data = {
                'battery_voltage': float(parts[1]),
                'motor_temps': [
                    float(parts[2]),
                    float(parts[3]),
                    float(parts[4]),
                    float(parts[5])
                ]
            }
            return ParseResult(success=True, message_type="STATUS", data=data)
        except ValueError as e:
            return ParseResult(
                success=False,
                message_type="STATUS",
                error=f"Parse error: {e}"
            )
    
    @classmethod
    def _parse_error(cls, parts: list) -> ParseResult:
        """Parse error message."""
        # ERROR,code,message,checksum
        if len(parts) < 3:
            return ParseResult(
                success=False,
                message_type="ERROR",
                error="Invalid error message format"
            )
        
        data = {
            'code': parts[1],
            'message':  ','.join(parts[2:-1])  # Message might contain commas
        }
        return ParseResult(success=True, message_type="ERROR", data=data)
    
    @classmethod
    def _parse_ack(cls, parts: list) -> ParseResult:
        """Parse acknowledgment message."""
        # ACK,cmd_id,checksum
        if len(parts) != 3:
            return ParseResult(
                success=False,
                message_type="ACK",
                error=f"Expected 3 fields, got {len(parts)}"
            )
        
        data = {'cmd_id': parts[1]}
        return ParseResult(success=True, message_type="ACK", data=data)
    
    @classmethod
    def format_command(cls, left_vel: float, right_vel: float) -> str:
        """
        Format a velocity command message.
        
        Args:
            left_vel: Left wheel velocity (m/s)
            right_vel: Right wheel velocity (m/s)
            
        Returns:
            Formatted command string with checksum
        """
        body = f"CMD,{left_vel:. 3f},{right_vel:.3f}"
        checksum = sum(ord(c) for c in body) % cls.CHECKSUM_MODULO
        return f"{body},{checksum}\n"
    
    @classmethod
    def format_stop_command(cls) -> str:
        """Format emergency stop command."""
        return cls.format_command(0.0, 0.0)
