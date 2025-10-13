"""
Message Serialization Module

Handles ROS 2 message serialization/deserialization using pycdr2.
Supports both ROS 2 message classes and manual pycdr2 definitions.
"""

import os
import struct
import time
from typing import Any, Optional, Type, Union
from dataclasses import dataclass

try:
    from pycdr2 import IdlStruct
    from pycdr2.types import int8, int32, uint32, float64
    PYCDR2_AVAILABLE = True
except ImportError:
    PYCDR2_AVAILABLE = False

try:
    from geometry_msgs.msg import Twist as ROS2Twist, Vector3 as ROS2Vector3
    from builtin_interfaces.msg import Time as ROS2Time
    from rcl_interfaces.msg import Log as ROS2Log
    ROS2_MSGS_AVAILABLE = True
except ImportError:
    ROS2_MSGS_AVAILABLE = False


class MessageSerializer:
    """Handles ROS 2 message serialization/deserialization."""
    
    def __init__(self):
        self._pycdr2_classes = {}
        self._ros2_classes = {}
        self._setup_default_classes()
    
    def _setup_default_classes(self):
        """Setup default pycdr2 classes for common ROS 2 messages."""
        if not PYCDR2_AVAILABLE:
            return
            
        # Define common pycdr2 classes
        @dataclass
        class Vector3(IdlStruct, typename="Vector3"):
            x: float64
            y: float64
            z: float64

        @dataclass
        class Twist(IdlStruct, typename="Twist"):
            linear: Vector3
            angular: Vector3

        @dataclass
        class Time(IdlStruct, typename="Time"):
            sec: int32
            nanosec: uint32

        @dataclass
        class Log(IdlStruct, typename="Log"):
            stamp: Time
            level: int8
            name: str
            msg: str
            file: str
            function: str
            line: uint32

        self._pycdr2_classes = {
            'Vector3': Vector3,
            'Twist': Twist,
            'Time': Time,
            'Log': Log
        }
    
    def serialize_message(self, message: Any) -> bytes:
        """
        Serialize a ROS 2 message to CDR bytes.
        
        Args:
            message: ROS 2 message instance or pycdr2 message instance
            
        Returns:
            Serialized CDR bytes
        """
        if ROS2_MSGS_AVAILABLE and hasattr(message, '__class__'):
            # Try to serialize as ROS 2 message
            if isinstance(message, ROS2Twist):
                return self._serialize_twist(message)
            elif isinstance(message, ROS2Log):
                return self._serialize_log(message)
        
        # Fallback to pycdr2 serialization
        if hasattr(message, 'serialize'):
            return message.serialize()
        
        raise ValueError(f"Cannot serialize message of type {type(message)}")
    
    def _serialize_twist(self, ros2_twist: ROS2Twist) -> bytes:
        """Serialize a ROS 2 Twist message."""
        if 'Twist' not in self._pycdr2_classes:
            raise ValueError("Twist class not available")
        
        twist = self._pycdr2_classes['Twist'](
            linear=self._pycdr2_classes['Vector3'](
                x=ros2_twist.linear.x,
                y=ros2_twist.linear.y,
                z=ros2_twist.linear.z
            ),
            angular=self._pycdr2_classes['Vector3'](
                x=ros2_twist.angular.x,
                y=ros2_twist.angular.y,
                z=ros2_twist.angular.z
            )
        )
        return twist.serialize()
    
    def _serialize_log(self, ros2_log: ROS2Log) -> bytes:
        """Serialize a ROS 2 Log message."""
        if 'Log' not in self._pycdr2_classes:
            raise ValueError("Log class not available")
        
        log = self._pycdr2_classes['Log'](
            stamp=self._pycdr2_classes['Time'](
                sec=ros2_log.stamp.sec,
                nanosec=ros2_log.stamp.nanosec
            ),
            level=ros2_log.level,
            name=ros2_log.name,
            msg=ros2_log.msg,
            file=ros2_log.file,
            function=ros2_log.function,
            line=ros2_log.line
        )
        return log.serialize()
    
    def deserialize_message(self, data: bytes, message_type: Type) -> Any:
        """
        Deserialize CDR bytes to a ROS 2 message.
        
        Args:
            data: Serialized CDR bytes
            message_type: ROS 2 message type class
            
        Returns:
            Deserialized message instance
        """
        # For now, we'll focus on serialization
        # Deserialization would require more complex type mapping
        raise NotImplementedError("Deserialization not yet implemented")
    
    def create_twist_message(self, linear_x: float = 0.0, linear_y: float = 0.0, linear_z: float = 0.0,
                           angular_x: float = 0.0, angular_y: float = 0.0, angular_z: float = 0.0) -> Any:
        """Create a Twist message."""
        if ROS2_MSGS_AVAILABLE:
            ros2_twist = ROS2Twist()
            ros2_twist.linear.x = linear_x
            ros2_twist.linear.y = linear_y
            ros2_twist.linear.z = linear_z
            ros2_twist.angular.x = angular_x
            ros2_twist.angular.y = angular_y
            ros2_twist.angular.z = angular_z
            return ros2_twist
        else:
            if 'Twist' not in self._pycdr2_classes:
                raise ValueError("Twist class not available")
            return self._pycdr2_classes['Twist'](
                linear=self._pycdr2_classes['Vector3'](x=linear_x, y=linear_y, z=linear_z),
                angular=self._pycdr2_classes['Vector3'](x=angular_x, y=angular_y, z=angular_z)
            )
