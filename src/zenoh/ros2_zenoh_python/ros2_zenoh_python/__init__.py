"""
ROS 2 Zenoh Python Package

A Python package that provides Zenoh as an alternative transport to rclpy for ROS 2.
This package enables direct Zenoh communication while maintaining ROS 2 compatibility.

Main components:
- Publisher: ROS 2-compatible publisher using Zenoh
- Subscriber: ROS 2-compatible subscriber using Zenoh
- Message serialization: CDR serialization for ROS 2 messages
- Liveliness tokens: ROS 2 metadata publishing via Zenoh liveliness tokens

Example usage:
    from ros2_zenoh_python import Publisher, Subscriber
    from geometry_msgs.msg import Twist
    
    # Create publisher
    pub = Publisher('/turtle1/cmd_vel', Twist)
    
    # Create subscriber
    def callback(msg):
        print(f"Received: {msg}")
    
    sub = Subscriber('/turtle1/cmd_vel', Twist, callback)
"""

from .publisher import Publisher
from .subscriber import Subscriber
from .node import Node
from .message_serializer import MessageSerializer
from .liveliness_manager import LivelinessManager
from .converter import MessageConverter, to_simple, to_ros2, get_simple_type
from .logger import Logger

# Import message types from separate module
from .message_types import (
    Time, Vector3, Twist, Point, Quaternion, Pose, 
    Header, PoseStamped, Duration, Log
)

INTERFACES_AVAILABLE = True

__version__ = "0.1.0"
__all__ = [
    "Publisher", "Subscriber", "Node", "MessageSerializer", "LivelinessManager",
    "MessageConverter", "to_simple", "to_ros2", "get_simple_type", "Logger",
    # Message types
    "Vector3", "Twist", "Time", "Log", "Pose", "Point", "Quaternion", "PoseStamped"
]

# Note: For unified CDR types, use ros2_interfaces_py package separately
# This package is designed to work with standard ROS 2 message types
