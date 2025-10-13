"""
ROS 2 Message Types

Simplified Python dataclasses for ROS 2 message types, organized by package.
"""

# Import package-specific message types
from .geometry_msgs import (
    Vector3, Point, Quaternion, Pose, Twist, Transform, Accel, Wrench,
    Point32, Pose2D, PoseStamped, TwistStamped, TransformStamped, PoseArray
)

from .std_msgs import (
    Header, Empty, Bool, String, Int8, Int16, Int32, Int64,
    UInt8, UInt16, UInt32, UInt64, Float32, Float64, Byte, Char,
    ColorRGBA, MultiArrayDimension, MultiArrayLayout, Int8MultiArray,
    UInt8MultiArray, Float32MultiArray
)

# Export all available message types
__all__ = [
    # geometry_msgs types
    'Vector3', 'Point', 'Quaternion', 'Pose', 'Twist', 'Transform', 'Accel', 'Wrench',
    'Point32', 'Pose2D', 'PoseStamped', 'TwistStamped', 'TransformStamped', 'PoseArray',
    
    # std_msgs types
    'Header', 'Empty', 'Bool', 'String', 'Int8', 'Int16', 'Int32', 'Int64',
    'UInt8', 'UInt16', 'UInt32', 'UInt64', 'Float32', 'Float64', 'Byte', 'Char',
    'ColorRGBA', 'MultiArrayDimension', 'MultiArrayLayout', 'Int8MultiArray',
    'UInt8MultiArray', 'Float32MultiArray'
]
