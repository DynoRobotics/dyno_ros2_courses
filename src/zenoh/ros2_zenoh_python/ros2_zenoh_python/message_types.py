"""
Message Types for ros2_zenoh_python

Simplified ROS 2 message types that work without ROS 2 dependencies.
"""

from dataclasses import dataclass

@dataclass
class Time:
    """Simplified builtin_interfaces.msg.Time message."""
    sec: int = 0
    nanosec: int = 0

@dataclass
class Vector3:
    """Simplified geometry_msgs.msg.Vector3 message."""
    x: float = 0.0  # float64 in ROS 2
    y: float = 0.0  # float64 in ROS 2
    z: float = 0.0  # float64 in ROS 2

@dataclass
class Twist:
    """Simplified geometry_msgs.msg.Twist message."""
    linear: Vector3 = None
    angular: Vector3 = None
    
    def __post_init__(self):
        if self.linear is None:
            self.linear = Vector3()
        if self.angular is None:
            self.angular = Vector3()

@dataclass
class Point:
    """Simplified geometry_msgs.msg.Point message."""
    x: float = 0.0  # float64 in ROS 2
    y: float = 0.0  # float64 in ROS 2
    z: float = 0.0  # float64 in ROS 2

@dataclass
class Quaternion:
    """Simplified geometry_msgs.msg.Quaternion message."""
    x: float = 0.0  # float64 in ROS 2
    y: float = 0.0  # float64 in ROS 2
    z: float = 0.0  # float64 in ROS 2
    w: float = 1.0  # float64 in ROS 2

@dataclass
class Pose:
    """Simplified geometry_msgs.msg.Pose message."""
    position: Point = None
    orientation: Quaternion = None
    
    def __post_init__(self):
        if self.position is None:
            self.position = Point()
        if self.orientation is None:
            self.orientation = Quaternion()

@dataclass
class Header:
    """Simplified std_msgs.msg.Header message."""
    stamp: Time = None
    frame_id: str = ""
    
    def __post_init__(self):
        if self.stamp is None:
            self.stamp = Time()

@dataclass
class PoseStamped:
    """Simplified geometry_msgs.msg.PoseStamped message."""
    header: Header = None
    pose: Pose = None
    
    def __post_init__(self):
        if self.header is None:
            self.header = Header()
        if self.pose is None:
            self.pose = Pose()

@dataclass
class Duration:
    """Simplified builtin_interfaces.msg.Duration message."""
    sec: int = 0
    nanosec: int = 0

@dataclass
class Log:
    """Simplified log message."""
    stamp: Time = None
    level: int = 0
    name: str = ""
    msg: str = ""
    file: str = ""
    function: str = ""
    line: int = 0
    
    def __post_init__(self):
        if self.stamp is None:
            self.stamp = Time()

# Export all message types
__all__ = [
    'Time', 'Vector3', 'Twist', 'Point', 'Quaternion', 'Pose', 
    'Header', 'PoseStamped', 'Duration', 'Log'
]
