"""
Simplified geometry_msgs types
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any


@dataclass
class Vector3:
    """Simplified geometry_msgs.msg.Vector3 message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Point:
    """Simplified geometry_msgs.msg.Point message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    """Simplified geometry_msgs.msg.Quaternion message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


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
class Transform:
    """Simplified geometry_msgs.msg.Transform message."""
    translation: Vector3 = None
    rotation: Quaternion = None
    
    def __post_init__(self):
        if self.translation is None:
            self.translation = Vector3()
        if self.rotation is None:
            self.rotation = Quaternion()


@dataclass
class Accel:
    """Simplified geometry_msgs.msg.Accel message."""
    linear: Vector3 = None
    angular: Vector3 = None
    
    def __post_init__(self):
        if self.linear is None:
            self.linear = Vector3()
        if self.angular is None:
            self.angular = Vector3()


@dataclass
class Wrench:
    """Simplified geometry_msgs.msg.Wrench message."""
    force: Vector3 = None
    torque: Vector3 = None
    
    def __post_init__(self):
        if self.force is None:
            self.force = Vector3()
        if self.torque is None:
            self.torque = Vector3()


@dataclass
class Point32:
    """Simplified geometry_msgs.msg.Point32 message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Pose2D:
    """Simplified geometry_msgs.msg.Pose2D message."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


# Stamped messages (with simplified headers)
@dataclass
class PoseStamped:
    """Simplified geometry_msgs.msg.PoseStamped message."""
    header: Optional[Dict[str, Any]] = None
    pose: Pose = None
    
    def __post_init__(self):
        if self.pose is None:
            self.pose = Pose()
        if self.header is None:
            self.header = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}


@dataclass
class TwistStamped:
    """Simplified geometry_msgs.msg.TwistStamped message."""
    header: Optional[Dict[str, Any]] = None
    twist: Twist = None
    
    def __post_init__(self):
        if self.twist is None:
            self.twist = Twist()
        if self.header is None:
            self.header = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}


@dataclass
class TransformStamped:
    """Simplified geometry_msgs.msg.TransformStamped message."""
    header: Optional[Dict[str, Any]] = None
    child_frame_id: str = ""
    transform: Transform = None
    
    def __post_init__(self):
        if self.transform is None:
            self.transform = Transform()
        if self.header is None:
            self.header = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}


@dataclass
class PoseArray:
    """Simplified geometry_msgs.msg.PoseArray message."""
    header: Optional[Dict[str, Any]] = None
    poses: List[Pose] = field(default_factory=list)
    
    def __post_init__(self):
        if self.header is None:
            self.header = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}


# Export all types
__all__ = [
    'Vector3', 'Point', 'Quaternion', 'Pose', 'Twist', 'Transform', 'Accel', 'Wrench',
    'Point32', 'Pose2D', 'PoseStamped', 'TwistStamped', 'TransformStamped', 'PoseArray'
]
