from dataclasses import dataclass, field
from typing import List

# Common ROS 2 message types converted to simplified Python dataclasses

@dataclass
class TransformStamped:
    """Simplified geometry_msgs.msg.TransformStamped message."""
    header: std_msgs/Header = 0
    child_frame_id: str = ""
    transform: Transform = Transform()

@dataclass
class QuaternionStamped:
    """Simplified geometry_msgs.msg.QuaternionStamped message."""
    header: std_msgs/Header = 0
    quaternion: Quaternion = Quaternion()

@dataclass
class PoseWithCovariance:
    """Simplified geometry_msgs.msg.PoseWithCovariance message."""
    pose: Pose = Pose()
    covariance: List[float] = field(default_factory=list)

@dataclass
class AccelWithCovariance:
    """Simplified geometry_msgs.msg.AccelWithCovariance message."""
    accel: Accel = Accel()
    covariance: List[float] = field(default_factory=list)

@dataclass
class TwistWithCovarianceStamped:
    """Simplified geometry_msgs.msg.TwistWithCovarianceStamped message."""
    header: std_msgs/Header = 0
    twist: TwistWithCovariance = 0

@dataclass
class PoseWithCovarianceStamped:
    """Simplified geometry_msgs.msg.PoseWithCovarianceStamped message."""
    header: std_msgs/Header = 0
    pose: PoseWithCovariance = 0

@dataclass
class Pose:
    """Simplified geometry_msgs.msg.Pose message."""
    position: Point = Point()
    orientation: Quaternion = Quaternion()

@dataclass
class Accel:
    """Simplified geometry_msgs.msg.Accel message."""
    linear: Vector3 = Vector3()
    angular: Vector3 = Vector3()

@dataclass
class Vector3:
    """Simplified geometry_msgs.msg.Vector3 message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class PolygonStamped:
    """Simplified geometry_msgs.msg.PolygonStamped message."""
    header: std_msgs/Header = 0
    polygon: Polygon = 0

@dataclass
class AccelStamped:
    """Simplified geometry_msgs.msg.AccelStamped message."""
    header: std_msgs/Header = 0
    accel: Accel = Accel()

@dataclass
class Inertia:
    """Simplified geometry_msgs.msg.Inertia message."""
    m: float = 0.0
    com: geometry_msgs/Vector3 = 0
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0

@dataclass
class PolygonInstance:
    """Simplified geometry_msgs.msg.PolygonInstance message."""
    polygon: geometry_msgs/Polygon = 0
    id: int = 0

@dataclass
class PoseStamped:
    """Simplified geometry_msgs.msg.PoseStamped message."""
    header: std_msgs/Header = 0
    pose: Pose = Pose()

@dataclass
class Point:
    """Simplified geometry_msgs.msg.Point message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class TwistStamped:
    """Simplified geometry_msgs.msg.TwistStamped message."""
    header: std_msgs/Header = 0
    twist: Twist = 0

@dataclass
class Vector3Stamped:
    """Simplified geometry_msgs.msg.Vector3Stamped message."""
    header: std_msgs/Header = 0
    vector: Vector3 = Vector3()

@dataclass
class VelocityStamped:
    """Simplified geometry_msgs.msg.VelocityStamped message."""
    header: std_msgs/Header = 0
    body_frame_id: str = ""
    reference_frame_id: str = ""
    velocity: Twist = 0

@dataclass
class Point32:
    """Simplified geometry_msgs.msg.Point32 message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Polygon:
    """Simplified geometry_msgs.msg.Polygon message."""
    points: List[Point32] = field(default_factory=list)

@dataclass
class WrenchStamped:
    """Simplified geometry_msgs.msg.WrenchStamped message."""
    header: std_msgs/Header = 0
    wrench: Wrench = Wrench()

@dataclass
class Twist:
    """Simplified geometry_msgs.msg.Twist message."""
    linear: Vector3 = Vector3()
    angular: Vector3 = Vector3()

@dataclass
class AccelWithCovarianceStamped:
    """Simplified geometry_msgs.msg.AccelWithCovarianceStamped message."""
    header: std_msgs/Header = 0
    accel: AccelWithCovariance = 0

@dataclass
class TwistWithCovariance:
    """Simplified geometry_msgs.msg.TwistWithCovariance message."""
    twist: Twist = 0
    covariance: List[float] = field(default_factory=list)

@dataclass
class Transform:
    """Simplified geometry_msgs.msg.Transform message."""
    translation: Vector3 = Vector3()
    rotation: Quaternion = Quaternion()

@dataclass
class Quaternion:
    """Simplified geometry_msgs.msg.Quaternion message."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

@dataclass
class InertiaStamped:
    """Simplified geometry_msgs.msg.InertiaStamped message."""
    header: std_msgs/Header = 0
    inertia: Inertia = 0

@dataclass
class PolygonInstanceStamped:
    """Simplified geometry_msgs.msg.PolygonInstanceStamped message."""
    header: std_msgs/Header = 0
    polygon: geometry_msgs/PolygonInstance = 0

@dataclass
class PointStamped:
    """Simplified geometry_msgs.msg.PointStamped message."""
    header: std_msgs/Header = 0
    point: Point = Point()

@dataclass
class Wrench:
    """Simplified geometry_msgs.msg.Wrench message."""
    force: Vector3 = Vector3()
    torque: Vector3 = Vector3()

@dataclass
class PoseArray:
    """Simplified geometry_msgs.msg.PoseArray message."""
    header: std_msgs/Header = 0
    poses: List[Pose] = field(default_factory=list)

@dataclass
class Pose2D:
    """Simplified geometry_msgs.msg.Pose2D message."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
