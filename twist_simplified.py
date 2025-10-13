from dataclasses import dataclass, field
from typing import List

# Common ROS 2 message types converted to simplified Python dataclasses

@dataclass
class Twist:
    """Simplified geometry_msgs.msg.Twist message."""
    linear: Vector3 = Vector3()
    angular: Vector3 = Vector3()
