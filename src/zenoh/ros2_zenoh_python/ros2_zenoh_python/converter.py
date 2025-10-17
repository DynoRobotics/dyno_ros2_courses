"""
Message Conversion Utilities

Convert between ROS 2 messages and simplified message types.
"""

from typing import Any, Type, Union
# Import from the message types module
from .message_types import Vector3, Twist, Pose, Point, Quaternion, PoseStamped, Time, Log


class MessageConverter:
    """Convert between ROS 2 messages and simplified types."""
    
    @staticmethod
    def to_simple(ros2_msg: Any) -> Any:
        """
        Convert a ROS 2 message to a simplified message type.
        
        Args:
            ros2_msg: ROS 2 message instance
            
        Returns:
            Simplified message instance
        """
        if ros2_msg is None:
            return None
        
        # Get the class name to determine conversion
        class_name = ros2_msg.__class__.__name__
        
        if class_name == 'Twist':
            return Twist(
                linear=MessageConverter.to_simple(ros2_msg.linear),
                angular=MessageConverter.to_simple(ros2_msg.angular)
            )
        elif class_name == 'Vector3':
            return Vector3(
                x=ros2_msg.x,
                y=ros2_msg.y,
                z=ros2_msg.z
            )
        elif class_name == 'Time':
            return Time(
                sec=ros2_msg.sec,
                nanosec=ros2_msg.nanosec
            )
        elif class_name == 'Log':
            return Log(
                stamp=MessageConverter.to_simple(ros2_msg.stamp),
                level=ros2_msg.level,
                name=ros2_msg.name,
                msg=ros2_msg.msg,
                file=ros2_msg.file,
                function=ros2_msg.function,
                line=ros2_msg.line
            )
        elif class_name == 'Pose':
            return Pose(
                position=MessageConverter.to_simple(ros2_msg.position),
                orientation=MessageConverter.to_simple(ros2_msg.orientation)
            )
        elif class_name == 'Point':
            return Point(
                x=ros2_msg.x,
                y=ros2_msg.y,
                z=ros2_msg.z
            )
        elif class_name == 'Quaternion':
            return Quaternion(
                x=ros2_msg.x,
                y=ros2_msg.y,
                z=ros2_msg.z,
                w=ros2_msg.w
            )
        elif class_name == 'PoseStamped':
            return PoseStamped(
                header={
                    'stamp': MessageConverter.to_simple(ros2_msg.header.stamp),
                    'frame_id': ros2_msg.header.frame_id
                },
                pose=MessageConverter.to_simple(ros2_msg.pose)
            )
        else:
            # Return as-is if no conversion available
            return ros2_msg
    
    @staticmethod
    def to_ros2(simple_msg: Any, ros2_type: Type) -> Any:
        """
        Convert a simplified message to a ROS 2 message type.
        
        Args:
            simple_msg: Simplified message instance
            ros2_type: ROS 2 message type class
            
        Returns:
            ROS 2 message instance
        """
        if simple_msg is None:
            return None
        
        try:
            # Try to import ROS 2 message types
            if ros2_type.__name__ == 'Twist':
                from geometry_msgs.msg import Twist as ROS2Twist, Vector3 as ROS2Vector3
                return ROS2Twist(
                    linear=MessageConverter.to_ros2(simple_msg.linear, ROS2Vector3),
                    angular=MessageConverter.to_ros2(simple_msg.angular, ROS2Vector3)
                )
            elif ros2_type.__name__ == 'Vector3':
                from geometry_msgs.msg import Vector3 as ROS2Vector3
                return ROS2Vector3(
                    x=simple_msg.x,
                    y=simple_msg.y,
                    z=simple_msg.z
                )
            elif ros2_type.__name__ == 'Time':
                from builtin_interfaces.msg import Time as ROS2Time
                return ROS2Time(
                    sec=simple_msg.sec,
                    nanosec=simple_msg.nanosec
                )
            elif ros2_type.__name__ == 'Log':
                from rcl_interfaces.msg import Log as ROS2Log
                from builtin_interfaces.msg import Time as ROS2Time
                return ROS2Log(
                    stamp=MessageConverter.to_ros2(simple_msg.stamp, ROS2Time),
                    level=simple_msg.level,
                    name=simple_msg.name,
                    msg=simple_msg.msg,
                    file=simple_msg.file,
                    function=simple_msg.function,
                    line=simple_msg.line
                )
            else:
                # Fallback: try to create ROS 2 message with attributes
                ros2_msg = ros2_type()
                for attr_name in dir(simple_msg):
                    if not attr_name.startswith('_'):
                        attr_value = getattr(simple_msg, attr_name)
                        if hasattr(ros2_msg, attr_name):
                            setattr(ros2_msg, attr_name, attr_value)
                return ros2_msg
                
        except ImportError:
            raise ImportError(f"ROS 2 message type {ros2_type} not available")
    
    @staticmethod
    def get_simple_type(ros2_type: Type) -> Type:
        """
        Get the corresponding simplified type for a ROS 2 type.
        
        Args:
            ros2_type: ROS 2 message type class
            
        Returns:
            Simplified message type class
        """
        type_mapping = {
            'Twist': Twist,
            'Vector3': Vector3,
            'Time': Time,
            'Log': Log,
            'Pose': Pose,
            'Point': Point,
            'Quaternion': Quaternion,
            'PoseStamped': PoseStamped,
        }
        
        return type_mapping.get(ros2_type.__name__, ros2_type)


# Convenience functions
def to_simple(ros2_msg: Any) -> Any:
    """Convert ROS 2 message to simplified type."""
    return MessageConverter.to_simple(ros2_msg)


def to_ros2(simple_msg: Any, ros2_type: Type) -> Any:
    """Convert simplified message to ROS 2 type."""
    return MessageConverter.to_ros2(simple_msg, ros2_type)


def get_simple_type(ros2_type: Type) -> Type:
    """Get simplified type for ROS 2 type."""
    return MessageConverter.get_simple_type(ros2_type)
