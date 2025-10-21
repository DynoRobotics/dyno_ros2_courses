"""
ROS 2 Zenoh Python Package

A Python package that provides Zenoh as an alternative transport to rclpy for ROS 2.
This package enables direct Zenoh communication while maintaining ROS 2 compatibility.

Main components:
- Node: ROS 2-compatible node with async/await support
- Publisher: ROS 2-compatible publisher using Zenoh
- Subscriber: ROS 2-compatible subscriber using Zenoh
- Bundled essential messages for out-of-the-box functionality
- Liveliness tokens: ROS 2 metadata publishing via Zenoh liveliness tokens
- Pythonic logging with /rosout support

Example usage:
    import asyncio
    from ros2_zenoh_python import Node
    from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.twist import Twist
    from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.vector3 import Vector3
    
    async def main():
        async with Node('my_node') as node:
            pub = node.create_publisher(Twist, '/cmd_vel')
            
            msg = Twist(
                linear=Vector3(x=1.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.5)
            )
            pub.publish(msg)
            
            await node.spin()
    
    asyncio.run(main())
"""

from .publisher import Publisher
from .subscriber import Subscriber
from .node import Node
from .liveliness_manager import LivelinessManager
from .logger import setup_logging, RosoutHandler

__version__ = "0.1.0"
__all__ = [
    "Node",
    "Publisher", 
    "Subscriber", 
    "LivelinessManager",
    "setup_logging",
    "RosoutHandler",
]
