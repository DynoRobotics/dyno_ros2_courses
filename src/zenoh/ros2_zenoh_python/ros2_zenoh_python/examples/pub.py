#!/usr/bin/env python3
"""
Simple ROS2 Zenoh Publisher Example

Publishes Twist messages that interoperate with standard ROS2 nodes.
"""

import asyncio
import logging
import sys
from pathlib import Path

# Add packages to path
repo_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(repo_root))
sys.path.insert(0, str(repo_root / "tools" / "unified_output" / "python"))

from ros2_zenoh_python import Node
from ros2_interfaces_py.geometry_msgs.msg.twist import Twist
from ros2_interfaces_py.geometry_msgs.msg.vector3 import Vector3

logger = logging.getLogger(__name__)


async def main():
    """Publish Twist messages."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )
    
    # Create a node (automatically connects to ROS2 Zenoh router at localhost:7447)
    async with Node('zenoh_publisher') as node:
        # Create publisher
        pub = node.create_publisher(Twist, '/turtle1/cmd_vel')
        logger.info(f"Publishing to /turtle1/cmd_vel")
        
        # Publish messages
        count = 0
        while not node.shutdown_requested:
            msg = Twist(
                linear=Vector3(x=1.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.5)
            )
            pub.publish(msg)
            count += 1
            logger.info(f"Published #{count}: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
            await asyncio.sleep(1.0)


if __name__ == "__main__":
    asyncio.run(main())
