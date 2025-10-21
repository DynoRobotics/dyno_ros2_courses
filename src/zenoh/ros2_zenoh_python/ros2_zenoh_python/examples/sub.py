#!/usr/bin/env python3
"""
Simple ROS2 Zenoh Subscriber Example

Subscribes to Twist messages that interoperate with standard ROS2 nodes.
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

logger = logging.getLogger(__name__)


async def twist_callback(msg: Twist):
    """Callback for received Twist messages."""
    logger.info(f"Received: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")


async def main():
    """Subscribe to Twist messages."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )
    
    # Create a node (automatically connects to ROS2 Zenoh router at localhost:7447)
    async with Node('zenoh_subscriber') as node:
        # Create subscriber with async callback
        sub = node.create_subscription(Twist, '/turtle1/cmd_vel', twist_callback)
        logger.info(f"Subscribed to /turtle1/cmd_vel")
        
        # Spin forever (handles Ctrl+C automatically)
        await node.spin()


if __name__ == "__main__":
    asyncio.run(main())
