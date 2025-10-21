#!/usr/bin/env python3
"""
Example ROS 2 publisher using ros2_zenoh_python with CDR types

This example publishes Twist messages that are compatible with ROS 2 nodes
using rmw_zenoh_cpp (the Zenoh RMW implementation).

Uses:
- ros2_zenoh_python.Node for ROS 2 Zenoh interop with custom config
- ros2_interfaces_py for CDR types with serialize()/deserialize()
- Python's standard logging module (publishes to /rosout)
"""

import sys
import time
import logging
from pathlib import Path

# Add packages to path
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "tools" / "unified_output" / "python"))

# Import directly from message files to avoid circular import in __init__.py
from ros2_interfaces_py.geometry_msgs.msg.twist import Twist
from ros2_interfaces_py.geometry_msgs.msg.vector3 import Vector3
from ros2_zenoh_python import Node
from ros2_zenoh_python.logger import setup_logging
import zenoh

# Get logger for this module (standard Python logging)
logger = logging.getLogger(__name__)


def main():
    # Setup basic logging (will be enhanced with /rosout when Node is created)
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] [%(name)s] %(message)s')
    
    logger.info("Starting ROS 2 Zenoh Publisher")
    
    # Configuration
    topic = '/turtle1/safe_cmd_vel'
    zenoh_endpoint = 'tcp/0.0.0.0:7447'
    node_name = 'zenoh_publisher'
    
    # Create Zenoh configuration
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
    
    logger.info(f"Zenoh config: mode=client, endpoint={zenoh_endpoint}")
    
    try:
        # Create a node with Zenoh config
        # Logging to /rosout is automatically enabled!
        with Node(node_name, zenoh_config=config) as node:
            logger.info(f"Node '{node.node_name}' created")
            
            # Create publisher using node
            publisher = node.create_publisher(Twist, topic)
            logger.info(f"Publisher created for topic '{topic}'")
            logger.info(f"DDS Key: {publisher.dds_key}")
            
            # Create a Twist message using the unified CDR types
            # These are dataclasses with serialize()/deserialize() methods
            twist = Twist(
                linear=Vector3(x=1.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.5)
            )
            
            logger.info("Publishing Twist messages (Press Ctrl+C to stop)...")
            
            count = 0
            while True:
                # Publish the message
                # The Publisher automatically detects that twist has a serialize() method
                # and uses it instead of trying to use rclpy serialization
                publisher.publish(twist)
                count += 1
                
                print(f"Published #{count}: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
                
                # Sleep for 1 second
                time.sleep(1.0)
            
    except KeyboardInterrupt:
        logger.info(f"Publisher stopped - Total messages published: {count}")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
