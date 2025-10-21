#!/usr/bin/env python3
"""
Example ROS 2 subscriber using ros2_zenoh_python with CDR types

This example subscribes to Twist messages that are compatible with ROS 2 nodes
using rmw_zenoh_cpp (the Zenoh RMW implementation).

Uses:
- ros2_zenoh_python.Subscriber for ROS 2 Zenoh interop
- ros2_interfaces_py for CDR types with serialize()/deserialize()
- Python's standard logging module (publishes to /rosout)
"""

import sys
import logging
from pathlib import Path

# Add packages to path
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "tools" / "unified_output" / "python"))

# Import directly from message files to avoid circular import in __init__.py
from ros2_interfaces_py.geometry_msgs.msg.twist import Twist
from ros2_zenoh_python import Node
from ros2_zenoh_python.logger import setup_logging
import zenoh

# Get logger for this module (standard Python logging)
logger = logging.getLogger(__name__)


def twist_callback(msg: Twist):
    """
    Callback function that receives deserialized Twist messages.
    
    The Subscriber automatically detects that Twist has a deserialize() method
    and uses it to convert CDR bytes to the message object.
    """
    print(f"Received: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")


def main():
    # Setup basic logging (will be enhanced with /rosout when Node is created)
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] [%(name)s] %(message)s')
    
    logger.info("Starting ROS 2 Zenoh Subscriber")
    
    # Configuration
    topic = '/turtle1/safe_cmd_vel'
    zenoh_endpoint = 'tcp/0.0.0.0:7447'
    node_name = 'zenoh_subscriber'
    
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
            
            # Create subscriber using node
            subscriber = node.create_subscription(Twist, topic, twist_callback)
            logger.info(f"Subscriber created for topic '{topic}'")
            logger.info(f"DDS Key: {subscriber.dds_key}")
            logger.info("Waiting for messages (Press Ctrl+C to stop)...")
            
            # Keep the subscriber alive
            subscriber.spin()
            
    except KeyboardInterrupt:
        logger.info("Subscriber stopped")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
