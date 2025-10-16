#!/usr/bin/env python3
"""
Subscriber Example using ros2_zenoh_python

This example demonstrates how to use the ros2_zenoh_python package
to subscribe to messages using the generated ROS 2 message types.
"""

import sys
import time
from ros2_zenoh_python import Node
from ros2_interfaces_python.geometry_msgs.msg.geometry_msgs import Twist


def message_callback(msg):
    """Callback function to handle received messages."""
    print(f"📨 Received message:")
    print(f"   Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}")
    print(f"   Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")
    print("-" * 50)


def main():
    # Simple configuration
    topic = '/turtle1/cmd_vel'
    duration = 30  # seconds
    zenoh_endpoint = 'tcp/172.18.0.2:7447'
    node_name = 'zenoh_subscriber'
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
    
    try:
        # Create a node
        with Node(node_name, zenoh_config=config) as node:
            logger = node.get_logger()
            logger.info(f"Node '{node.node_name}' created")
            
            # Create subscription using generated message types
            sub = node.create_subscription(Twist, topic, message_callback)
            logger.info(f"Subscriber created for topic '{topic}'")
            
            logger.info(f"Listening for messages on topic '{topic}' for {duration} seconds...")
            logger.info("Press Ctrl+C to stop early")
            
            # Keep running to receive messages
            time.sleep(duration)
            
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    
    print("✅ Subscriber example completed")


if __name__ == '__main__':
    main()
