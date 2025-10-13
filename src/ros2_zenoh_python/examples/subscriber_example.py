#!/usr/bin/env python3
"""
Subscriber Example using ros2_zenoh_python

This example demonstrates how to use the ros2_zenoh_python package
to subscribe to messages using the rclpy-like interface.
"""

import sys
import time
import argparse
from ros2_zenoh_python import Node, Twist, Vector3, to_simple, to_ros2

try:
    from geometry_msgs.msg import Twist as ROS2Twist
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS 2 message definitions not available")


def message_callback(msg):
    """Callback function to handle received messages."""
    print(f"Received message:")
    print(f"  Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}")
    print(f"  Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")
    print("-" * 50)


def main():
    parser = argparse.ArgumentParser(description='ROS 2 Zenoh Subscriber Example')
    parser.add_argument('--topic', default='/turtle1/cmd_vel', 
                       help='Topic name to subscribe to')
    parser.add_argument('--duration', type=int, default=30,
                       help='Duration to listen (seconds)')
    parser.add_argument('--zenoh-endpoint', default='tcp/172.18.0.2:7447',
                       help='Zenoh endpoint to connect to')
    parser.add_argument('--use-simplified', action='store_true',
                       help='Use simplified message types instead of ROS 2')
    parser.add_argument('--node-name', default='zenoh_subscriber',
                       help='Node name')
    
    args = parser.parse_args()
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{args.zenoh_endpoint}"]')
    
    try:
        # Create a node (matches rclpy.create_node())
        with Node(args.node_name, zenoh_config=config) as node:
            logger = node.get_logger()
            logger.info(f"Node '{node.node_name}' created")
            
            # Create subscription based on message type preference
            if args.use_simplified or not ROS2_AVAILABLE:
                # Use simplified message types
                sub = node.create_subscription(Twist, args.topic, message_callback)
                logger.info(f"Subscriber created for topic '{args.topic}' (simplified message types)")
            else:
                # Use ROS 2 message types
                sub = node.create_subscription(ROS2Twist, args.topic, message_callback)
                logger.info(f"Subscriber created for topic '{args.topic}' (ROS 2 message types)")
            
            logger.info(f"Listening for messages on topic '{args.topic}' for {args.duration} seconds...")
            logger.info("Press Ctrl+C to stop early")
            
            # Keep running to receive messages
            time.sleep(args.duration)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    print("Subscriber example completed")


if __name__ == '__main__':
    main()
