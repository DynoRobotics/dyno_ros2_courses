#!/usr/bin/env python3
"""
Working Twist Subscriber Example

This example demonstrates how to use the fixed ros2_zenoh_python package
to subscribe to Twist messages using the corrected message structure.
"""

import sys
import time
from ros2_zenoh_python import Node, Twist, Vector3


def twist_callback(msg):
    """Callback function for received Twist messages."""
    print(f"🔔 Received Twist message:")
    print(f"   Linear:  x={msg.linear.x:.6f}, y={msg.linear.y:.6f}, z={msg.linear.z:.6f}")
    print(f"   Angular: x={msg.angular.x:.6f}, y={msg.angular.y:.6f}, z={msg.angular.z:.6f}")
    
    # Calculate speed and turn rate
    import math
    linear_speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
    angular_speed = math.sqrt(msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)
    
    print(f"   Speed: {linear_speed:.3f} m/s, Turn rate: {angular_speed:.3f} rad/s")
    print("---")


def main():
    # Simple configuration
    topic = 'turtle1/cmd_vel'  # Use valid Zenoh key expression (no leading/trailing slashes)
    duration = 30  # seconds
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        topic = sys.argv[1]
    if len(sys.argv) > 2:
        duration = int(sys.argv[2])
    
    try:
        # Create a node
        with Node("zenoh_twist_subscriber") as node:
            logger = node.get_logger()
            logger.info(f"Node '{node.node_name}' created")
            
            # Create subscriber using fixed message types
            sub = node.create_subscriber(Twist, topic, twist_callback)
            logger.info(f"Subscriber created for topic '{topic}'")
            
            logger.info(f"📥 Listening for Twist messages on topic '{topic}' for {duration} seconds...")
            logger.info("Press Ctrl+C to stop early")
            
            # Listen for messages
            start_time = time.time()
            message_count = 0
            
            while time.time() - start_time < duration:
                try:
                    # The callback will be triggered automatically when messages arrive
                    time.sleep(0.1)  # Small sleep to prevent busy waiting
                except KeyboardInterrupt:
                    logger.info("🛑 Interrupted by user")
                    break
            
            logger.info(f"✅ Subscriber example completed! Received {message_count} messages.")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
