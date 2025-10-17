#!/usr/bin/env python3
"""
Working Twist Publisher Example

This example demonstrates how to use the fixed ros2_zenoh_python package
to publish Twist messages using the corrected message structure.
"""

import sys
import time
from ros2_zenoh_python import Node, Twist, Vector3


def main():
    # Simple configuration
    topic = 'turtle1/cmd_vel'  # Use valid Zenoh key expression (no leading/trailing slashes)
    duration = 10  # seconds
    rate = 1.0  # Hz
    linear_x = 1.0
    angular_z = 0.5
    
    try:
        # Create a node
        with Node("zenoh_twist_publisher") as node:
            logger = node.get_logger()
            logger.info(f"Node '{node.node_name}' created")
            
            # Create publisher using fixed message types
            pub = node.create_publisher(Twist, topic)
            logger.info(f"Publisher created for topic '{topic}'")
            
            logger.info(f"Publishing Twist messages for {duration} seconds...")
            logger.info(f"Linear X: {linear_x}, Angular Z: {angular_z}")
            logger.info(f"Rate: {rate} Hz")
            
            start_time = time.time()
            message_count = 0
            
            while time.time() - start_time < duration:
                # Create message using fixed types with proper Vector3 substructs
                msg = Twist(
                    linear=Vector3(x=linear_x, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=angular_z)
                )
                
                # Publish the message
                pub.publish(msg)
                message_count += 1
                
                logger.info(f"Published Twist[{message_count}]: linear=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) angular=({msg.angular.x:.2f},{msg.angular.y:.2f},{msg.angular.z:.2f})")
                
                # Wait for next message
                time.sleep(1.0 / rate)
            
            logger.info(f"✅ Published {message_count} Twist messages successfully!")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
