#!/usr/bin/env python3
"""
Complete Publisher-Subscriber Example using ros2_zenoh_python

This example demonstrates a complete publisher-subscriber workflow
using the generated ROS 2 message types from ros2_interfaces_python.
"""

import sys
import time
import threading
from ros2_zenoh_python import Node
from ros2_interfaces_python.geometry_msgs.msg.geometry_msgs import Twist, Vector3


def message_callback(msg):
    """Callback function to handle received messages."""
    print(f"📨 Received message:")
    print(f"   Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}")
    print(f"   Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")
    print("-" * 50)


def publisher_thread(node, topic, duration, rate, linear_x, angular_z):
    """Publisher thread function."""
    logger = node.get_logger()
    
    try:
        # Create publisher using generated message types
        pub = node.create_publisher(Twist, topic)
        logger.info(f"📤 Publisher created for topic '{topic}'")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            # Create message using generated types
            msg = Twist(
                linear=Vector3(x=linear_x, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=angular_z)
            )
            
            # Publish message
            pub.publish(msg)
            logger.debug(f"📤 Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
            
            # Wait for next publish cycle
            time.sleep(1.0 / rate)
        
        # Send stop command
        logger.info("🛑 Sending stop command...")
        stop_msg = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        pub.publish(stop_msg)
        
    except Exception as e:
        logger.error(f"Publisher error: {e}")


def main():
    # Simple configuration - no argparse needed
    topic = '/turtle1/cmd_vel'
    duration = 10  # seconds
    rate = 1.0  # Hz
    linear_x = 1.0
    angular_z = 0.5
    zenoh_endpoint = 'tcp/172.18.0.2:7447'
    node_name = 'zenoh_example'
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
    
    try:
        # Create a single node that manages the Zenoh session
        with Node(node_name, zenoh_config=config) as node:
            logger = node.get_logger()
            logger.info(f"🚀 Node '{node.node_name}' created with shared Zenoh session")
            
            # Create subscription using generated message types
            sub = node.create_subscription(Twist, topic, message_callback)
            logger.info(f"📥 Subscriber created for topic '{topic}'")
            
            # Start publisher in a separate thread
            pub_thread = threading.Thread(
                target=publisher_thread,
                args=(node, topic, duration, rate, linear_x, angular_z)
            )
            pub_thread.daemon = True
            pub_thread.start()
            
            logger.info(f"🔄 Running publisher-subscriber for {duration} seconds...")
            logger.info(f"📊 Linear X: {linear_x}, Angular Z: {angular_z}")
            logger.info(f"⏱️  Rate: {rate} Hz")
            logger.info("Press Ctrl+C to stop early")
            
            # Keep running to receive messages
            time.sleep(duration)
            
            # Wait for publisher thread to finish
            pub_thread.join(timeout=1.0)
            
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    
    print("✅ Complete example finished")


if __name__ == '__main__':
    main()
