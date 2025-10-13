#!/usr/bin/env python3
"""
Complete Publisher-Subscriber Example using ros2_zenoh_python

This example demonstrates a complete publisher-subscriber workflow
using the rclpy-like interface with both ROS 2 and simplified message types.
"""

import sys
import time
import argparse
import threading
from ros2_zenoh_python import Node, Twist, Vector3, to_simple, to_ros2

try:
    from geometry_msgs.msg import Twist as ROS2Twist
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS 2 message definitions not available")


def message_callback(msg):
    """Callback function to handle received messages."""
    print(f"📨 Received message:")
    print(f"   Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}")
    print(f"   Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")
    print("-" * 50)


def publisher_thread(node, topic, duration, rate, linear_x, angular_z, use_simplified):
    """Publisher thread function."""
    logger = node.get_logger()
    
    try:
        # Create publisher based on message type preference
        if use_simplified or not ROS2_AVAILABLE:
            # Use simplified message types
            pub = node.create_publisher(Twist, topic)
            logger.info(f"📤 Publisher created for topic '{topic}' (simplified message types)")
            
            start_time = time.time()
            while time.time() - start_time < duration:
                # Create simplified message
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
            
        else:
            # Use ROS 2 message types
            pub = node.create_publisher(ROS2Twist, topic)
            logger.info(f"📤 Publisher created for topic '{topic}' (ROS 2 message types)")
            
            start_time = time.time()
            while time.time() - start_time < duration:
                # Create ROS 2 message
                msg = ROS2Twist()
                msg.linear.x = linear_x
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = angular_z
                
                # Publish message
                pub.publish(msg)
                logger.debug(f"📤 Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                
                # Wait for next publish cycle
                time.sleep(1.0 / rate)
            
            # Send stop command
            logger.info("🛑 Sending stop command...")
            stop_msg = ROS2Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            pub.publish(stop_msg)
            
    except Exception as e:
        logger.error(f"Publisher error: {e}")


def main():
    parser = argparse.ArgumentParser(description='ROS 2 Zenoh Complete Example')
    parser.add_argument('--topic', default='/turtle1/cmd_vel', 
                       help='Topic name')
    parser.add_argument('--duration', type=int, default=10,
                       help='Duration to run (seconds)')
    parser.add_argument('--rate', type=float, default=1.0,
                       help='Publishing rate (Hz)')
    parser.add_argument('--linear-x', type=float, default=1.0,
                       help='Linear velocity in x direction')
    parser.add_argument('--angular-z', type=float, default=0.5,
                       help='Angular velocity around z axis')
    parser.add_argument('--zenoh-endpoint', default='tcp/172.18.0.2:7447',
                       help='Zenoh endpoint to connect to')
    parser.add_argument('--use-simplified', action='store_true',
                       help='Use simplified message types instead of ROS 2')
    parser.add_argument('--node-name', default='zenoh_example',
                       help='Node name')
    
    args = parser.parse_args()
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{args.zenoh_endpoint}"]')
    
    try:
        # Create a single node that manages the Zenoh session
        with Node(args.node_name, zenoh_config=config) as node:
            logger = node.get_logger()
            logger.info(f"🚀 Node '{node.node_name}' created with shared Zenoh session")
            
            # Create subscription based on message type preference
            if args.use_simplified or not ROS2_AVAILABLE:
                # Use simplified message types
                sub = node.create_subscription(Twist, args.topic, message_callback)
                logger.info(f"📥 Subscriber created for topic '{args.topic}' (simplified message types)")
            else:
                # Use ROS 2 message types
                sub = node.create_subscription(ROS2Twist, args.topic, message_callback)
                logger.info(f"📥 Subscriber created for topic '{args.topic}' (ROS 2 message types)")
            
            # Start publisher in a separate thread
            pub_thread = threading.Thread(
                target=publisher_thread,
                args=(node, args.topic, args.duration, args.rate, 
                      args.linear_x, args.angular_z, args.use_simplified)
            )
            pub_thread.daemon = True
            pub_thread.start()
            
            logger.info(f"🔄 Running publisher-subscriber for {args.duration} seconds...")
            logger.info(f"📊 Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
            logger.info(f"⏱️  Rate: {args.rate} Hz")
            logger.info("Press Ctrl+C to stop early")
            
            # Keep running to receive messages
            time.sleep(args.duration)
            
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
