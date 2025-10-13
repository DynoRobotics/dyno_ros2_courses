#!/usr/bin/env python3
"""
Example: Using Package-Based Generated Types with ros2_zenoh_python

This example demonstrates how to use the package-based generated simplified types
with the ros2_zenoh_python package for ROS 2 communication.
"""

import sys
import time
import argparse
from ros2_zenoh_python import Node, Publisher, Subscriber

# Import the generated simplified types from different packages
try:
    from geometry_msgs_simplified import Twist, Vector3, Pose, Point, Quaternion
    from std_msgs_simplified import Header, String, Bool
    from nav_msgs_simplified import Path, Odometry
    GENERATED_TYPES_AVAILABLE = True
except ImportError:
    GENERATED_TYPES_AVAILABLE = False
    print("Warning: Generated simplified types not available. Using built-in types.")
    from ros2_zenoh_python import Twist, Vector3, Pose, Point, Quaternion

# Also import ROS 2 types for comparison
try:
    from geometry_msgs.msg import Twist as ROS2Twist, Vector3 as ROS2Vector3
    from std_msgs.msg import Header as ROS2Header, String as ROS2String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS 2 message definitions not available")


def twist_callback(msg):
    """Callback function to handle Twist messages."""
    print(f"📨 Received Twist message:")
    print(f"   Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}")
    print(f"   Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")
    print("-" * 50)


def string_callback(msg):
    """Callback function to handle String messages."""
    print(f"📨 Received String message: '{msg.data}'")
    print("-" * 50)


def main():
    parser = argparse.ArgumentParser(description='Example using package-based generated simplified types')
    parser.add_argument('--twist-topic', default='/turtle1/cmd_vel', 
                       help='Twist topic name')
    parser.add_argument('--string-topic', default='/chatter', 
                       help='String topic name')
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
    parser.add_argument('--use-generated', action='store_true',
                       help='Use generated simplified types instead of ROS 2')
    parser.add_argument('--node-name', default='package_types_example',
                       help='Node name')
    
    args = parser.parse_args()
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{args.zenoh_endpoint}"]')
    
    try:
        # Create a node
        with Node(args.node_name, zenoh_config=config) as node:
            logger = node.get_logger()
            logger.info(f"🚀 Node '{node.node_name}' created")
            
            # Choose message types based on availability and preference
            if args.use_generated and GENERATED_TYPES_AVAILABLE:
                # Use generated simplified types
                twist_msg_type = Twist
                string_msg_type = String
                logger.info(f"📤 Using generated simplified types from multiple packages")
                
                # Create publishers
                twist_pub = node.create_publisher(twist_msg_type, args.twist_topic)
                logger.info(f"📤 Twist publisher created for topic '{args.twist_topic}'")
                
                string_pub = node.create_publisher(string_msg_type, args.string_topic)
                logger.info(f"📤 String publisher created for topic '{args.string_topic}'")
                
                # Create subscribers
                twist_sub = node.create_subscription(twist_msg_type, args.twist_topic, twist_callback)
                logger.info(f"📥 Twist subscriber created for topic '{args.twist_topic}'")
                
                string_sub = node.create_subscription(string_msg_type, args.string_topic, string_callback)
                logger.info(f"📥 String subscriber created for topic '{args.string_topic}'")
                
                logger.info(f"🔄 Publishing messages for {args.duration} seconds...")
                logger.info(f"📊 Twist: Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"📊 String: 'Hello from package-based types!'")
                logger.info(f"⏱️  Rate: {args.rate} Hz")
                
                start_time = time.time()
                message_count = 0
                while time.time() - start_time < args.duration:
                    # Create Twist message
                    twist_msg = Twist(
                        linear=Vector3(x=args.linear_x, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=args.angular_z)
                    )
                    
                    # Create String message
                    string_msg = String(data=f"Hello from package-based types! Message #{message_count}")
                    
                    # Publish messages
                    twist_pub.publish(twist_msg)
                    string_pub.publish(string_msg)
                    
                    logger.debug(f"📤 Published: Twist(linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}), String('{string_msg.data}')")
                    
                    message_count += 1
                    time.sleep(1.0 / args.rate)
                
                # Send stop command
                logger.info("🛑 Sending stop command...")
                stop_twist = Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                twist_pub.publish(stop_twist)
                
            elif ROS2_AVAILABLE:
                # Use ROS 2 message types
                twist_msg_type = ROS2Twist
                string_msg_type = ROS2String
                logger.info(f"📤 Using ROS 2 message types")
                
                # Create publishers
                twist_pub = node.create_publisher(twist_msg_type, args.twist_topic)
                logger.info(f"📤 Twist publisher created for topic '{args.twist_topic}'")
                
                string_pub = node.create_publisher(string_msg_type, args.string_topic)
                logger.info(f"📤 String publisher created for topic '{args.string_topic}'")
                
                # Create subscribers
                twist_sub = node.create_subscription(twist_msg_type, args.twist_topic, twist_callback)
                logger.info(f"📥 Twist subscriber created for topic '{args.twist_topic}'")
                
                string_sub = node.create_subscription(string_msg_type, args.string_topic, string_callback)
                logger.info(f"📥 String subscriber created for topic '{args.string_topic}'")
                
                logger.info(f"🔄 Publishing ROS 2 messages for {args.duration} seconds...")
                logger.info(f"📊 Twist: Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"📊 String: 'Hello from ROS 2 types!'")
                logger.info(f"⏱️  Rate: {args.rate} Hz")
                
                start_time = time.time()
                message_count = 0
                while time.time() - start_time < args.duration:
                    # Create ROS 2 Twist message
                    twist_msg = ROS2Twist()
                    twist_msg.linear.x = args.linear_x
                    twist_msg.linear.y = 0.0
                    twist_msg.linear.z = 0.0
                    twist_msg.angular.x = 0.0
                    twist_msg.angular.y = 0.0
                    twist_msg.angular.z = args.angular_z
                    
                    # Create ROS 2 String message
                    string_msg = ROS2String()
                    string_msg.data = f"Hello from ROS 2 types! Message #{message_count}"
                    
                    # Publish messages
                    twist_pub.publish(twist_msg)
                    string_pub.publish(string_msg)
                    
                    logger.debug(f"📤 Published: Twist(linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}), String('{string_msg.data}')")
                    
                    message_count += 1
                    time.sleep(1.0 / args.rate)
                
                # Send stop command
                logger.info("🛑 Sending stop command...")
                stop_twist = ROS2Twist()
                stop_twist.linear.x = 0.0
                stop_twist.linear.y = 0.0
                stop_twist.linear.z = 0.0
                stop_twist.angular.x = 0.0
                stop_twist.angular.y = 0.0
                stop_twist.angular.z = 0.0
                twist_pub.publish(stop_twist)
                
            else:
                # Use built-in simplified types
                twist_msg_type = Twist
                logger.info(f"📤 Using built-in simplified types")
                
                # Create publisher
                twist_pub = node.create_publisher(twist_msg_type, args.twist_topic)
                logger.info(f"📤 Twist publisher created for topic '{args.twist_topic}'")
                
                # Create subscriber
                twist_sub = node.create_subscription(twist_msg_type, args.twist_topic, twist_callback)
                logger.info(f"📥 Twist subscriber created for topic '{args.twist_topic}'")
                
                logger.info(f"🔄 Publishing built-in simplified Twist messages for {args.duration} seconds...")
                logger.info(f"📊 Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"⏱️  Rate: {args.rate} Hz")
                
                start_time = time.time()
                while time.time() - start_time < args.duration:
                    # Create built-in simplified message
                    twist_msg = Twist(
                        linear=Vector3(x=args.linear_x, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=args.angular_z)
                    )
                    
                    # Publish message
                    twist_pub.publish(twist_msg)
                    logger.debug(f"📤 Published: Twist(linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z})")
                    
                    time.sleep(1.0 / args.rate)
                
                # Send stop command
                logger.info("🛑 Sending stop command...")
                stop_twist = Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                twist_pub.publish(stop_twist)
            
            logger.info("✅ Package-based types example completed")
            
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    
    print("🎉 Package-based types example finished")


if __name__ == '__main__':
    main()
