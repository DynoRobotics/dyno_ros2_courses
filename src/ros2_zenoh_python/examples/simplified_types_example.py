#!/usr/bin/env python3
"""
Example: Using Generated Simplified Types with ros2_zenoh_python

This example demonstrates how to use the generated simplified types
with the ros2_zenoh_python package for ROS 2 communication.
"""

import sys
import time
import argparse
from ros2_zenoh_python import Node, Publisher, Subscriber

# Import the generated simplified types
try:
    from geometry_msgs_simplified import Twist, Vector3, Pose, Point, Quaternion
    GENERATED_TYPES_AVAILABLE = True
except ImportError:
    GENERATED_TYPES_AVAILABLE = False
    print("Warning: Generated simplified types not available. Using built-in types.")
    from ros2_zenoh_python import Twist, Vector3, Pose, Point, Quaternion

# Also import ROS 2 types for comparison
try:
    from geometry_msgs.msg import Twist as ROS2Twist, Vector3 as ROS2Vector3
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


def main():
    parser = argparse.ArgumentParser(description='Example using generated simplified types')
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
    parser.add_argument('--use-generated', action='store_true',
                       help='Use generated simplified types instead of ROS 2')
    parser.add_argument('--node-name', default='simplified_types_example',
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
            
            # Choose message type based on availability and preference
            if args.use_generated and GENERATED_TYPES_AVAILABLE:
                # Use generated simplified types
                msg_type = Twist
                logger.info(f"📤 Using generated simplified types")
                
                # Create publisher
                pub = node.create_publisher(msg_type, args.topic)
                logger.info(f"📤 Publisher created for topic '{args.topic}'")
                
                # Create subscriber
                sub = node.create_subscription(msg_type, args.topic, message_callback)
                logger.info(f"📥 Subscriber created for topic '{args.topic}'")
                
                logger.info(f"🔄 Publishing simplified Twist messages for {args.duration} seconds...")
                logger.info(f"📊 Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"⏱️  Rate: {args.rate} Hz")
                
                start_time = time.time()
                while time.time() - start_time < args.duration:
                    # Create simplified message
                    msg = Twist(
                        linear=Vector3(x=args.linear_x, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=args.angular_z)
                    )
                    
                    # Publish message
                    pub.publish(msg)
                    logger.debug(f"📤 Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                    
                    # Wait for next publish cycle
                    time.sleep(1.0 / args.rate)
                
                # Send stop command
                logger.info("🛑 Sending stop command...")
                stop_msg = Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                pub.publish(stop_msg)
                
            elif ROS2_AVAILABLE:
                # Use ROS 2 message types
                msg_type = ROS2Twist
                logger.info(f"📤 Using ROS 2 message types")
                
                # Create publisher
                pub = node.create_publisher(msg_type, args.topic)
                logger.info(f"📤 Publisher created for topic '{args.topic}'")
                
                # Create subscriber
                sub = node.create_subscription(msg_type, args.topic, message_callback)
                logger.info(f"📥 Subscriber created for topic '{args.topic}'")
                
                logger.info(f"🔄 Publishing ROS 2 Twist messages for {args.duration} seconds...")
                logger.info(f"📊 Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"⏱️  Rate: {args.rate} Hz")
                
                start_time = time.time()
                while time.time() - start_time < args.duration:
                    # Create ROS 2 message
                    msg = ROS2Twist()
                    msg.linear.x = args.linear_x
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                    msg.angular.x = 0.0
                    msg.angular.y = 0.0
                    msg.angular.z = args.angular_z
                    
                    # Publish message
                    pub.publish(msg)
                    logger.debug(f"📤 Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                    
                    # Wait for next publish cycle
                    time.sleep(1.0 / args.rate)
                
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
                
            else:
                # Use built-in simplified types
                msg_type = Twist
                logger.info(f"📤 Using built-in simplified types")
                
                # Create publisher
                pub = node.create_publisher(msg_type, args.topic)
                logger.info(f"📤 Publisher created for topic '{args.topic}'")
                
                # Create subscriber
                sub = node.create_subscription(msg_type, args.topic, message_callback)
                logger.info(f"📥 Subscriber created for topic '{args.topic}'")
                
                logger.info(f"🔄 Publishing built-in simplified Twist messages for {args.duration} seconds...")
                logger.info(f"📊 Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"⏱️  Rate: {args.rate} Hz")
                
                start_time = time.time()
                while time.time() - start_time < args.duration:
                    # Create built-in simplified message
                    msg = Twist(
                        linear=Vector3(x=args.linear_x, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=args.angular_z)
                    )
                    
                    # Publish message
                    pub.publish(msg)
                    logger.debug(f"📤 Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                    
                    # Wait for next publish cycle
                    time.sleep(1.0 / args.rate)
                
                # Send stop command
                logger.info("🛑 Sending stop command...")
                stop_msg = Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                pub.publish(stop_msg)
            
            logger.info("✅ Example completed")
            
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    
    print("🎉 Simplified types example finished")


if __name__ == '__main__':
    main()
