#!/usr/bin/env python3
"""
rclpy-like Example using ros2_zenoh_python

This example demonstrates the rclpy-like interface with simplified message types.
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
    parser = argparse.ArgumentParser(description='ROS 2 Zenoh rclpy-like Example')
    parser.add_argument('--mode', choices=['pub', 'sub', 'both'], default='both',
                       help='Mode: pub (publisher only), sub (subscriber only), both (both)')
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
    
    args = parser.parse_args()
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{args.zenoh_endpoint}"]')
    
    try:
        # Create a single node that manages the Zenoh session
        # This matches rclpy.create_node() interface
        with Node("zenoh_example_node", zenoh_config=config) as node:
            logger = node.get_logger()
            logger.info(f"Node '{node.node_name}' created with shared Zenoh session")
            
            publishers = []
            subscribers = []
            
            # Create publishers if requested
            # This matches rclpy node.create_publisher() interface
            if args.mode in ['pub', 'both']:
                if ROS2_AVAILABLE:
                    pub = node.create_publisher(ROS2Twist, args.topic)
                    logger.info(f"Publisher created for topic '{args.topic}' (ROS 2 mode)")
                else:
                    pub = node.create_publisher(Twist, args.topic)
                    logger.info(f"Publisher created for topic '{args.topic}' (simplified mode)")
                publishers.append(pub)
            
            # Create subscribers if requested
            # This matches rclpy node.create_subscription() interface
            if args.mode in ['sub', 'both']:
                if ROS2_AVAILABLE:
                    sub = node.create_subscription(ROS2Twist, args.topic, message_callback)
                    logger.info(f"Subscriber created for topic '{args.topic}' (ROS 2 mode)")
                else:
                    sub = node.create_subscription(Twist, args.topic, message_callback)
                    logger.info(f"Subscriber created for topic '{args.topic}' (simplified mode)")
                subscribers.append(sub)
            
            logger.info(f"Running for {args.duration} seconds...")
            logger.info("Press Ctrl+C to stop early")
            
            start_time = time.time()
            while time.time() - start_time < args.duration:
                # Publish messages if we have publishers
                if publishers:
                    for pub in publishers:
                        if ROS2_AVAILABLE:
                            # Create ROS 2 message
                            ros2_msg = ROS2Twist()
                            ros2_msg.linear.x = args.linear_x
                            ros2_msg.angular.z = args.angular_z
                            pub.publish(ros2_msg)
                        else:
                            # Create simplified message
                            simple_msg = Twist(
                                linear=Vector3(x=args.linear_x, y=0.0, z=0.0),
                                angular=Vector3(x=0.0, y=0.0, z=args.angular_z)
                            )
                            pub.publish(simple_msg)
                
                # Wait for next cycle
                time.sleep(1.0 / args.rate)
            
            # Send stop command
            if publishers:
                logger.info("Sending stop command...")
                for pub in publishers:
                    if ROS2_AVAILABLE:
                        stop_msg = ROS2Twist()
                        stop_msg.linear.x = 0.0
                        stop_msg.angular.z = 0.0
                        pub.publish(stop_msg)
                    else:
                        stop_msg = Twist(
                            linear=Vector3(x=0.0, y=0.0, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=0.0)
                        )
                        pub.publish(stop_msg)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    print("Example finished")


if __name__ == '__main__':
    main()
