#!/usr/bin/env python3
"""
Publisher Example using ros2_zenoh_python

This example demonstrates how to use the ros2_zenoh_python package
to publish messages using the rclpy-like interface.
"""

import sys
import time
import argparse
from ros2_zenoh_python import Node, Twist, Vector3, to_simple, to_ros2
from ros2_interfaces_python.msg.geometry_msgs import Twist as GeometryTwist
from ros2_interfaces_python.msg.std_msgs import Header

try:
    from geometry_msgs.msg import Twist as ROS2Twist
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS 2 message definitions not available")


def main():
    parser = argparse.ArgumentParser(description='ROS 2 Zenoh Publisher Example')
    parser.add_argument('--topic', default='/turtle1/cmd_vel', 
                       help='Topic name to publish to')
    parser.add_argument('--duration', type=int, default=10,
                       help='Duration to publish (seconds)')
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
    parser.add_argument('--node-name', default='zenoh_publisher',
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
            
            # Create publisher based on message type preference
            if args.use_simplified or not ROS2_AVAILABLE:
                # Use simplified message types
                pub = node.create_publisher(Twist, args.topic)
                logger.info(f"Publisher created for topic '{args.topic}' (simplified message types)")
                
                logger.info(f"Publishing simplified Twist messages for {args.duration} seconds...")
                logger.info(f"Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"Rate: {args.rate} Hz")
                
                start_time = time.time()
                while time.time() - start_time < args.duration:
                    # Create simplified message
                    msg = Twist(
                        linear=Vector3(x=args.linear_x, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=args.angular_z)
                    )
                    
                    # Publish message
                    pub.publish(msg)
                    logger.debug(f"Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                    
                    # Wait for next publish cycle
                    time.sleep(1.0 / args.rate)
                
                # Send stop command
                logger.info("Sending stop command...")
                stop_msg = Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                pub.publish(stop_msg)
                
            else:
                # Use ROS 2 message types
                pub = node.create_publisher(ROS2Twist, args.topic)
                logger.info(f"Publisher created for topic '{args.topic}' (ROS 2 message types)")
                
                logger.info(f"Publishing ROS 2 Twist messages for {args.duration} seconds...")
                logger.info(f"Linear X: {args.linear_x}, Angular Z: {args.angular_z}")
                logger.info(f"Rate: {args.rate} Hz")
                
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
                    logger.debug(f"Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                    
                    # Wait for next publish cycle
                    time.sleep(1.0 / args.rate)
                
                # Send stop command
                logger.info("Sending stop command...")
                stop_msg = ROS2Twist()
                stop_msg.linear.x = 0.0
                stop_msg.linear.y = 0.0
                stop_msg.linear.z = 0.0
                stop_msg.angular.x = 0.0
                stop_msg.angular.y = 0.0
                stop_msg.angular.z = 0.0
                pub.publish(stop_msg)
            
            logger.info("Publisher finished")
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    print("Publisher example completed")


if __name__ == '__main__':
    main()
