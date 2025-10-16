#!/usr/bin/env python3
"""
rclpy-like Example using ros2_zenoh_python

This example demonstrates the rclpy-like interface with generated ROS 2 message types.
"""

import sys
import time
from ros2_zenoh_python import Node
from ros2_interfaces_python.geometry_msgs.msg.geometry_msgs import Twist, Vector3


def message_callback(msg):
    """Callback function to handle received messages."""
    print(f"📨 Received message:")
    print(f"   Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}")
    print(f"   Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")
    print("-" * 50)


def main():
    # Simple configuration
    mode = 'both'  # 'pub', 'sub', or 'both'
    topic = '/turtle1/cmd_vel'
    duration = 10  # seconds
    rate = 1.0  # Hz
    linear_x = 1.0
    angular_z = 0.5
    zenoh_endpoint = 'tcp/172.18.0.2:7447'
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
    
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
            if mode in ['pub', 'both']:
                pub = node.create_publisher(Twist, topic)
                logger.info(f"Publisher created for topic '{topic}'")
                publishers.append(pub)
            
            # Create subscribers if requested
            # This matches rclpy node.create_subscription() interface
            if mode in ['sub', 'both']:
                sub = node.create_subscription(Twist, topic, message_callback)
                logger.info(f"Subscriber created for topic '{topic}'")
                subscribers.append(sub)
            
            logger.info(f"Running for {duration} seconds...")
            logger.info("Press Ctrl+C to stop early")
            
            start_time = time.time()
            while time.time() - start_time < duration:
                # Publish messages if we have publishers
                if publishers:
                    for pub in publishers:
                        # Create message using generated types
                        msg = Twist(
                            linear=Vector3(x=linear_x, y=0.0, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=angular_z)
                        )
                        pub.publish(msg)
                
                # Wait for next cycle
                time.sleep(1.0 / rate)
            
            # Send stop command
            if publishers:
                logger.info("Sending stop command...")
                for pub in publishers:
                    stop_msg = Twist(
                        linear=Vector3(x=0.0, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0)
                    )
                    pub.publish(stop_msg)
            
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    
    print("✅ Example finished")


if __name__ == '__main__':
    main()
