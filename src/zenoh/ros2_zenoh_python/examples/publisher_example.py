#!/usr/bin/env python3
"""
Publisher Example using ros2_zenoh_python

This example demonstrates how to use the ros2_zenoh_python package
to publish messages using the generated ROS 2 message types.
"""

import sys
import time
from ros2_zenoh_python import Node
from ros2_interfaces_python.geometry_msgs.msg.geometry_msgs import Twist, Vector3


def main():
    # Simple configuration
    topic = '/turtle1/cmd_vel'
    duration = 10  # seconds
    rate = 1.0  # Hz
    linear_x = 1.0
    angular_z = 0.5
    zenoh_endpoint = 'tcp/172.18.0.2:7447'
    node_name = 'zenoh_publisher'
    
    # Create Zenoh configuration
    import zenoh
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
    
    try:
        # Create a node
        with Node(node_name, zenoh_config=config) as node:
            logger = node.get_logger()
            logger.info(f"Node '{node.node_name}' created")
            
            # Create publisher using generated message types
            pub = node.create_publisher(Twist, topic)
            logger.info(f"Publisher created for topic '{topic}'")
            
            logger.info(f"Publishing Twist messages for {duration} seconds...")
            logger.info(f"Linear X: {linear_x}, Angular Z: {angular_z}")
            logger.info(f"Rate: {rate} Hz")
            
            start_time = time.time()
            while time.time() - start_time < duration:
                # Create message using generated types
                msg = Twist(
                    linear=Vector3(x=linear_x, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=angular_z)
                )
                
                # Publish message
                pub.publish(msg)
                logger.debug(f"Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                
                # Wait for next publish cycle
                time.sleep(1.0 / rate)
            
            # Send stop command
            logger.info("Sending stop command...")
            stop_msg = Twist(
                linear=Vector3(x=0.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0)
            )
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
