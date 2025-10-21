#!/usr/bin/env python3
"""
Simple ROS2 service client for introspection.

Uses AddTwoInts service from example_interfaces.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import time


class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        self.get_logger().info('Service available!')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        self.get_logger().info(f'Sending request: {a} + {b}')
        future = self.client.call_async(request)
        return future


def main():
    rclpy.init()
    node = SimpleServiceClient()
    
    try:
        # Send a few requests
        for i in range(5):
            future = node.send_request(i, i * 10)
            rclpy.spin_until_future_complete(node, future)
            
            if future.result() is not None:
                response = future.result()
                node.get_logger().info(f'Response: {response.sum}')
            else:
                node.get_logger().error('Service call failed')
            
            time.sleep(1)
    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


