#!/usr/bin/env python3
"""
Introspect the internal structure of an rclpy ActionServer.
This will show us EXACTLY what entities it creates.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci
import inspect

class IntrospectionNode(Node):
    def __init__(self):
        super().__init__('introspection_node')
        
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        
        print("\n" + "="*80)
        print("RCLPY ACTION SERVER INTROSPECTION")
        print("="*80)
        
        # Introspect the ActionServer object
        print("\n📋 ActionServer attributes:")
        for attr in dir(self.action_server):
            if not attr.startswith('_'):
                continue
            val = getattr(self.action_server, attr, None)
            if val is not None and not callable(val):
                print(f"   {attr}: {type(val).__name__}")
        
        # Look for service servers
        print("\n🔍 Looking for internal entities...")
        for attr in dir(self.action_server):
            val = getattr(self.action_server, attr, None)
            val_type = type(val).__name__
            
            if 'service' in attr.lower() or 'Service' in val_type:
                print(f"   SERVICE: {attr} -> {val_type}")
            if 'publisher' in attr.lower() or 'Publisher' in val_type:
                print(f"   PUBLISHER: {attr} -> {val_type}")
            if 'subscriber' in attr.lower() or 'Subscriber' in val_type:
                print(f"   SUBSCRIBER: {attr} -> {val_type}")
        
        # Check the actual implementation
        print("\n📝 ActionServer source file:")
        try:
            source_file = inspect.getfile(ActionServer)
            print(f"   {source_file}")
        except:
            pass
        
        print("\n" + "="*80)
        self.get_logger().info('Introspection complete. Ctrl+C to exit.')
    
    def execute_callback(self, goal_handle):
        result = Fibonacci.Result()
        result.sequence = [0, 1]
        goal_handle.succeed()
        return result

def main():
    rclpy.init()
    node = IntrospectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

