#!/usr/bin/env python3
"""
Inspect the liveliness tokens created by a real rclpy action server.
This will show us what entities actions actually create.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci
import time
import sys
import subprocess
import threading

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci action server ready')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal with order {goal_handle.request.order}...')
        result = Fibonacci.Result()
        result.sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            result.sequence.append(result.sequence[i] + result.sequence[i-1])
            feedback = Fibonacci.Feedback()
            feedback.sequence = result.sequence
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
        goal_handle.succeed()
        return result

def query_liveliness():
    """Query Zenoh liveliness tokens to see what the action server created."""
    time.sleep(2)  # Wait for server to start
    
    print("\n" + "="*80)
    print("QUERYING ZENOH LIVELINESS TOKENS")
    print("="*80)
    
    # Use zenoh-python to query liveliness
    try:
        import zenoh
        conf = zenoh.Config()
        session = zenoh.open(conf)
        
        # Query all liveliness tokens
        replies = session.liveliness().get("@ros2_lv/**")
        
        print("\nLiveliness tokens from action server:\n")
        for reply in replies:
            key = str(reply.ok.key_expr)
            if 'fibonacci' in key.lower():
                print(f"  {key}")
        
        session.close()
    except ImportError:
        print("⚠️  zenoh-python not available, using ros2 cli instead")
        # Use ros2 CLI to introspect
        result = subprocess.run(
            ['ros2', 'action', 'list', '-v'],
            capture_output=True,
            text=True
        )
        print("\nros2 action list output:")
        print(result.stdout)
        
        # Check topics
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True
        )
        print("\nTopics created by action:")
        for line in result.stdout.split('\n'):
            if 'fibonacci' in line.lower():
                print(f"  {line}")
        
        # Check services
        result = subprocess.run(
            ['ros2', 'service', 'list'],
            capture_output=True,
            text=True
        )
        print("\nServices created by action:")
        for line in result.stdout.split('\n'):
            if 'fibonacci' in line.lower():
                print(f"  {line}")

def main():
    rclpy.init()
    
    # Start liveliness query in background
    query_thread = threading.Thread(target=query_liveliness, daemon=True)
    query_thread.start()
    
    # Create action server
    action_server = FibonacciActionServer()
    
    print("\n" + "="*80)
    print("ACTION SERVER ANALYSIS")
    print("="*80)
    print("\nAn rclpy ActionServer creates the following ROS2 entities:")
    print("\n📋 Services (3):")
    print("  1. /_action/send_goal     - SendGoal service")
    print("  2. /_action/cancel_goal   - CancelGoal service") 
    print("  3. /_action/get_result    - GetResult service")
    print("\n📡 Topics (2):")
    print("  1. /_action/feedback      - Feedback messages (pub by server)")
    print("  2. /_action/status        - Goal status array (pub by server)")
    print("\n✅ Actions are composed of services + topics!")
    print("   No special 'action' entity at the RMW level.")
    print("="*80)
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

