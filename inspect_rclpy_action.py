#!/usr/bin/env python3
"""Simple rclpy action server for introspection."""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class MinimalActionServer(Node):
    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('🚀 Action server started!')
        self.get_logger().info('')
        self.get_logger().info('📋 In another terminal, run:')
        self.get_logger().info('   ros2 service list | grep fibonacci')
        self.get_logger().info('   ros2 topic list | grep fibonacci')
        self.get_logger().info('')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: {goal_handle.request.order}')
        
        # Compute fibonacci
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main():
    rclpy.init()
    server = MinimalActionServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

