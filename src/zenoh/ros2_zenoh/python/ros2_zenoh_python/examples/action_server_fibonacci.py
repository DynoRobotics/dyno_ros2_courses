#!/usr/bin/env python3
"""
Fibonacci Action Server Example

This example demonstrates how to create an action server using ros2_zenoh_python.
The server accepts Fibonacci sequence requests and provides feedback during execution.
"""

import asyncio
from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs import example_interfaces


async def execute_fibonacci(goal_handle):
    """
    Execute the Fibonacci action.
    
    Args:
        goal_handle: Handle for tracking goal status and publishing feedback
        
    Returns:
        example_interfaces.action.Fibonacci.Result with the final sequence
    """
    print(f"🎯 Executing goal: compute Fibonacci sequence up to order {goal_handle.goal.order}")
    
    # Initialize result with first two numbers
    result = example_interfaces.action.Fibonacci.Result()
    result.sequence = [0, 1]
    
    # Compute Fibonacci sequence
    for i in range(1, goal_handle.goal.order):
        # Check if cancellation was requested
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            print("🚫 Goal canceled")
            return result
        
        # Compute next number in sequence
        result.sequence.append(
            result.sequence[i] + result.sequence[i-1]
        )
        
        # Publish feedback
        feedback = example_interfaces.action.Fibonacci.Feedback()
        feedback.sequence = list(result.sequence)
        goal_handle.publish_feedback(feedback)
        
        print(f"📊 Feedback: {feedback.sequence}")
        
        # Simulate computation time
        await asyncio.sleep(0.5)
    
    # Mark goal as succeeded
    goal_handle.succeed()
    print(f"✅ Goal succeeded! Final sequence: {result.sequence}")
    
    return result


def goal_callback(goal):
    """
    Decide whether to accept or reject a goal request.
    
    Args:
        goal: The goal request
        
    Returns:
        GoalResponse.ACCEPT or GoalResponse.REJECT
    """
    from ros2_zenoh_python.action_server import GoalResponse
    
    if goal.order <= 0:
        print(f"❌ Rejecting goal: order must be positive (got {goal.order})")
        return GoalResponse.REJECT
    
    if goal.order > 20:
        print(f"❌ Rejecting goal: order too large (max 20, got {goal.order})")
        return GoalResponse.REJECT
    
    print(f"✅ Accepting goal: order = {goal.order}")
    return GoalResponse.ACCEPT


async def main():
    """Main entry point."""
    # Create node
    async with Node('fibonacci_action_server') as node:
        print("🚀 Starting Fibonacci action server...")
        
        # Create action server
        action_server = node.create_action_server(
            example_interfaces.action.Fibonacci,
            'fibonacci',
            execute_fibonacci,
            goal_callback=goal_callback
        )
        
        print("""
╔══════════════════════════════════════════════════════════════════════════╗
║                   FIBONACCI ACTION SERVER READY                          ║
╚══════════════════════════════════════════════════════════════════════════╝

📋 Action: /fibonacci
📦 Type:   example_interfaces/action/Fibonacci

🔧 Test with:
   - ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}"
   - Or use the action_client_fibonacci.py example

⏸️  Press Ctrl+C to stop
""")
        
        # Spin forever
        await node.spin()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")

