#!/usr/bin/env python3
"""
Fibonacci Action Client Example

This example demonstrates how to use an action client with ros2_zenoh_python.
The client sends a Fibonacci sequence request and receives feedback during execution.
"""

import asyncio
from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs import example_interfaces


async def feedback_callback(feedback_msg):
    """Handle feedback from the action server."""
    print(f"📊 Feedback: {feedback_msg.feedback.sequence}")


async def main():
    """Main entry point."""
    # Create node
    async with Node('fibonacci_action_client') as node:
        print("🚀 Creating Fibonacci action client...")
        
        # Create action client
        client = node.create_action_client(
            example_interfaces.action.Fibonacci,
            'fibonacci'
        )
        
        print(f"✅ Action client created for /fibonacci\n")
        
        # Create goal
        goal = example_interfaces.action.Fibonacci.Goal()
        goal.order = 10
        
        print(f"🎯 Sending goal: Fibonacci({goal.order})")
        
        # Send goal
        goal_handle = await client.send_goal_async(
            goal,
            feedback_callback=feedback_callback
        )
        
        if not goal_handle.accepted:
            print("❌ Goal rejected!")
            return
        
        print(f"✅ Goal accepted!\n")
        
        # Wait for result
        print("⏳ Waiting for result...")
        result = await client.get_result_async(goal_handle)
        
        print(f"\n🎉 Result: {result.sequence}")
        print(f"   Status: {goal_handle.status.name}")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")

