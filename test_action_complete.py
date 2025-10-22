#!/usr/bin/env python3
"""
Test complete action system (server + client).
"""

import asyncio
from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs import example_interfaces


async def execute_fibonacci(goal_handle):
    """Execute Fibonacci action on server."""
    print(f"  [SERVER] Executing goal: order={goal_handle.goal.order}")
    
    result = example_interfaces.action.Fibonacci.Result()
    result.sequence = [0, 1]
    
    for i in range(1, goal_handle.goal.order):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            print(f"  [SERVER] Goal canceled")
            return result
        
        result.sequence.append(result.sequence[i] + result.sequence[i-1])
        
        feedback = example_interfaces.action.Fibonacci.Feedback()
        feedback.sequence = list(result.sequence)
        goal_handle.publish_feedback(feedback)
        
        await asyncio.sleep(0.1)
    
    goal_handle.succeed()
    print(f"  [SERVER] Goal succeeded!")
    return result


async def feedback_callback(feedback_msg):
    """Handle feedback on client."""
    print(f"  [CLIENT] Feedback: {feedback_msg.feedback.sequence}")


async def run_server(node):
    """Run action server."""
    server = node.create_action_server(
        example_interfaces.action.Fibonacci,
        'fibonacci',
        execute_fibonacci
    )
    print("[SERVER] Action server ready")
    # Server just needs to exist, spin handles callbacks


async def run_client(node):
    """Run action client."""
    # Wait a bit for server to be ready
    await asyncio.sleep(0.5)
    
    client = node.create_action_client(
        example_interfaces.action.Fibonacci,
        'fibonacci'
    )
    print("[CLIENT] Action client ready")
    
    # Send goal
    goal = example_interfaces.action.Fibonacci.Goal()
    goal.order = 5
    
    print(f"[CLIENT] Sending goal: order={goal.order}")
    goal_handle = await client.send_goal_async(goal, feedback_callback=feedback_callback)
    
    if not goal_handle.accepted:
        print("[CLIENT] ❌ Goal rejected!")
        return False
    
    print("[CLIENT] ✅ Goal accepted!")
    
    # Get result
    result = await client.get_result_async(goal_handle)
    print(f"[CLIENT] ✅ Result: {result.sequence}")
    print(f"[CLIENT] Status: {goal_handle.status.name}")
    
    return True


async def main():
    """Main entry point."""
    print("="*70)
    print("  TESTING COMPLETE ACTION SYSTEM")
    print("="*70)
    print()
    
    async with Node('test_actions') as node:
        # Start server
        await run_server(node)
        
        # Run client
        success = await run_client(node)
        
        # Small delay to let final feedback arrive
        await asyncio.sleep(0.5)
        
        print()
        print("="*70)
        if success:
            print("  ✅ TEST PASSED - Actions working end-to-end!")
        else:
            print("  ❌ TEST FAILED")
        print("="*70)


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Interrupted")

