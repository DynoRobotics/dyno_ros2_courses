"""
Test basic action functionality for ros2_zenoh_python.

Tests action server and client communication without rclpy.
"""

import pytest
import asyncio
import time
from ros2_zenoh_python import Node, ActionServer, ActionClient, GoalResponse, CancelResponse, GoalStatus
from ros2_zenoh_python._bundled_msgs import example_interfaces

Fibonacci = example_interfaces.action.Fibonacci


@pytest.mark.asyncio
async def test_action_goal_accepted(shared_zenoh_node):
    """Test that action goal can be sent and accepted."""
    
    # Goal execution callback
    async def execute_fibonacci(goal_handle):
        """Execute the Fibonacci action."""
        result = Fibonacci.Result()
        result.sequence = [0, 1, 1, 2, 3, 5, 8]
        return result
    
    # Create server
    server = shared_zenoh_node.create_action_server(
        Fibonacci,
        'test_fibonacci',
        execute_callback=execute_fibonacci
    )
    
    # Create client
    client = shared_zenoh_node.create_action_client(
        Fibonacci,
        'test_fibonacci'
    )
    
    # Wait for discovery
    await client.wait_for_action_server(timeout=5.0)
    
    # Send goal
    goal = Fibonacci.Goal()
    goal.order = 7
    
    goal_handle = await client.send_goal_async(goal)
    
    # Verify goal was accepted
    assert goal_handle is not None
    assert goal_handle.status == GoalStatus.STATUS_ACCEPTED or goal_handle.status == GoalStatus.STATUS_EXECUTING
    
    # Get result
    result = await client.get_result_async(goal_handle)
    
    # Verify result
    assert result is not None
    assert result.sequence == [0, 1, 1, 2, 3, 5, 8]
    
    # Cleanup
    server.destroy()
    client.destroy()


@pytest.mark.asyncio
async def test_action_with_feedback(shared_zenoh_node):
    """Test action with feedback messages."""
    
    feedback_received = []
    
    async def execute_with_feedback(goal_handle):
        """Execute with feedback."""
        # Send some feedback
        for i in range(3):
            feedback = Fibonacci.Feedback()
            feedback.sequence = [0] * (i + 1)
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.1)
        
        result = Fibonacci.Result()
        result.sequence = [0, 1, 1]
        return result
    
    def feedback_callback(feedback_msg):
        """Collect feedback."""
        feedback_received.append(len(feedback_msg.feedback.sequence))
    
    # Create server
    server = shared_zenoh_node.create_action_server(
        Fibonacci,
        'test_feedback',
        execute_callback=execute_with_feedback
    )
    
    # Create client
    client = shared_zenoh_node.create_action_client(
        Fibonacci,
        'test_feedback'
    )
    
    await client.wait_for_action_server(timeout=5.0)
    
    # Send goal with feedback callback
    goal = Fibonacci.Goal()
    goal.order = 3
    
    goal_handle = await client.send_goal_async(goal, feedback_callback=feedback_callback)
    result = await client.get_result_async(goal_handle)
    
    # Actively wait for feedback (max 0.5s)
    for _ in range(25):
        if len(feedback_received) > 0:
            break
        await asyncio.sleep(0.02)
    
    # Verify we got feedback
    assert len(feedback_received) > 0, "No feedback received - check feedback mechanism"
    
    # Cleanup
    server.destroy()
    client.destroy()


@pytest.mark.asyncio
async def test_action_goal_rejected(shared_zenoh_node):
    """Test that action goal can be rejected."""
    
    def goal_callback(goal_request):
        """Reject goals with order > 10."""
        if goal_request.order > 10:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    async def execute_callback(goal_handle):
        """Simple execution."""
        result = Fibonacci.Result()
        result.sequence = [0, 1]
        return result
    
    # Create server with goal callback
    server = shared_zenoh_node.create_action_server(
        Fibonacci,
        'test_reject',
        execute_callback=execute_callback,
        goal_callback=goal_callback
    )
    
    # Create client
    client = shared_zenoh_node.create_action_client(
        Fibonacci,
        'test_reject'
    )
    
    await client.wait_for_action_server(timeout=5.0)
    
    # Send goal that should be rejected
    goal = Fibonacci.Goal()
    goal.order = 100
    
    goal_handle = await client.send_goal_async(goal)
    
    # Goal should be None (rejected)
    assert goal_handle is None
    
    # Send goal that should be accepted
    goal.order = 5
    goal_handle = await client.send_goal_async(goal)
    assert goal_handle is not None
    
    # Cleanup
    server.destroy()
    client.destroy()


@pytest.mark.asyncio
async def test_action_cancel(shared_zenoh_node):
    """Test action goal cancellation."""
    
    cancel_requested = False
    
    async def execute_long_action(goal_handle):
        """Long-running action that can be cancelled."""
        nonlocal cancel_requested
        
        for i in range(10):
            if goal_handle.is_cancel_requested:
                cancel_requested = True
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = [0] * i
                return result
            
            await asyncio.sleep(0.1)
        
        result = Fibonacci.Result()
        result.sequence = [0] * 10
        return result
    
    # Create server
    server = shared_zenoh_node.create_action_server(
        Fibonacci,
        'test_cancel',
        execute_callback=execute_long_action
    )
    
    # Create client
    client = shared_zenoh_node.create_action_client(
        Fibonacci,
        'test_cancel'
    )
    
    await client.wait_for_action_server(timeout=5.0)
    
    # Send goal
    goal = Fibonacci.Goal()
    goal.order = 10
    
    goal_handle = await client.send_goal_async(goal)
    assert goal_handle is not None
    
    # Cancel immediately
    cancel_response = await client.cancel_goal_async(goal_handle)
    
    # Actively wait for cancel (max 0.5s)
    for _ in range(25):
        if cancel_requested:
            break
        await asyncio.sleep(0.02)
    
    # Verify cancellation was requested
    assert cancel_requested, "Cancel was not processed - check cancel mechanism"
    
    # Cleanup
    server.destroy()
    client.destroy()


@pytest.mark.asyncio
async def test_action_creation(shared_zenoh_node):
    """Test basic action server and client creation."""
    
    async def dummy_execute(goal_handle):
        result = Fibonacci.Result()
        result.sequence = []
        return result
    
    # Create server
    server = shared_zenoh_node.create_action_server(
        Fibonacci,
        'test_create',
        execute_callback=dummy_execute
    )
    
    assert server is not None
    assert server.action_name == 'test_create'
    
    # Create client
    client = shared_zenoh_node.create_action_client(
        Fibonacci,
        'test_create'
    )
    
    assert client is not None
    assert client.action_name == 'test_create'
    
    # Cleanup
    server.destroy()
    client.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

