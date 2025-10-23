"""
Test action interoperability between ros2_zenoh_python and rclpy.

These tests verify that ros2_zenoh_python action servers/clients
can communicate with rclpy action clients/servers.
"""

import pytest
import pytest_asyncio
import asyncio
import threading
import time
import rclpy
from rclpy.action import ActionServer as RclpyActionServer, ActionClient as RclpyActionClient, GoalResponse, CancelResponse
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from example_interfaces.action import Fibonacci as RclpyFibonacci

from ros2_zenoh_python import Node as ZenohNode, ActionServer as ZenohActionServer, ActionClient as ZenohActionClient
from ros2_zenoh_python._bundled_msgs import example_interfaces

ZenohFibonacci = example_interfaces.action.Fibonacci


class SharedRclpyActionServer:
    """Shared rclpy node with action server for all tests."""
    
    def __init__(self):
        self.node = rclpy.create_node('shared_rclpy_action_server')
        # Use ReentrantCallbackGroup to allow cancel callback while execute is running
        self.node._default_callback_group = ReentrantCallbackGroup()
        self._action_server = RclpyActionServer(
            self.node,
            RclpyFibonacci,
            'fibonacci_rclpy',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # Use MultiThreadedExecutor to allow cancel callback concurrently with execute
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        self.cancel_callback_called = False
        # Synchronization for deterministic testing
        self.execute_started = threading.Event()
        self.block_execution = False  # Control for cancel tests
    
    def goal_callback(self, goal_request):
        """Accept or reject goals based on order."""
        if goal_request.order > 10:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, cancel_request):
        """Handle cancel requests."""
        self.cancel_callback_called = True
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """Execute Fibonacci action with feedback and cancel support."""
        result = RclpyFibonacci.Result()
        
        # Signal that execution has started
        self.execute_started.set()
        
        # Send feedback quickly (no artificial delays)
        feedback_msg = RclpyFibonacci.Feedback()
        for i in range(5):  # Just 5 iterations, fast
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.sequence = [0] * i
                return result
            
            if i < 3:  # Send 3 feedback messages
                feedback_msg.sequence = [0] * (i + 1)
                goal_handle.publish_feedback(feedback_msg)
            
            # For cancel tests, block here after signaling start
            if self.block_execution and i == 0:
                # Wait for cancel (with timeout)
                for _ in range(100):  # Max 1 second
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.sequence = [0]
                        return result
                    time.sleep(0.01)
            else:
                time.sleep(0.01)  # Minimal delay
        
        result.sequence = [0, 1, 1, 2, 3, 5, 8, 13]
        goal_handle.succeed()
        return result
    
    def cleanup(self):
        """Cleanup resources."""
        self._action_server.destroy()
        self.executor.shutdown()
        self.node.destroy_node()


class SharedRclpyActionClient:
    """Shared rclpy node with action client for all tests."""
    
    def __init__(self):
        self.node = rclpy.create_node('shared_rclpy_action_client')
        self._action_client = RclpyActionClient(
            self.node,
            RclpyFibonacci,
            'fibonacci_zenoh'
        )
        # Use MultiThreadedExecutor for consistency
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
    
    def send_goal(self, order, feedback_callback=None, wait_timeout=1.0):
        """Send goal and wait for result."""
        goal_msg = RclpyFibonacci.Goal()
        goal_msg.order = order
        
        # Wait for server with configurable timeout
        if not self._action_client.wait_for_server(timeout_sec=wait_timeout):
            return None, None
        
        # Send goal
        future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=feedback_callback
        )
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return None, None
        
        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
        
        return goal_handle, result_future.result().result
    
    def cleanup(self):
        """Cleanup resources."""
        self._action_client.destroy()
        self.executor.shutdown()
        self.node.destroy_node()


# Module-scoped fixtures
@pytest.fixture(scope="module")
def shared_rclpy_server(rclpy_session):
    """Create a shared rclpy action server for all tests in this module."""
    server = SharedRclpyActionServer()
    yield server
    server.cleanup()


@pytest.fixture(scope="module")
def shared_rclpy_client(rclpy_session):
    """Create a shared rclpy action client for all tests in this module."""
    client = SharedRclpyActionClient()
    yield client
    client.cleanup()


@pytest_asyncio.fixture(scope="module")
async def shared_zenoh_session_for_actions(zenoh_session_client):
    """Shared Zenoh session for action tests."""
    yield zenoh_session_client


# Tests
@pytest.mark.asyncio
async def test_zenoh_client_to_rclpy_server(shared_rclpy_server, shared_zenoh_session_for_actions):
    """Test Zenoh action client calling rclpy action server."""
    
    async with ZenohNode('test_zenoh_client_1', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        client = node.create_action_client(
            ZenohFibonacci,
            'fibonacci_rclpy'
        )
    
        try:
            # Wait for action server
            assert await client.wait_for_action_server(timeout=5.0), "Action server not found"
            
            # Send goal
            goal = ZenohFibonacci.Goal()
            goal.order = 8
            
            goal_handle = await client.send_goal_async(goal)
            assert goal_handle is not None, "Goal was not accepted"
            
            # Get result
            result = await client.get_result_async(goal_handle)
            
            # Verify result
            assert result is not None
            assert result.sequence == [0, 1, 1, 2, 3, 5, 8, 13]
        finally:
            client.destroy()


@pytest.mark.asyncio
async def test_rclpy_client_to_zenoh_server(shared_rclpy_client, shared_zenoh_session_for_actions):
    """Test rclpy action client calling Zenoh action server."""
    
    async def execute_callback(goal_handle):
        """Execute Fibonacci action."""
        result = ZenohFibonacci.Result()
        result.sequence = [0, 1, 1, 2, 3, 5, 8, 13]
        return result
    
    async with ZenohNode('test_zenoh_server_1', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        server = node.create_action_server(
            ZenohFibonacci,
            'fibonacci_zenoh',
            execute_callback=execute_callback
        )
        
        try:
            # Wait for rclpy client to see Zenoh server (max 3 attempts)
            goal_handle, result = None, None
            for attempt in range(3):
                goal_handle, result = await asyncio.get_event_loop().run_in_executor(
                    None, shared_rclpy_client.send_goal, 8
                )
                if goal_handle is not None:
                    break
                await asyncio.sleep(0.1)
            
            # Verify result
            assert goal_handle is not None, "Goal was not accepted after retries"
            assert result is not None
            assert list(result.sequence) == [0, 1, 1, 2, 3, 5, 8, 13]
        finally:
            server.destroy()


@pytest.mark.asyncio
async def test_bidirectional_action_communication(shared_rclpy_server, shared_rclpy_client, shared_zenoh_session_for_actions):
    """Test bidirectional action communication between Zenoh and rclpy."""
    
    async def execute_callback_zenoh(goal_handle):
        """Zenoh server callback."""
        result = ZenohFibonacci.Result()
        result.sequence = [0, 1, 1, 2, 3, 5, 8, 13]
        return result
    
    async with ZenohNode('test_zenoh_bidir', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        # Create Zenoh server
        zenoh_server = node.create_action_server(
            ZenohFibonacci,
            'fibonacci_zenoh',
            execute_callback=execute_callback_zenoh
        )
        
        # Create Zenoh client
        zenoh_client = node.create_action_client(
            ZenohFibonacci,
            'fibonacci_rclpy'
        )
        
        try:
            # Test 1: Zenoh client -> rclpy server
            assert await zenoh_client.wait_for_action_server(timeout=5.0)
            goal = ZenohFibonacci.Goal()
            goal.order = 8
            
            goal_handle_1 = await zenoh_client.send_goal_async(goal)
            assert goal_handle_1 is not None
            result_1 = await zenoh_client.get_result_async(goal_handle_1)
            assert result_1.sequence == [0, 1, 1, 2, 3, 5, 8, 13]
            
            # Test 2: rclpy client -> Zenoh server (max 3 attempts)
            goal_handle_2, result_2 = None, None
            for attempt in range(3):
                goal_handle_2, result_2 = await asyncio.get_event_loop().run_in_executor(
                    None, shared_rclpy_client.send_goal, 8
                )
                if goal_handle_2 is not None:
                    break
                await asyncio.sleep(0.1)
            assert goal_handle_2 is not None, "Goal not accepted - discovery issue?"
            assert list(result_2.sequence) == [0, 1, 1, 2, 3, 5, 8, 13]
        finally:
            zenoh_server.destroy()
            zenoh_client.destroy()


@pytest.mark.asyncio
async def test_zenoh_client_receives_feedback_from_rclpy_server(shared_rclpy_server, shared_zenoh_session_for_actions):
    """Test that Zenoh client receives feedback from rclpy server."""
    
    async with ZenohNode('test_node_1', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        feedback_received = []
        
        def feedback_callback(feedback_msg):
            feedback_received.append(len(feedback_msg.feedback.sequence))
        
        client = node.create_action_client(
            ZenohFibonacci,
            'fibonacci_rclpy'
        )
        
        try:
            assert await client.wait_for_action_server(timeout=5.0)
            
            goal = ZenohFibonacci.Goal()
            goal.order = 8
            
            goal_handle = await client.send_goal_async(goal, feedback_callback=feedback_callback)
            assert goal_handle is not None
            
            result = await client.get_result_async(goal_handle)
            
            # Actively wait for feedback (max 0.5s)
            for _ in range(25):
                if len(feedback_received) > 0:
                    break
                await asyncio.sleep(0.02)
            
            # Verify feedback was received
            assert len(feedback_received) > 0, "No feedback received - check feedback mechanism"
            assert result.sequence == [0, 1, 1, 2, 3, 5, 8, 13]
        finally:
            client.destroy()


@pytest.mark.asyncio
async def test_zenoh_client_cancels_rclpy_server_goal(shared_rclpy_server, shared_zenoh_session_for_actions):
    """Test that Zenoh client can cancel a goal on rclpy server."""
    
    # Enable blocking execution for deterministic cancel testing
    shared_rclpy_server.block_execution = True
    shared_rclpy_server.execute_started.clear()
    
    try:
        async with ZenohNode('test_node_2', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
            client = node.create_action_client(
                ZenohFibonacci,
                'fibonacci_rclpy'
            )
        
            try:
                assert await client.wait_for_action_server(timeout=5.0)
            
                goal = ZenohFibonacci.Goal()
                goal.order = 8
            
                # Send goal in background
                goal_handle_future = asyncio.create_task(client.send_goal_async(goal))
                
                # Wait for execution to actually start
                await asyncio.get_event_loop().run_in_executor(
                    None, shared_rclpy_server.execute_started.wait, 2.0
                )
                
                goal_handle = await goal_handle_future
                assert goal_handle is not None, "Goal was not accepted"
                
                # Cancel while executing
                cancel_response = await client.cancel_goal_async(goal_handle)
            
                # Verify cancellation
                assert cancel_response is not None
            finally:
                client.destroy()
    finally:
        # Reset state
        shared_rclpy_server.block_execution = False
        shared_rclpy_server.execute_started.clear()


@pytest.mark.asyncio
async def test_rclpy_client_receives_feedback_from_zenoh_server(shared_rclpy_client, shared_zenoh_session_for_actions):
    """Test that rclpy client receives feedback from Zenoh server."""
    
    async with ZenohNode('test_node_3', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        feedback_received = []
        
        def feedback_callback(feedback_msg):
            feedback_received.append(len(feedback_msg.feedback.sequence))
        
        async def execute_with_feedback(goal_handle):
            """Execute with feedback."""
            for i in range(3):
                feedback = ZenohFibonacci.Feedback()
                feedback.sequence = [0] * (i + 1)
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.05)
            
            result = ZenohFibonacci.Result()
            result.sequence = [0, 1, 1, 2, 3, 5, 8, 13]
            return result
        
        server = node.create_action_server(
            ZenohFibonacci,
            'fibonacci_zenoh',
            execute_callback=execute_with_feedback
        )
        
        try:
            # Send goal with feedback callback (max 3 attempts)
            goal_handle, result = None, None
            for attempt in range(3):
                goal_handle, result = await asyncio.get_event_loop().run_in_executor(
                    None, shared_rclpy_client.send_goal, 8, feedback_callback
                )
                if goal_handle is not None:
                    break
                await asyncio.sleep(0.1)
            
            # Actively wait for feedback (max 0.5s)
            for _ in range(25):
                if len(feedback_received) > 0:
                    break
                await asyncio.sleep(0.02)
            
            assert goal_handle is not None
            assert list(result.sequence) == [0, 1, 1, 2, 3, 5, 8, 13]
            assert len(feedback_received) > 0, "No feedback received"
        finally:
            server.destroy()


@pytest.mark.asyncio
async def test_rclpy_client_cancels_zenoh_server_goal(rclpy_session, shared_zenoh_session_for_actions):
    """Test that rclpy client can cancel a goal on Zenoh server."""
    
    async with ZenohNode('test_node_4', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        cancel_requested = False
        
        async def execute_long_action(goal_handle):
            """Long-running action that can be cancelled."""
            nonlocal cancel_requested
            
            for i in range(10):
                if goal_handle.is_cancel_requested:
                    cancel_requested = True
                    goal_handle.canceled()
                    result = ZenohFibonacci.Result()
                    result.sequence = [0] * i
                    return result
                await asyncio.sleep(0.1)
            
            result = ZenohFibonacci.Result()
            result.sequence = [0] * 10
            return result
        
        server = node.create_action_server(
            ZenohFibonacci,
            'fibonacci_zenoh',
            execute_callback=execute_long_action
        )
        
        try:
            # Use synchronous rclpy operations in executor
            def send_and_cancel_goal():
                """Synchronous function to send and cancel goal."""
                client_node = rclpy.create_node('cancel_test_client')
                action_client = RclpyActionClient(client_node, RclpyFibonacci, 'fibonacci_zenoh')
                executor = SingleThreadedExecutor()
                executor.add_node(client_node)
                executor_thread = threading.Thread(target=executor.spin, daemon=True)
                executor_thread.start()
                
                try:
                    # Wait for server
                    if not action_client.wait_for_server(timeout_sec=5.0):
                        return False, "Server not found"
                    
                    # Send goal
                    goal_msg = RclpyFibonacci.Goal()
                    goal_msg.order = 10
                    future = action_client.send_goal_async(goal_msg)
                    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
                    goal_handle = future.result()
                    
                    if not goal_handle or not goal_handle.accepted:
                        return False, "Goal not accepted"
                    
                    # Cancel goal
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=5.0)
                    
                    return True, "Success"
                finally:
                    action_client.destroy()
                    executor.shutdown()
                    client_node.destroy_node()
            
            # Run in executor to avoid blocking asyncio
            success, msg = await asyncio.get_event_loop().run_in_executor(None, send_and_cancel_goal)
            assert success, msg
            
            # Actively wait for cancel (max 0.5s)
            for _ in range(25):
                if cancel_requested:
                    break
                await asyncio.sleep(0.02)
            
            # Verify cancellation was processed
            assert cancel_requested, "Cancel was not processed"
        finally:
            server.destroy()


@pytest.mark.asyncio
async def test_zenoh_client_goal_rejected_by_rclpy_server(shared_rclpy_server, shared_zenoh_session_for_actions):
    """Test that rclpy server can reject a goal from Zenoh client."""
    
    async with ZenohNode('test_node_5', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        client = node.create_action_client(
            ZenohFibonacci,
            'fibonacci_rclpy'
        )
    
        try:
            assert await client.wait_for_action_server(timeout=5.0)
        
            # Send goal that should be rejected (order > 10)
            goal = ZenohFibonacci.Goal()
            goal.order = 100
        
            goal_handle = await client.send_goal_async(goal)
        
            # Goal should be rejected (None)
            assert goal_handle is None
        finally:
            client.destroy()


@pytest.mark.asyncio
async def test_rclpy_client_goal_rejected_by_zenoh_server(shared_rclpy_client, shared_zenoh_session_for_actions):
    """Test that Zenoh server can reject a goal from rclpy client."""
    
    async with ZenohNode('test_node_6', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        from ros2_zenoh_python import GoalResponse
        
        def goal_callback(goal_request):
            """Reject goals with order > 10."""
            if goal_request.order > 10:
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT
        
        async def execute_callback(goal_handle):
            result = ZenohFibonacci.Result()
            result.sequence = [0, 1]
            return result
        
        server = node.create_action_server(
            ZenohFibonacci,
            'fibonacci_zenoh',
            execute_callback=execute_callback,
            goal_callback=goal_callback
        )
        
        try:
            # Send goal that should be rejected (wait_for_server handles discovery)
            goal_handle, result = await asyncio.get_event_loop().run_in_executor(
                None, shared_rclpy_client.send_goal, 100
            )
            
            # Goal should be rejected
            assert goal_handle is None
        finally:
            server.destroy()


@pytest.mark.asyncio
async def test_zenoh_client_sees_rclpy_server_cancel_callback(shared_rclpy_server, shared_zenoh_session_for_actions):
    """Test that rclpy server's cancel callback is invoked when Zenoh client cancels."""
    
    # Reset the callback flag and enable blocking
    shared_rclpy_server.cancel_callback_called = False
    shared_rclpy_server.block_execution = True
    shared_rclpy_server.execute_started.clear()
    
    try:
        async with ZenohNode('test_node_7', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
            client = node.create_action_client(
                ZenohFibonacci,
                'fibonacci_rclpy'
            )
            
            try:
                # Wait for action server to be available
                assert await client.wait_for_action_server(timeout=5.0)
                
                goal = ZenohFibonacci.Goal()
                goal.order = 8
                
                # Send goal in background
                goal_handle_future = asyncio.create_task(client.send_goal_async(goal))
                
                # Wait for execution to actually start
                await asyncio.get_event_loop().run_in_executor(
                    None, shared_rclpy_server.execute_started.wait, 2.0
                )
                
                goal_handle = await goal_handle_future
                assert goal_handle is not None, "Goal was not accepted"
                
                # Cancel while executing
                cancel_response = await client.cancel_goal_async(goal_handle)
                
                # Actively wait for cancel callback (max 0.5s)
                for _ in range(25):
                    if shared_rclpy_server.cancel_callback_called:
                        break
                    await asyncio.sleep(0.02)
                
                # Verify cancel callback was invoked
                assert shared_rclpy_server.cancel_callback_called, "Cancel callback was not invoked on rclpy server"
            finally:
                client.destroy()
    finally:
        # Reset state
        shared_rclpy_server.block_execution = False
        shared_rclpy_server.execute_started.clear()


@pytest.mark.asyncio
async def test_rclpy_client_cancel_handled_by_zenoh_server(shared_zenoh_session_for_actions):
    """Test that Zenoh server's cancel callback is invoked when rclpy client cancels."""
    
    async with ZenohNode('test_node_8', zenoh_session=shared_zenoh_session_for_actions, enable_rosout=False) as node:
        from ros2_zenoh_python import CancelResponse
        
        cancel_callback_called = False
        
        def cancel_callback(cancel_request):
            """Handle cancel requests."""
            nonlocal cancel_callback_called
            cancel_callback_called = True
            return CancelResponse.ACCEPT
        
        cancel_requested_in_execute = False
        
        async def execute_long_action(goal_handle):
            """Long-running action."""
            nonlocal cancel_requested_in_execute
            
            for i in range(10):
                if goal_handle.is_cancel_requested:
                    cancel_requested_in_execute = True
                    goal_handle.canceled()
                    result = ZenohFibonacci.Result()
                    result.sequence = [0] * i
                    return result
                await asyncio.sleep(0.1)
            
            result = ZenohFibonacci.Result()
            result.sequence = [0] * 10
            return result
        
        server = node.create_action_server(
            ZenohFibonacci,
            'fibonacci_zenoh',
            execute_callback=execute_long_action,
            cancel_callback=cancel_callback
        )
        
        try:
            # Use synchronous rclpy operations in executor
            def send_and_cancel_goal():
                """Synchronous function to send and cancel goal."""
                client_node = rclpy.create_node('cancel_test_client_2')
                action_client = RclpyActionClient(client_node, RclpyFibonacci, 'fibonacci_zenoh')
                executor = SingleThreadedExecutor()
                executor.add_node(client_node)
                executor_thread = threading.Thread(target=executor.spin, daemon=True)
                executor_thread.start()
                
                try:
                    # wait_for_server handles discovery
                    if not action_client.wait_for_server(timeout_sec=5.0):
                        return False, "Server not found"
                    
                    # Send goal
                    goal_msg = RclpyFibonacci.Goal()
                    goal_msg.order = 10
                    future = action_client.send_goal_async(goal_msg)
                    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
                    goal_handle = future.result()
                    
                    if not goal_handle or not goal_handle.accepted:
                        return False, "Goal not accepted"
                    
                    # Cancel goal
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=5.0)
                    
                    return True, "Success"
                finally:
                    action_client.destroy()
                    executor.shutdown()
                    client_node.destroy_node()
            
            # Run in executor to avoid blocking asyncio
            success, msg = await asyncio.get_event_loop().run_in_executor(None, send_and_cancel_goal)
            assert success, msg
            
            # Actively wait for cancel (max 0.5s)
            for _ in range(25):
                if cancel_callback_called or cancel_requested_in_execute:
                    break
                await asyncio.sleep(0.02)
            
            # Verify cancel callback was invoked
            assert cancel_callback_called or cancel_requested_in_execute, "Cancel was not processed"
        finally:
            server.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
