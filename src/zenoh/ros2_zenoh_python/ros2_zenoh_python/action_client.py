"""
ROS2 Action Client implementation using Zenoh.

Actions in ROS2 are composed of:
- 3 Service Clients: send_goal, cancel_goal, get_result
- 2 Subscribers: feedback, status

Based on rmw_zenoh design, actions are NOT RMW primitives but high-level
constructs built from existing clients and subscriptions.
"""

from __future__ import annotations
import asyncio
import logging
import uuid
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Optional, Type

if TYPE_CHECKING:
    from .node import Node

from .action_server import GoalStatus

logger = logging.getLogger(__name__)


@dataclass
class ClientGoalHandle:
    """Client-side handle for tracking an action goal."""
    goal_id: Any  # UUID
    goal: Any  # Goal message
    status: GoalStatus = GoalStatus.STATUS_UNKNOWN
    accepted: bool = False
    
    def __init__(self, goal_id, goal):
        self.goal_id = goal_id
        self.goal = goal
        self.status = GoalStatus.STATUS_UNKNOWN
        self.accepted = False


class ActionClient:
    """
    ROS2-compatible action client using Zenoh.
    
    Actions are composed of 3 service clients + 2 subscriptions:
    - send_goal client: Submit action goals
    - cancel_goal client: Request goal cancellation
    - get_result client: Retrieve final results
    - feedback subscription: Receive progress updates
    - status subscription: Monitor goal status
    
    Example:
        from ros2_zenoh_python import Node
        from ros2_zenoh_python._bundled_msgs import example_interfaces
        
        async def feedback_callback(feedback_msg):
            print(f"Feedback: {feedback_msg.feedback.sequence}")
        
        async def main():
            async with Node('fibonacci_client') as node:
                client = node.create_action_client(
                    example_interfaces.action.Fibonacci,
                    'fibonacci'
                )
                
                # Send goal
                goal = example_interfaces.action.Fibonacci.Goal()
                goal.order = 10
                
                goal_handle = await client.send_goal_async(
                    goal,
                    feedback_callback=feedback_callback
                )
                
                if not goal_handle.accepted:
                    print("Goal rejected")
                    return
                
                # Wait for result
                result = await client.get_result_async(goal_handle)
                print(f"Result: {result.sequence}")
        
        asyncio.run(main())
    """
    
    def __init__(self, action_type: Type, action_name: str, node: Node):
        """
        Create an action client.
        
        Args:
            action_type: Action type class (e.g., Fibonacci)
            action_name: Name of the action (e.g., 'fibonacci')
            node: Parent node
        """
        self.action_type = action_type
        self.action_name = action_name
        self.node = node
        
        # Get CancelGoal service type from action_msgs
        from ._bundled_msgs import action_msgs
        self.cancel_goal_type = action_msgs.srv.CancelGoal
        
        # Create 3 service clients
        self._send_goal_client = node.create_client(
            action_type.SendGoal,
            f"{action_name}/_action/send_goal"
        )
        
        self._cancel_goal_client = node.create_client(
            self.cancel_goal_type,
            f"{action_name}/_action/cancel_goal"
        )
        
        self._get_result_client = node.create_client(
            action_type.GetResult,
            f"{action_name}/_action/get_result"
        )
        
        # Create 2 subscriptions
        self._feedback_callbacks = {}  # goal_id -> callback
        self._feedback_sub = node.create_subscription(
            action_type.FeedbackMessage,
            f"{action_name}/_action/feedback",
            self._feedback_callback
        )
        
        self._status_sub = node.create_subscription(
            action_msgs.msg.GoalStatusArray,
            f"{action_name}/_action/status",
            self._status_callback
        )
        
        # Track goal handles
        self._goal_handles = {}  # goal_id_bytes -> ClientGoalHandle
        
        logger.info(f"ActionClient created for '{action_name}'")
    
    async def send_goal_async(self, goal, feedback_callback: Optional[Callable] = None):
        """
        Send a goal to the action server.
        
        Args:
            goal: Goal message
            feedback_callback: Optional callback for feedback messages.
                             Called with FeedbackMessage as argument.
        
        Returns:
            ClientGoalHandle for tracking the goal
        """
        # Generate unique goal ID
        from ._bundled_msgs import unique_identifier_msgs
        goal_id = unique_identifier_msgs.msg.UUID()
        goal_uuid = uuid.uuid4()
        goal_id.uuid = list(goal_uuid.bytes)
        
        # Create request
        request = self.action_type.SendGoal.Request()
        request.goal_id = goal_id
        request.goal = goal
        
        # Register feedback callback
        goal_id_bytes = bytes(goal_id.uuid)
        if feedback_callback:
            self._feedback_callbacks[goal_id_bytes] = feedback_callback
        
        # Send goal
        response = await self._send_goal_client.call_async(request)
        
        # Return None if goal was rejected (consistent with rclpy behavior)
        if not response.accepted:
            logger.debug(f"Goal rejected")
            return None
        
        # Create goal handle for accepted goals
        goal_handle = ClientGoalHandle(goal_id, goal)
        goal_handle.accepted = True
        goal_handle.status = GoalStatus.STATUS_ACCEPTED
        self._goal_handles[goal_id_bytes] = goal_handle
        logger.debug(f"Goal accepted: {goal_id_bytes.hex()}")
        
        return goal_handle
    
    async def cancel_goal_async(self, goal_handle: ClientGoalHandle):
        """
        Request cancellation of a goal.
        
        Args:
            goal_handle: The goal handle to cancel
            
        Returns:
            CancelGoal response
        """
        from ._bundled_msgs import action_msgs
        
        request = self.cancel_goal_type.Request()
        request.goal_info = action_msgs.msg.GoalInfo()
        request.goal_info.goal_id = goal_handle.goal_id
        
        # Set timestamp
        from ._bundled_msgs import builtin_interfaces
        import time
        request.goal_info.stamp = builtin_interfaces.msg.Time()
        request.goal_info.stamp.sec = int(time.time())
        request.goal_info.stamp.nanosec = int((time.time() % 1) * 1e9)
        
        response = await self._cancel_goal_client.call_async(request)
        return response
    
    async def get_result_async(self, goal_handle: ClientGoalHandle):
        """
        Get the final result of a goal.
        
        This will block until the goal completes.
        
        Args:
            goal_handle: The goal handle
            
        Returns:
            Result message
        """
        request = self.action_type.GetResult.Request()
        request.goal_id = goal_handle.goal_id
        
        response = await self._get_result_client.call_async(request)
        
        # Update goal handle status
        goal_handle.status = GoalStatus(response.status)
        
        return response.result
    
    def _feedback_callback(self, feedback_msg):
        """Handle feedback messages."""
        goal_id_bytes = bytes(feedback_msg.goal_id.uuid)
        
        callback = self._feedback_callbacks.get(goal_id_bytes)
        if callback:
            if asyncio.iscoroutinefunction(callback):
                # Schedule on event loop if one exists
                try:
                    loop = asyncio.get_event_loop()
                    if loop.is_running():
                        asyncio.run_coroutine_threadsafe(callback(feedback_msg), loop)
                    else:
                        # No running loop, skip async callback
                        logger.warning("Async feedback callback skipped (no running event loop)")
                except RuntimeError:
                    logger.warning("Async feedback callback skipped (no event loop)")
            else:
                callback(feedback_msg)
    
    def _status_callback(self, status_array):
        """Handle status updates."""
        for status_msg in status_array.status_list:
            goal_id_bytes = bytes(status_msg.goal_info.goal_id.uuid)
            
            goal_handle = self._goal_handles.get(goal_id_bytes)
            if goal_handle:
                goal_handle.status = GoalStatus(status_msg.status)
                logger.debug(f"Goal {goal_id_bytes.hex()[:8]}... status: {goal_handle.status.name}")
    
    async def wait_for_action_server(self, timeout: float = 5.0) -> bool:
        """
        Wait for action server to be available.
        
        Checks for the send_goal service to determine if server is ready.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if server is found, False if timeout
        """
        # Use the send_goal client's wait_for_server method
        return await self._send_goal_client.wait_for_server(timeout=timeout)
    
    def destroy(self):
        """Destroy the action client."""
        self._feedback_callbacks.clear()
        self._goal_handles.clear()
        logger.info(f"ActionClient '{self.action_name}' destroyed")


__all__ = ['ActionClient', 'ClientGoalHandle']

