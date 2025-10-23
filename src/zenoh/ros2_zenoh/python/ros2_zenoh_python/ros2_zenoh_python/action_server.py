"""
ROS2 Action Server implementation using Zenoh.

Actions in ROS2 are composed of:
- 3 Service Servers: send_goal, cancel_goal, get_result
- 2 Publishers: feedback, status

Based on rmw_zenoh design, actions are NOT RMW primitives but high-level
constructs built from existing services and topics.
"""

from __future__ import annotations
import asyncio
import logging
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import IntEnum
from typing import TYPE_CHECKING, Any, Callable, Optional, Type, Dict, List

if TYPE_CHECKING:
    from .node import Node

logger = logging.getLogger(__name__)


class GoalResponse(IntEnum):
    """Response codes for goal requests."""
    REJECT = 1
    ACCEPT_AND_EXECUTE = 2
    ACCEPT_AND_DEFER = 3
    ACCEPT = 2  # Alias for ACCEPT_AND_EXECUTE


class CancelResponse(IntEnum):
    """Response codes for cancel requests."""
    REJECT = 1
    ACCEPT = 2


class GoalStatus(IntEnum):
    """ROS2 action goal status codes (from action_msgs/msg/GoalStatus)."""
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


@dataclass
class GoalHandle:
    """Handle for tracking an action goal."""
    goal_id: Any  # UUID
    goal: Any  # Goal message
    status: GoalStatus = GoalStatus.STATUS_ACCEPTED
    result: Optional[Any] = None
    accepted: bool = False
    executing: bool = False
    canceling: bool = False
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _action_server: Optional['ActionServer'] = None
    
    def succeed(self):
        """Mark goal as succeeded."""
        with self._lock:
            self.status = GoalStatus.STATUS_SUCCEEDED
            self.executing = False
    
    def abort(self):
        """Mark goal as aborted."""
        with self._lock:
            self.status = GoalStatus.STATUS_ABORTED
            self.executing = False
    
    def canceled(self):
        """Mark goal as canceled."""
        with self._lock:
            self.status = GoalStatus.STATUS_CANCELED
            self.executing = False
            self.canceling = False
    
    def publish_feedback(self, feedback):
        """Publish feedback for this goal."""
        if self._action_server:
            self._action_server._publish_feedback(self.goal_id, feedback)
    
    @property
    def is_cancel_requested(self) -> bool:
        """Check if cancellation was requested."""
        with self._lock:
            return self.canceling


class ActionServer:
    """
    ROS2-compatible action server using Zenoh.
    
    Actions are composed of 3 services + 2 topics:
    - send_goal service: Accept/reject action goals
    - cancel_goal service: Handle cancellation requests
    - get_result service: Return final results
    - feedback topic: Publish progress updates
    - status topic: Publish goal status array
    
    Example:
        from ros2_zenoh_python import Node
        from example_interfaces.action import Fibonacci
        
        async def execute_fibonacci(goal_handle):
            result = Fibonacci.Result()
            result.sequence = [0, 1]
            
            for i in range(1, goal_handle.goal.order):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                
                result.sequence.append(
                    result.sequence[i] + result.sequence[i-1]
                )
                
                # Send feedback
                feedback = Fibonacci.Feedback()
                feedback.sequence = list(result.sequence)
                goal_handle.publish_feedback(feedback)
                
                await asyncio.sleep(0.5)
            
            goal_handle.succeed()
            return result
        
        async with Node('fibonacci_server') as node:
            action_server = node.create_action_server(
                Fibonacci,
                'fibonacci',
                execute_fibonacci
            )
            await node.spin()
    """
    
    def __init__(self, action_type: Type, action_name: str, 
                 execute_callback: Callable,
                 node: Node,
                 goal_callback: Optional[Callable] = None,
                 cancel_callback: Optional[Callable] = None):
        """
        Create an action server.
        
        Args:
            action_type: Action type class (e.g., Fibonacci)
            action_name: Name of the action (e.g., 'fibonacci')
            execute_callback: Async callback that executes the action.
                            Called with GoalHandle, should return Result.
            node: Parent node
            goal_callback: Optional callback to accept/reject goals.
                          If None, all goals are accepted.
            cancel_callback: Optional callback to accept/reject cancel requests.
                           If None, all cancel requests are accepted.
        """
        self.action_type = action_type
        self.action_name = action_name
        self.execute_callback = execute_callback
        self.goal_callback = goal_callback
        self.cancel_callback = cancel_callback
        self.node = node
        
        # Track active goals
        self._goals: Dict[bytes, GoalHandle] = {}
        self._goals_lock = threading.Lock()
        
        # Get CancelGoal service type from action_msgs
        from ._bundled_msgs import action_msgs
        self.cancel_goal_type = action_msgs.srv.CancelGoal
        
        # Create 3 service servers
        self._send_goal_service = node.create_service(
            action_type.SendGoal,
            f"{action_name}/_action/send_goal",
            self._handle_send_goal
        )
        
        self._cancel_goal_service = node.create_service(
            self.cancel_goal_type,
            f"{action_name}/_action/cancel_goal",
            self._handle_cancel_goal
        )
        
        self._get_result_service = node.create_service(
            action_type.GetResult,
            f"{action_name}/_action/get_result",
            self._handle_get_result
        )
        
        # Create 2 publishers
        self._feedback_pub = node.create_publisher(
            action_type.FeedbackMessage,
            f"{action_name}/_action/feedback"
        )
        
        self._status_pub = node.create_publisher(
            action_msgs.msg.GoalStatusArray,
            f"{action_name}/_action/status"
        )
        
        # Start status publishing thread
        self._status_thread_running = True
        self._status_thread = threading.Thread(target=self._status_publisher_thread, daemon=True)
        self._status_thread.start()
        
        logger.info(f"ActionServer created for '{action_name}'")
    
    async def _handle_send_goal(self, request):
        """Handle send_goal service request."""
        response = self.action_type.SendGoal.Response()
        
        # Extract goal_id and goal from request
        goal_id = request.goal_id
        goal = request.goal
        
        # Check if goal should be accepted
        accept = True
        if self.goal_callback:
            try:
                if asyncio.iscoroutinefunction(self.goal_callback):
                    goal_response = await self.goal_callback(goal)
                else:
                    goal_response = self.goal_callback(goal)
                
                # Handle both GoalResponse enum and boolean returns (for backward compatibility)
                if isinstance(goal_response, bool):
                    accept = goal_response
                elif isinstance(goal_response, int):
                    # GoalResponse can be REJECT (1), ACCEPT_AND_EXECUTE (2), or ACCEPT_AND_DEFER (3)
                    accept = goal_response in (GoalResponse.ACCEPT, GoalResponse.ACCEPT_AND_EXECUTE, GoalResponse.ACCEPT_AND_DEFER)
                else:
                    # Fallback: treat as truthy/falsy
                    accept = bool(goal_response)
            except Exception as e:
                logger.error(f"Goal callback error: {e}")
                accept = False
        
        response.accepted = accept
        
        # Import builtin_interfaces
        from ._bundled_msgs import builtin_interfaces
        response.stamp = builtin_interfaces.msg.Time()
        response.stamp.sec = int(time.time())
        response.stamp.nanosec = int((time.time() % 1) * 1e9)
        
        if accept:
            # Create goal handle
            goal_handle = GoalHandle(
                goal_id=goal_id,
                goal=goal,
                status=GoalStatus.STATUS_ACCEPTED,
                _action_server=self
            )
            goal_handle.accepted = True
            
            # Store goal
            goal_id_bytes = self._uuid_to_bytes(goal_id)
            with self._goals_lock:
                self._goals[goal_id_bytes] = goal_handle
            
            # Start executing goal in background
            asyncio.create_task(self._execute_goal(goal_handle))
            
            logger.debug(f"Goal accepted: {goal_id_bytes.hex()}")
        else:
            logger.debug(f"Goal rejected")
        
        return response
    
    async def _execute_goal(self, goal_handle: GoalHandle):
        """Execute the action goal."""
        try:
            # Update status to EXECUTING
            with goal_handle._lock:
                goal_handle.status = GoalStatus.STATUS_EXECUTING
                goal_handle.executing = True
            
            # Execute user callback
            if asyncio.iscoroutinefunction(self.execute_callback):
                result = await self.execute_callback(goal_handle)
            else:
                result = self.execute_callback(goal_handle)
            
            # Store result
            goal_handle.result = result
            
            # If not already terminated, mark as succeeded
            with goal_handle._lock:
                if goal_handle.status == GoalStatus.STATUS_EXECUTING:
                    goal_handle.status = GoalStatus.STATUS_SUCCEEDED
                goal_handle.executing = False
            
            logger.debug(f"Goal completed with status: {goal_handle.status.name}")
            
        except Exception as e:
            logger.error(f"Goal execution error: {e}", exc_info=True)
            goal_handle.abort()
    
    async def _handle_cancel_goal(self, request):
        """Handle cancel_goal service request."""
        response = self.cancel_goal_type.Response()
        
        # Extract goal_id from request
        goal_id = request.goal_info.goal_id
        goal_id_bytes = self._uuid_to_bytes(goal_id)
        
        # Find goal
        with self._goals_lock:
            goal_handle = self._goals.get(goal_id_bytes)
        
        if goal_handle is None:
            # Goal not found
            response.return_code = self.cancel_goal_type.Response.ERROR_UNKNOWN_GOAL_ID
            response.goals_canceling = []
            return response
        
        # Check if cancel should be accepted
        accept = True
        if self.cancel_callback:
            try:
                if asyncio.iscoroutinefunction(self.cancel_callback):
                    accept = await self.cancel_callback(goal_handle)
                else:
                    accept = self.cancel_callback(goal_handle)
            except Exception as e:
                logger.error(f"Cancel callback error: {e}")
                accept = False
        
        if not accept:
            response.return_code = self.cancel_goal_type.Response.ERROR_REJECTED
            response.goals_canceling = []
            return response
        
        # Mark goal as canceling
        with goal_handle._lock:
            if goal_handle.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                goal_handle.status = GoalStatus.STATUS_CANCELING
                goal_handle.canceling = True
                
                from ._bundled_msgs import action_msgs
                goal_info = action_msgs.msg.GoalInfo()
                goal_info.goal_id = goal_id
                goal_info.stamp = request.goal_info.stamp
                
                response.return_code = self.cancel_goal_type.Response.ERROR_NONE
                response.goals_canceling = [goal_info]
            else:
                response.return_code = self.cancel_goal_type.Response.ERROR_GOAL_TERMINATED
                response.goals_canceling = []
        
        return response
    
    async def _handle_get_result(self, request):
        """Handle get_result service request."""
        response = self.action_type.GetResult.Response()
        
        # Extract goal_id
        goal_id = request.goal_id
        goal_id_bytes = self._uuid_to_bytes(goal_id)
        
        # Find goal
        with self._goals_lock:
            goal_handle = self._goals.get(goal_id_bytes)
        
        if goal_handle is None:
            # Goal not found - return unknown status
            response.status = GoalStatus.STATUS_UNKNOWN
            response.result = self.action_type.Result()
            return response
        
        # Wait for goal to finish if still executing
        max_wait = 10.0  # seconds
        wait_start = time.time()
        while goal_handle.executing and (time.time() - wait_start) < max_wait:
            await asyncio.sleep(0.01)
        
        # Return result
        response.status = int(goal_handle.status)
        response.result = goal_handle.result if goal_handle.result else self.action_type.Result()
        
        return response
    
    def _publish_feedback(self, goal_id, feedback):
        """Publish feedback for a goal."""
        feedback_msg = self.action_type.FeedbackMessage()
        feedback_msg.goal_id = goal_id
        feedback_msg.feedback = feedback
        
        self._feedback_pub.publish(feedback_msg)
    
    def _status_publisher_thread(self):
        """Background thread to publish status updates."""
        # Import action_msgs and builtin_interfaces
        from ._bundled_msgs import action_msgs, builtin_interfaces
        
        while self._status_thread_running:
            try:
                # Build status array
                status_array = action_msgs.msg.GoalStatusArray()
                
                with self._goals_lock:
                    status_list = []
                    for goal_id_bytes, goal_handle in self._goals.items():
                        status_msg = action_msgs.msg.GoalStatus()
                        
                        status_msg.goal_info = action_msgs.msg.GoalInfo()
                        status_msg.goal_info.goal_id = goal_handle.goal_id
                        
                        # Set timestamp
                        status_msg.goal_info.stamp = builtin_interfaces.msg.Time()
                        status_msg.goal_info.stamp.sec = int(time.time())
                        status_msg.goal_info.stamp.nanosec = int((time.time() % 1) * 1e9)
                        
                        status_msg.status = int(goal_handle.status)
                        status_list.append(status_msg)
                    
                    status_array.status_list = status_list
                
                # Publish status
                self._status_pub.publish(status_array)
                
                time.sleep(0.5)  # Publish at 2 Hz
                
            except Exception as e:
                logger.error(f"Status publisher error: {e}")
                time.sleep(1.0)
    
    def _uuid_to_bytes(self, uuid_msg) -> bytes:
        """Convert UUID message to bytes."""
        # UUID message has a 'uuid' field that is a 16-byte array
        return bytes(uuid_msg.uuid)
    
    def destroy(self):
        """Destroy the action server."""
        if hasattr(self, '_destroyed') and self._destroyed:
            return  # Already destroyed
        
        # Stop status thread
        self._status_thread_running = False
        if self._status_thread.is_alive():
            self._status_thread.join(timeout=2.0)
        
        # Destroy services and publishers
        # (Node will handle cleanup on shutdown)
        
        self._destroyed = True
        logger.info(f"ActionServer '{self.action_name}' destroyed")


__all__ = ['ActionServer', 'GoalHandle', 'GoalStatus']

