#!/usr/bin/env python3
"""
FastAPI backend for ROS2 Web UI
Provides REST API and WebSocket for discovering and managing ROS2 actions
"""

import asyncio
import logging
import uuid
import json
from typing import List, Dict, Any, Optional, Set
from contextlib import asynccontextmanager
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.executors import SingleThreadedExecutor
import threading

from fastapi import (
    FastAPI,
    HTTPException,
    BackgroundTasks,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ROS2 interface parsing imports
from rosidl_runtime_py import get_interface_path
from rosidl_adapter.parser import (
    parse_message_string,
    ACTION_REQUEST_RESPONSE_SEPARATOR,
    Field,
    MessageSpecification,
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class InterfaceTextLine:
    """A convenience class for a single text line in an interface file."""

    def __init__(
        self,
        pkg_name: str,
        msg_name: str,
        line_text: str,
    ):
        if line_text in (ACTION_REQUEST_RESPONSE_SEPARATOR,):
            msg_spec = None
        else:
            msg_spec = parse_message_string(
                pkg_name=pkg_name,
                msg_name=msg_name,
                message_string=line_text,
            )
            if msg_spec and len(msg_spec.fields) > 1:
                raise ValueError("'line_text' must be only one line")
        self._msg_spec: Optional[MessageSpecification] = msg_spec
        self._raw_line_text = line_text

    def __str__(self) -> str:
        return self._raw_line_text

    def is_comment(self) -> bool:
        return self._msg_spec and self._msg_spec.annotations.get("comment", False)

    def is_trailing_comment(self) -> bool:
        return self._is_field_trailing_comment()

    def _is_field_trailing_comment(self) -> bool:
        return self._field and self._field.annotations.get("comment", False)

    @property
    def trailing_comment(self) -> Optional[str]:
        if self._is_field_trailing_comment():
            return self._field.annotations["comment"][0]
        else:
            return None

    @property
    def _field(self) -> Optional[Field]:
        if self._msg_spec and self._msg_spec.fields:
            return self._msg_spec.fields[0]

    @property
    def field_name(self) -> Optional[str]:
        if self._field:
            return self._field.name
        return None

    @property
    def field_type(self) -> Optional[str]:
        if self._field:
            return str(self._field.type)
        return None


# Pydantic models for API responses
class ActionGoalField(BaseModel):
    name: str
    type: str
    description: str = ""


class ActionInfo(BaseModel):
    name: str
    type: str
    description: str
    goal: List[ActionGoalField]
    available: bool
    namespace: str = ""


class ActionListResponse(BaseModel):
    actions: List[ActionInfo]
    total_count: int


class ActionSendGoalRequest(BaseModel):
    parameters: Dict[str, Any]


class ActionSendGoalResponse(BaseModel):
    success: bool
    message: str
    goal_id: Optional[str] = None
    accepted: Optional[bool] = None
    rejection_reason: Optional[str] = None


class ActionCancelGoalResponse(BaseModel):
    success: bool
    message: str
    goal_id: str


class WebSocketEventBus:
    """Manages WebSocket connections and event broadcasting"""

    def __init__(self):
        self.connections: Set[WebSocket] = set()
        self.subscriptions: Dict[WebSocket, Set[str]] = defaultdict(set)

    async def connect(self, websocket: WebSocket):
        """Add a new WebSocket connection"""
        await websocket.accept()
        self.connections.add(websocket)
        logger.info(f"WebSocket connected. Total connections: {len(self.connections)}")

    def disconnect(self, websocket: WebSocket):
        """Remove a WebSocket connection"""
        self.connections.discard(websocket)
        if websocket in self.subscriptions:
            del self.subscriptions[websocket]
        logger.info(
            f"WebSocket disconnected. Total connections: {len(self.connections)}"
        )

    def subscribe(self, websocket: WebSocket, event_types: List[str]):
        """Subscribe a WebSocket to specific event types"""
        self.subscriptions[websocket].update(event_types)
        logger.debug(f"WebSocket subscribed to: {event_types}")

    def unsubscribe(self, websocket: WebSocket, event_types: List[str]):
        """Unsubscribe a WebSocket from specific event types"""
        self.subscriptions[websocket].difference_update(event_types)
        logger.debug(f"WebSocket unsubscribed from: {event_types}")

    async def broadcast(self, event_type: str, data: Any):
        """Broadcast an event to all subscribed WebSocket connections"""
        if not self.connections:
            return

        message = {
            "type": event_type,
            "data": data,
            "timestamp": asyncio.get_event_loop().time(),
        }

        # Find connections subscribed to this event type
        target_connections = [
            ws
            for ws in self.connections
            if event_type in self.subscriptions.get(ws, set())
        ]

        if not target_connections:
            return

        # Send to all subscribed connections
        disconnected = []
        for websocket in target_connections:
            try:
                await websocket.send_json(message)
            except Exception as e:
                logger.warning(f"Failed to send WebSocket message: {e}")
                disconnected.append(websocket)

        # Clean up disconnected WebSockets
        for ws in disconnected:
            self.disconnect(ws)


class ROS2ActionManager(Node):
    """ROS2 node for discovering and calling actions"""

    def __init__(self, event_bus: WebSocketEventBus):
        super().__init__("web_bridge_action_manager")
        self.logger = self.get_logger()
        self.logger.info("Initializing ROS2 Action Manager")

        # WebSocket event bus for real-time updates
        self.event_bus = event_bus

        # Cache for discovered actions
        self._action_cache = {}
        self._last_discovery_time = 0
        self._cache_duration = 5.0  # Cache for 5 seconds

        # Cache for action descriptions
        self._description_cache = {}

        # Cache for action goal fields
        self._goal_fields_cache = {}

        # Active action clients
        self._action_clients = {}

        # Active goals tracking
        self._active_goals = {}

    def discover_actions(self) -> List[ActionInfo]:
        """Discover all available ROS2 actions"""
        import time

        current_time = time.time()

        # Return cached results if still valid
        if (current_time - self._last_discovery_time) < self._cache_duration:
            return list(self._action_cache.values())

        self.logger.info("Discovering available ROS2 actions...")

        # Get list of action names and types using ROS2 API
        action_names_and_types = get_action_names_and_types(self)

        discovered_actions = []

        for action_name, action_types in action_names_and_types:
            # action_types is a list, get the first (and typically only) type
            if action_types:  # Check if list is not empty
                action_type = action_types[0]  # Get first action type
                # Parse action information
                action_info = self._parse_action_info(action_name, action_type)
                if action_info:
                    discovered_actions.append(action_info)
            else:
                self.logger.warning(f"Action {action_name} has no types, skipping")

        # Update cache
        self._action_cache = {action.name: action for action in discovered_actions}
        self._last_discovery_time = current_time

        self.logger.info(f"Discovered {len(discovered_actions)} actions")
        return discovered_actions

    def _parse_action_info(
        self, action_name: str, action_type: str
    ) -> Optional[ActionInfo]:
        """Parse action information from ROS2 action type"""
        try:
            # Extract namespace and action name
            namespace = ""
            clean_name = action_name
            if action_name.startswith("/"):
                parts = action_name.split("/")
                if len(parts) > 2:
                    namespace = "/".join(parts[1:-1])
                    clean_name = parts[-1]
                else:
                    clean_name = parts[-1]

            # Get action goal fields from action definition
            goal_fields = self._get_action_goal_fields(action_type)

            # Check if action server is available
            available = self._check_action_availability(action_name, action_type)

            return ActionInfo(
                name=clean_name,
                type=action_type,
                description=self._get_action_description(action_type),
                goal=goal_fields,
                available=available,
                namespace=namespace,
            )

        except Exception as e:
            self.logger.error(f"Error parsing action {action_name}: {e}")
            return None

    def _get_action_goal_fields(self, action_type: str) -> List[ActionGoalField]:
        """Get goal fields dynamically from action definition files"""
        # Try dynamic extraction first
        dynamic_goal_fields = self._extract_dynamic_goal_fields(action_type)
        if dynamic_goal_fields:
            return dynamic_goal_fields

        # Fallback: return generic goal field
        return [
            ActionGoalField(
                name="goal", type="unknown", description="Action goal parameters"
            )
        ]

    def _extract_dynamic_goal_fields(
        self, action_type: str
    ) -> Optional[List[ActionGoalField]]:
        """Extract goal fields dynamically from action definition file"""
        try:
            # Check cache first
            if action_type in self._goal_fields_cache:
                return self._goal_fields_cache[action_type]

            # Get the interface file path
            file_path = get_interface_path(action_type)

            # Parse action type
            parts = action_type.split("/")
            if len(parts) != 3:
                return None
            pkg_name, _, msg_name = parts

            goal_fields = []

            # Read and parse the action file
            with open(file_path, "r") as file_handler:
                lines = file_handler.readlines()

            # Parse goal section only (before first separator)
            in_goal_section = False
            for line in lines:
                line_stripped = line.strip()

                # Skip empty lines and pure comments
                if not line_stripped or (
                    line_stripped.startswith("#")
                    and not any(c.isalnum() for c in line_stripped[1:])
                ):
                    continue

                # Check if we've reached the goal definition section
                if line_stripped == "# goal definition" or (
                    not line_stripped.startswith("#") and not in_goal_section
                ):
                    in_goal_section = True
                    if not line_stripped.startswith("#"):
                        # This is the first field line, process it
                        pass
                    else:
                        continue

                # Stop at separator (end of goal section)
                if line_stripped == ACTION_REQUEST_RESPONSE_SEPARATOR:
                    break

                # Skip if not in goal section yet
                if not in_goal_section:
                    continue

                # Parse field lines using InterfaceTextLine
                try:
                    interface_line = InterfaceTextLine(
                        pkg_name=pkg_name,
                        msg_name=msg_name,
                        line_text=line.rstrip(),
                    )

                    # Extract field information
                    if interface_line.field_name and interface_line.field_type:
                        description = ""
                        if interface_line.trailing_comment:
                            description = interface_line.trailing_comment

                        goal_fields.append(
                            ActionGoalField(
                                name=interface_line.field_name,
                                type=interface_line.field_type,
                                description=description,
                            )
                        )

                except Exception as e:
                    # Skip lines that can't be parsed as fields
                    self.logger.debug(f"Could not parse line '{line.strip()}': {e}")
                    continue

            # Cache the result
            self._goal_fields_cache[action_type] = goal_fields

            if goal_fields:
                self.logger.debug(
                    f"Extracted {len(goal_fields)} goal fields for {action_type}: {[f.name for f in goal_fields]}"
                )

            return goal_fields

        except Exception as e:
            self.logger.warning(f"Could not extract goal fields for {action_type}: {e}")
            return None

    def _extract_dynamic_description(self, action_type: str) -> Optional[str]:
        """Extract description dynamically from action definition file"""
        try:
            # Check cache first
            if action_type in self._description_cache:
                return self._description_cache[action_type]

            # Get the interface file path
            file_path = get_interface_path(action_type)

            # Parse action type for logging
            parts = action_type.split("/")
            if len(parts) != 3:
                return None
            pkg_name, _, msg_name = parts

            description = None

            # Read and parse the action file
            with open(file_path, "r") as file_handler:
                lines = file_handler.readlines()

                # Look for description in the first few comment lines
                for i, line in enumerate(lines[:10]):  # Check first 10 lines
                    line = line.strip()

                    # Skip empty lines
                    if not line:
                        continue

                    # Look for comment lines that contain action description
                    if line.startswith("#") and ":" in line:
                        # Extract description from lines like "# Move action: Description here"
                        if "action:" in line.lower():
                            description = line.split(":", 1)[1].strip()
                            break

                    # Stop at first non-comment line (start of goal definition)
                    if not line.startswith("#"):
                        break

                # If no description found in header, look for goal section comments
                if not description:
                    in_goal_section = True
                    for line in lines:
                        line_stripped = line.strip()

                        # Stop at first separator (end of goal section)
                        if line_stripped == ACTION_REQUEST_RESPONSE_SEPARATOR:
                            break

                        # Look for meaningful comments in goal section
                        if line_stripped.startswith("#") and len(line_stripped) > 2:
                            comment_text = line_stripped[1:].strip()
                            # Skip generic comments like "goal definition"
                            if (
                                comment_text.lower()
                                not in ["goal definition", "goal", "request"]
                                and len(comment_text) > 10
                            ):
                                description = comment_text
                                break

            # Cache the result (even if None)
            self._description_cache[action_type] = description

            if description:
                self.logger.debug(
                    f"Extracted description for {action_type}: {description}"
                )

            return description

        except Exception as e:
            self.logger.warning(f"Could not extract description for {action_type}: {e}")
            return None

    def _generate_description_from_name(self, action_type: str) -> str:
        """Generate a description from the action type name"""
        try:
            parts = action_type.split("/")
            if len(parts) == 3:
                action_name = parts[2]
                # Convert CamelCase to readable format
                import re

                readable_name = re.sub(r"([A-Z])", r" \1", action_name).strip()
                return f"{readable_name} action"
            return f"Action of type {action_type}"
        except Exception:
            return f"Action of type {action_type}"

    def _get_action_description(self, action_type: str) -> str:
        """Get description for action types using dynamic extraction with fallbacks"""
        # Try dynamic extraction first
        dynamic_description = self._extract_dynamic_description(action_type)
        if dynamic_description:
            return dynamic_description

        # Fallback: generate from action name
        return self._generate_description_from_name(action_type)

    def _check_action_availability(self, action_name: str, action_type: str) -> bool:
        """Check if action server is currently available"""
        try:
            # Import the action type dynamically
            action_module = self._import_action_type(action_type)
            if not action_module:
                return False

            # Create a temporary action client to check availability
            client = ActionClient(self, action_module, action_name)
            available = client.wait_for_server(timeout_sec=1.0)
            client.destroy()

            return available

        except Exception as e:
            self.logger.warning(f"Could not check availability for {action_name}: {e}")
            return False

    def _import_action_type(self, action_type: str):
        """Dynamically import action type"""
        try:
            # Parse action type (e.g., "my_first_interfaces/action/Rotate")
            parts = action_type.split("/")
            if len(parts) != 3:
                return None

            package_name, action_dir, action_name = parts

            # Import the action module
            module_name = f"{package_name}.action"
            module = __import__(module_name, fromlist=[action_name])
            return getattr(module, action_name)

        except Exception as e:
            self.logger.warning(f"Could not import action type {action_type}: {e}")
            return None

    async def send_action_goal(
        self, action_name: str, action_type: str, parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Send a goal to a ROS2 action and return goal info with acceptance status"""
        try:
            # Import the action type
            action_module = self._import_action_type(action_type)
            if not action_module:
                raise ValueError(f"Could not import action type: {action_type}")

            # Create or get existing action client
            client_key = f"{action_name}_{action_type}"
            if client_key not in self._action_clients:
                self._action_clients[client_key] = ActionClient(
                    self, action_module, action_name
                )
                self.logger.info(
                    f"Created action client for {action_name} ({action_type})"
                )

            client = self._action_clients[client_key]

            # Wait for server to be available
            if not client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"Action server {action_name} is not available")

            # Build the goal message
            goal_msg = self._build_goal_message(action_module, parameters)

            # Generate unique goal ID
            goal_id = str(uuid.uuid4())

            # Send the goal asynchronously
            future = client.send_goal_async(goal_msg)

            # Store the goal for tracking
            self._active_goals[goal_id] = {
                "client": client,
                "future": future,
                "action_name": action_name,
                "action_type": action_type,
                "parameters": parameters,
                "status": "pending",
            }

            self.logger.info(f"Sent goal {goal_id} to action {action_name}")

            # Broadcast goal added event
            await self._emit_goal_added(goal_id)

            # Wait for goal acceptance/rejection with timeout
            try:
                # Convert ROS2 future to asyncio future for awaiting
                goal_handle = await self._wait_for_future(future, timeout=10.0)

                if goal_handle.accepted:
                    # Goal was accepted, update status and track result
                    self._active_goals[goal_id]["status"] = "accepted"
                    self._active_goals[goal_id]["goal_handle"] = goal_handle

                    self.logger.info(f"Goal {goal_id} was accepted")

                    # Emit goal update event
                    status = {
                        "goal_id": goal_id,
                        "action_name": action_name,
                        "action_type": action_type,
                        "status": "accepted",
                    }
                    await self._emit_goal_update(status)

                    # Attach callback to handle goal completion
                    if hasattr(goal_handle, "get_result_async"):
                        result_future = goal_handle.get_result_async()
                        self._active_goals[goal_id]["result_future"] = result_future
                        result_future.add_done_callback(
                            lambda f: self._handle_goal_completion(goal_id, f)
                        )

                    return {"goal_id": goal_id, "accepted": True, "status": "accepted"}
                else:
                    # Goal was rejected, extract rejection reason from goal handle
                    rejection_reason = self._extract_rejection_reason(goal_handle)
                    self.logger.info(f"Goal {goal_id} was rejected: {rejection_reason}")

                    status = {
                        "goal_id": goal_id,
                        "action_name": action_name,
                        "action_type": action_type,
                        "status": "rejected",
                    }

                    # Emit events and remove goal
                    await self._emit_goal_update(status)
                    await self._emit_goal_removed(goal_id)

                    # Remove from active goals
                    if goal_id in self._active_goals:
                        del self._active_goals[goal_id]

                    return {
                        "goal_id": goal_id,
                        "accepted": False,
                        "status": "rejected",
                        "rejection_reason": rejection_reason,
                    }

            except asyncio.TimeoutError:
                # Timeout waiting for goal response
                self.logger.error(f"Timeout waiting for goal {goal_id} response")

                status = {
                    "goal_id": goal_id,
                    "action_name": action_name,
                    "action_type": action_type,
                    "status": "timeout",
                }

                # Emit events and remove goal
                await self._emit_goal_update(status)
                await self._emit_goal_removed(goal_id)

                # Remove from active goals
                if goal_id in self._active_goals:
                    del self._active_goals[goal_id]

                return {
                    "goal_id": goal_id,
                    "accepted": False,
                    "status": "timeout",
                    "rejection_reason": "Timeout waiting for goal acceptance",
                }

        except Exception as e:
            self.logger.error(f"Error sending goal to action {action_name}: {e}")
            raise

    async def _wait_for_future(self, future, timeout: float = 10.0):
        """Convert ROS2 future to asyncio awaitable with timeout"""
        import concurrent.futures

        # Get the current asyncio event loop (FastAPI's loop)
        current_loop = asyncio.get_running_loop()

        # Create an asyncio event to signal completion
        done_event = asyncio.Event()
        result_container = {"result": None, "exception": None}

        def on_done(f):
            try:
                result_container["result"] = f.result()
            except Exception as e:
                result_container["exception"] = e
            # Use the captured loop to signal completion from ROS2 thread
            current_loop.call_soon_threadsafe(done_event.set)

        # Add callback to the ROS2 future
        future.add_done_callback(on_done)

        # Wait for completion or timeout
        try:
            await asyncio.wait_for(done_event.wait(), timeout=timeout)

            if result_container["exception"]:
                raise result_container["exception"]

            return result_container["result"]

        except asyncio.TimeoutError:
            self.logger.warning(f"Future timed out after {timeout} seconds")
            raise

    async def _emit_goal_added(self, goal_id: str):
        """Emit WebSocket event when a new goal is added"""
        try:
            status = self.get_goal_status(goal_id)
            if status:
                await self.event_bus.broadcast("goal_added", status)
        except Exception as e:
            self.logger.warning(f"Failed to emit goal_added event: {e}")

    async def _emit_goal_update(self, goal_status: Dict[str, Any]):
        """Emit WebSocket event when a goal status changes"""
        try:
            await self.event_bus.broadcast("goal_update", goal_status)
        except Exception as e:
            self.logger.warning(f"Failed to emit goal_update event: {e}")

    async def _emit_goal_removed(self, goal_id: str):
        """Emit WebSocket event when a goal is removed"""
        try:
            await self.event_bus.broadcast("goal_removed", {"goal_id": goal_id})
        except Exception as e:
            self.logger.warning(f"Failed to emit goal_removed event: {e}")

    def _handle_goal_response(self, goal_id: str, future):
        """Handle goal acceptance/rejection response"""
        try:
            if goal_id not in self._active_goals:
                self.logger.warning(f"Goal {goal_id} not found when handling response")
                return

            goal_info = self._active_goals[goal_id]

            try:
                goal_handle = future.result()
                if goal_handle.accepted:
                    # Goal was accepted, update status and track result
                    goal_info["status"] = "accepted"
                    goal_info["goal_handle"] = goal_handle

                    self.logger.info(f"Goal {goal_id} was accepted")

                    # Emit goal update event
                    status = {
                        "goal_id": goal_id,
                        "action_name": goal_info["action_name"],
                        "action_type": goal_info["action_type"],
                        "status": "accepted",
                    }
                    self._schedule_async_task(self._emit_goal_update(status))

                    # Attach callback to handle goal completion
                    if hasattr(goal_handle, "get_result_async"):
                        result_future = goal_handle.get_result_async()
                        goal_info["result_future"] = result_future
                        result_future.add_done_callback(
                            lambda f: self._handle_goal_completion(goal_id, f)
                        )
                else:
                    # Goal was rejected, remove from tracking
                    self.logger.info(f"Goal {goal_id} was rejected")

                    status = {
                        "goal_id": goal_id,
                        "action_name": goal_info["action_name"],
                        "action_type": goal_info["action_type"],
                        "status": "rejected",
                    }

                    # Emit events and remove goal
                    self._schedule_async_task(self._emit_goal_update(status))
                    self._schedule_async_task(self._emit_goal_removed(goal_id))

                    # Remove from active goals
                    if goal_id in self._active_goals:
                        del self._active_goals[goal_id]

            except Exception as e:
                # Error occurred during goal submission
                self.logger.error(f"Error in goal response for {goal_id}: {e}")

                status = {
                    "goal_id": goal_id,
                    "action_name": goal_info["action_name"],
                    "action_type": goal_info["action_type"],
                    "status": "error",
                    "error": str(e),
                }

                # Emit events and remove goal
                self._schedule_async_task(self._emit_goal_update(status))
                self._schedule_async_task(self._emit_goal_removed(goal_id))

                # Remove from active goals
                if goal_id in self._active_goals:
                    del self._active_goals[goal_id]

        except Exception as e:
            self.logger.error(
                f"Unexpected error in _handle_goal_response for {goal_id}: {e}"
            )

    def _schedule_async_task(self, coro):
        """Schedule an async task in a thread-safe way"""
        try:
            # Try to get the current event loop
            loop = asyncio.get_event_loop()
            if loop.is_running():
                # If we're in an async context, schedule the task
                asyncio.create_task(coro)
            else:
                # If no loop is running, run it in a new thread
                import threading

                def run_async():
                    asyncio.run(coro)

                thread = threading.Thread(target=run_async, daemon=True)
                thread.start()
        except RuntimeError:
            # No event loop in current thread, run in a new thread
            import threading

            def run_async():
                asyncio.run(coro)

            thread = threading.Thread(target=run_async, daemon=True)
            thread.start()

    def _handle_goal_completion(self, goal_id: str, result_future):
        """Handle goal completion (success or failure)"""
        try:
            if goal_id not in self._active_goals:
                self.logger.warning(
                    f"Goal {goal_id} not found when handling completion"
                )
                return

            goal_info = self._active_goals[goal_id]

            try:
                result = result_future.result()
                # Goal completed successfully
                self.logger.info(f"Goal {goal_id} completed successfully")

                status = {
                    "goal_id": goal_id,
                    "action_name": goal_info["action_name"],
                    "action_type": goal_info["action_type"],
                    "status": "completed",
                }

                # Emit events and remove goal
                self._schedule_async_task(self._emit_goal_update(status))
                self._schedule_async_task(self._emit_goal_removed(goal_id))

            except Exception as e:
                # Goal failed
                self.logger.info(f"Goal {goal_id} failed: {e}")

                status = {
                    "goal_id": goal_id,
                    "action_name": goal_info["action_name"],
                    "action_type": goal_info["action_type"],
                    "status": "failed",
                    "error": str(e),
                }

                # Emit events and remove goal
                self._schedule_async_task(self._emit_goal_update(status))
                self._schedule_async_task(self._emit_goal_removed(goal_id))

            # Remove from active goals in all cases
            if goal_id in self._active_goals:
                del self._active_goals[goal_id]

        except Exception as e:
            self.logger.error(
                f"Unexpected error in _handle_goal_completion for {goal_id}: {e}"
            )

    def _build_goal_message(self, action_module, parameters: Dict[str, Any]):
        """Build a goal message from parameters"""
        try:
            # Create an instance of the Goal message
            goal_msg = action_module.Goal()

            # Set parameters based on the goal fields
            for param_name, param_value in parameters.items():
                if hasattr(goal_msg, param_name):
                    # Convert parameter value to appropriate type
                    converted_value = self._convert_parameter_value(
                        goal_msg, param_name, param_value
                    )
                    setattr(goal_msg, param_name, converted_value)
                    self.logger.debug(
                        f"Set goal parameter {param_name} = {converted_value}"
                    )
                else:
                    self.logger.warning(
                        f"Goal message does not have parameter: {param_name}"
                    )

            return goal_msg

        except Exception as e:
            self.logger.error(f"Error building goal message: {e}")
            raise

    def _convert_parameter_value(
        self, goal_msg, param_name: str, param_value: Any
    ) -> Any:
        """Convert parameter value to the appropriate type for the goal message"""
        try:
            # Get the field type from the goal message
            field_type = type(getattr(goal_msg, param_name))

            # Convert based on the target type
            if field_type == bool:
                if isinstance(param_value, str):
                    return param_value.lower() in ("true", "1", "yes", "on")
                return bool(param_value)
            elif field_type == int:
                return int(float(param_value))  # Handle string numbers
            elif field_type == float:
                return float(param_value)
            elif field_type == str:
                return str(param_value)
            else:
                # For other types, try direct assignment
                return param_value

        except Exception as e:
            self.logger.warning(
                f"Could not convert parameter {param_name} value {param_value}: {e}"
            )
            return param_value

    def _extract_rejection_reason(self, goal_handle) -> str:
        """Extract rejection reason from ROS2 goal handle"""
        try:
            # Try to get rejection reason from various possible sources
            rejection_reason = "Goal was rejected by action server"

            # Check if goal handle has a status message or error information
            if hasattr(goal_handle, "status"):
                # ROS2 goal handles may have status information
                status = goal_handle.status
                if hasattr(status, "text") and status.text:
                    rejection_reason = status.text
                elif hasattr(status, "message") and status.message:
                    rejection_reason = status.message
                elif hasattr(status, "error_message") and status.error_message:
                    rejection_reason = status.error_message

            # Check if there's a goal_id with status information
            if hasattr(goal_handle, "goal_id") and hasattr(goal_handle.goal_id, "uuid"):
                # Sometimes rejection info is in the goal_id metadata
                pass  # This is mainly for identification, not rejection reasons

            # Check for any error or rejection message attributes
            for attr_name in ["rejection_reason", "error_message", "message", "reason"]:
                if hasattr(goal_handle, attr_name):
                    attr_value = getattr(goal_handle, attr_name)
                    if (
                        attr_value
                        and isinstance(attr_value, str)
                        and attr_value.strip()
                    ):
                        rejection_reason = attr_value.strip()
                        break

            # Log the extracted reason for debugging
            self.logger.debug(f"Extracted rejection reason: {rejection_reason}")

            return rejection_reason

        except Exception as e:
            self.logger.warning(
                f"Could not extract rejection reason from goal handle: {e}"
            )
            return "Goal was rejected by action server (reason unavailable)"

    def get_goal_status(self, goal_id: str) -> Optional[Dict[str, Any]]:
        """Get the current status of a goal (simplified for callback-based approach)"""
        if goal_id not in self._active_goals:
            return None

        goal_info = self._active_goals[goal_id]

        # Build status from stored information
        status = {
            "goal_id": goal_id,
            "action_name": goal_info["action_name"],
            "action_type": goal_info["action_type"],
            "status": goal_info.get("status", "pending"),
        }

        # Add error information if present
        if "error" in goal_info:
            status["error"] = goal_info["error"]

        return status

    def cancel_goal(self, goal_id: str) -> bool:
        """Cancel a running goal"""
        try:
            if goal_id not in self._active_goals:
                self.logger.warning(f"Goal {goal_id} not found in active goals")
                return False

            goal_info = self._active_goals[goal_id]

            # Check if we have a goal handle (goal was accepted)
            if "goal_handle" in goal_info:
                goal_handle = goal_info["goal_handle"]

                # Cancel the goal
                cancel_future = goal_handle.cancel_goal_async()
                self.logger.info(f"Sent cancel request for goal {goal_id}")

                # Emit goal removed event
                asyncio.create_task(self._emit_goal_removed(goal_id))

                # Remove from active goals immediately
                del self._active_goals[goal_id]
                return True
            else:
                # Goal might still be pending, remove it anyway
                asyncio.create_task(self._emit_goal_removed(goal_id))
                del self._active_goals[goal_id]
                self.logger.info(f"Removed pending goal {goal_id}")
                return True

        except Exception as e:
            self.logger.error(f"Error cancelling goal {goal_id}: {e}")
            return False

    def get_active_goals(self) -> List[Dict[str, Any]]:
        """Get list of all active goals"""
        active_goals = []

        for goal_id, goal_info in self._active_goals.items():
            status = self.get_goal_status(goal_id)
            if status:
                active_goals.append(status)

        return active_goals


# Global ROS2 node, executor, and event bus
ros2_node = None
ros2_executor = None
ros2_thread = None
event_bus = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage ROS2 node lifecycle"""
    global ros2_node, ros2_executor, ros2_thread, event_bus

    # Startup
    logger.info("Starting ROS2 node...")
    rclpy.init()

    # Initialize event bus
    event_bus = WebSocketEventBus()

    ros2_node = ROS2ActionManager(event_bus)
    ros2_executor = SingleThreadedExecutor()
    ros2_executor.add_node(ros2_node)

    # Run ROS2 executor in separate thread
    ros2_thread = threading.Thread(target=ros2_executor.spin, daemon=True)
    ros2_thread.start()

    logger.info("ROS2 node started successfully")

    yield

    # Shutdown
    logger.info("Shutting down ROS2 node...")
    if ros2_executor:
        ros2_executor.shutdown()
    if ros2_node:
        ros2_node.destroy_node()
    rclpy.shutdown()
    logger.info("ROS2 node shut down")


# Create FastAPI app
app = FastAPI(
    title="ROS2 Web Bridge API",
    description="REST API and WebSocket for discovering and managing ROS2 actions",
    version="1.0.0",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "http://127.0.0.1:5173",
        "http://0.0.0.0:5173",
        "http://localhost:5174",
        "http://localhost:3000",
        "http://0.0.0.0:5174",
        "http://0.0.0.0:3000",
    ],  # React dev servers
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/", summary="Health check")
async def root():
    """Health check endpoint"""
    return {"message": "ROS2 Web Bridge API is running", "status": "healthy"}


@app.websocket("/ws/events")
async def websocket_endpoint(websocket: WebSocket):
    """Unified WebSocket endpoint for real-time events"""
    if not event_bus:
        await websocket.close(code=1011, reason="Server not ready")
        return

    await event_bus.connect(websocket)

    try:
        while True:
            # Wait for client messages (subscription requests)
            data = await websocket.receive_json()

            if data.get("action") == "subscribe":
                event_types = data.get("event_types", [])
                event_bus.subscribe(websocket, event_types)

                # Send initial snapshot for goals
                if "goal_update" in event_types and ros2_node:
                    active_goals = ros2_node.get_active_goals()
                    await websocket.send_json(
                        {
                            "type": "goals_snapshot",
                            "data": active_goals,
                            "timestamp": asyncio.get_event_loop().time(),
                        }
                    )

            elif data.get("action") == "unsubscribe":
                event_types = data.get("event_types", [])
                event_bus.unsubscribe(websocket, event_types)

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        event_bus.disconnect(websocket)


@app.get(
    "/api/actions",
    response_model=ActionListResponse,
    summary="List all available actions",
)
async def list_actions():
    """Get list of all available ROS2 actions"""
    try:
        if not ros2_node:
            raise HTTPException(status_code=503, detail="ROS2 node not available")

        actions = ros2_node.discover_actions()

        return ActionListResponse(actions=actions, total_count=len(actions))

    except Exception as e:
        logger.error(f"Error listing actions: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to list actions: {str(e)}")


@app.get(
    "/api/actions/{action_name}",
    response_model=ActionInfo,
    summary="Get specific action details",
)
async def get_action(action_name: str):
    """Get details for a specific action"""
    try:
        if not ros2_node:
            raise HTTPException(status_code=503, detail="ROS2 node not available")

        actions = ros2_node.discover_actions()

        for action in actions:
            if action.name == action_name:
                return action

        raise HTTPException(status_code=404, detail=f"Action '{action_name}' not found")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting action {action_name}: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to get action: {str(e)}")


@app.post(
    "/api/actions/{action_path:path}/send_goal",
    response_model=ActionSendGoalResponse,
    summary="Send goal to an action",
)
async def send_goal(
    action_path: str, request: ActionSendGoalRequest, background_tasks: BackgroundTasks
):
    """Send a goal to a ROS2 action with specified parameters"""
    try:
        if not ros2_node:
            raise HTTPException(status_code=503, detail="ROS2 node not available")

        # Get all discovered actions to validate the request
        actions = ros2_node.discover_actions()

        # Parse the action path to extract namespace and action name
        if "/" in action_path:
            # Split namespace and action name (e.g., "turtle1/rotate" -> namespace="turtle1", name="rotate")
            parts = action_path.split("/")
            if len(parts) == 2:
                namespace, action_name = parts
            else:
                raise HTTPException(
                    status_code=400, detail=f"Invalid action path format: {action_path}"
                )
        else:
            # No namespace, just action name
            namespace = ""
            action_name = action_path

        # Find the matching action
        matching_action = None
        for action in actions:
            if action.name == action_name and action.namespace == namespace:
                matching_action = action
                break

        if not matching_action:
            raise HTTPException(
                status_code=404,
                detail=f"Action '{action_name}' not found in namespace '{namespace}'",
            )

        if not matching_action.available:
            raise HTTPException(
                status_code=503,
                detail=f"Action '{action_name}' in namespace '{namespace}' is not available",
            )

        # Reconstruct the full action name with leading slash for ROS2
        full_action_name = (
            f"/{action_path}" if not action_path.startswith("/") else action_path
        )

        # Actually send the goal to the ROS2 action
        try:
            goal_result = await ros2_node.send_action_goal(
                full_action_name, matching_action.type, request.parameters
            )

            logger.info(
                f"Successfully sent goal {goal_result['goal_id']} to action {full_action_name} with parameters: {request.parameters}"
            )

            # Build response based on goal acceptance status
            if goal_result["accepted"]:
                return ActionSendGoalResponse(
                    success=True,
                    message=f"Goal accepted by action '{matching_action.name}' in namespace '{matching_action.namespace}'",
                    goal_id=goal_result["goal_id"],
                    accepted=True,
                )
            else:
                return ActionSendGoalResponse(
                    success=False,
                    message=f"Goal rejected by action '{matching_action.name}' in namespace '{matching_action.namespace}'",
                    goal_id=goal_result["goal_id"],
                    accepted=False,
                    rejection_reason=goal_result.get(
                        "rejection_reason", "Goal was rejected"
                    ),
                )

        except Exception as e:
            logger.error(f"Failed to send goal to action {full_action_name}: {e}")
            raise HTTPException(
                status_code=500, detail=f"Failed to send goal to action: {str(e)}"
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error sending goal to action {action_path}: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to send goal: {str(e)}")


@app.get(
    "/api/goals/{goal_id}/status",
    summary="Get goal status",
)
async def get_goal_status(goal_id: str):
    """Get the status of a specific goal"""
    try:
        if not ros2_node:
            raise HTTPException(status_code=503, detail="ROS2 node not available")

        status = ros2_node.get_goal_status(goal_id)

        if status is None:
            raise HTTPException(status_code=404, detail=f"Goal '{goal_id}' not found")

        return status

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting goal status {goal_id}: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to get goal status: {str(e)}"
        )


@app.post(
    "/api/goals/{goal_id}/cancel",
    response_model=ActionCancelGoalResponse,
    summary="Cancel a running goal",
)
async def cancel_goal(goal_id: str):
    """Cancel a running goal"""
    try:
        if not ros2_node:
            raise HTTPException(status_code=503, detail="ROS2 node not available")

        success = ros2_node.cancel_goal(goal_id)

        if not success:
            raise HTTPException(
                status_code=404,
                detail=f"Goal '{goal_id}' not found or could not be cancelled",
            )

        return ActionCancelGoalResponse(
            success=True,
            message=f"Goal '{goal_id}' has been cancelled",
            goal_id=goal_id,
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error cancelling goal {goal_id}: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to cancel goal: {str(e)}")


@app.get(
    "/api/goals/active",
    summary="Get all active goals",
)
async def get_active_goals():
    """Get list of all active goals"""
    try:
        if not ros2_node:
            raise HTTPException(status_code=503, detail="ROS2 node not available")

        active_goals = ros2_node.get_active_goals()

        return {"active_goals": active_goals, "total_count": len(active_goals)}

    except Exception as e:
        logger.error(f"Error getting active goals: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to get active goals: {str(e)}"
        )


def main():
    """Main entry point for the FastAPI application"""
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
