#!/usr/bin/env python3
"""
ROS2 Action Manager
Handles ROS2 action discovery, goal management, and real-time feedback
"""

import asyncio
import logging
import uuid
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, get_action_names_and_types

# ROS2 interface parsing imports
from rosidl_runtime_py import get_interface_path

# Local imports
from ..models import ActionGoalField, ActionInfo
from .interface_parser import InterfaceTextLine

from action_msgs.msg import GoalStatus
import action_msgs.msg

# Action definition separator constant
ACTION_REQUEST_RESPONSE_SEPARATOR = "---"

logger = logging.getLogger(__name__)


class ROS2ActionManager(Node):
    """ROS2 node for discovering and calling actions"""

    def __init__(self, event_bus):
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

        # Status mapping for ROS2 action statuses
        self.status_strings = {
            action_msgs.msg.GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",
            action_msgs.msg.GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",
            action_msgs.msg.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",
            action_msgs.msg.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",
            action_msgs.msg.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
            action_msgs.msg.GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",
            action_msgs.msg.GoalStatus.STATUS_ABORTED: "STATUS_ABORTED",
        }

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

        # Debug logging for discovered actions
        self.logger.info(f"Raw discovered actions:")
        for action in discovered_actions:
            self.logger.info(
                f"  - {action.name} (namespace: '{action.namespace}', type: {action.type})"
            )

        # Update cache using unique key (namespace + name)
        self._action_cache = {}
        for action in discovered_actions:
            # Create unique key that includes namespace
            cache_key = (
                f"{action.namespace}/{action.name}" if action.namespace else action.name
            )
            self._action_cache[cache_key] = action
        self._last_discovery_time = current_time

        # Debug logging for unique namespaces
        unique_namespaces = set()
        for action in discovered_actions:
            if action.namespace:
                unique_namespaces.add(action.namespace)
            else:
                unique_namespaces.add("")
        self.logger.info(f"Unique namespaces found: {sorted(unique_namespaces)}")

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

            # Debug logging for namespace parsing
            self.logger.info(
                f"Parsing action: {action_name} -> namespace: '{namespace}', clean_name: '{clean_name}'"
            )

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

            # Create feedback callback function for this specific goal
            def feedback_callback(feedback_msg):
                try:
                    self.logger.debug(
                        f"Received feedback for goal {goal_id}: {feedback_msg.feedback}"
                    )

                    # Extract feedback data from the message
                    feedback_data = self._extract_feedback_data(
                        feedback_msg.feedback, action_type
                    )

                    # Store latest feedback in goal info if goal still exists
                    if goal_id in self._active_goals:
                        self._active_goals[goal_id]["latest_feedback"] = feedback_data

                        # Emit feedback event via WebSocket
                        self._schedule_async_task(
                            self._emit_goal_feedback(goal_id, feedback_data)
                        )

                except Exception as e:
                    self.logger.warning(
                        f"Error in feedback callback for goal {goal_id}: {e}"
                    )

            # Send the goal asynchronously WITH feedback callback
            future = client.send_goal_async(
                goal_msg, feedback_callback=feedback_callback
            )

            # Store the goal for tracking
            self._active_goals[goal_id] = {
                "client": client,
                "future": future,
                "action_name": action_name,
                "action_type": action_type,
                "parameters": parameters,
                "status": "pending",
            }

            self.logger.info(
                f"Sent goal {goal_id} to action {action_name} with feedback callback"
            )

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

                    # Set up result callback for completion handling
                    self._setup_result_callback(goal_id, goal_handle)

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

    async def _emit_goal_feedback(self, goal_id: str, feedback_data: Dict[str, Any]):
        """Emit WebSocket event when goal feedback is received"""
        try:
            feedback_event = {"goal_id": goal_id, "feedback": feedback_data}
            await self.event_bus.broadcast("goal_feedback", feedback_event)
        except Exception as e:
            self.logger.warning(f"Failed to emit goal_feedback event: {e}")

    def _setup_result_callback(self, goal_id: str, goal_handle):
        """Set up result callback for goal completion handling"""
        try:
            if goal_id not in self._active_goals:
                self.logger.warning(
                    f"Goal {goal_id} not found when setting up result callback"
                )
                return

            # Set up result future and callbacks
            if hasattr(goal_handle, "get_result_async"):
                result_future = goal_handle.get_result_async()
                self._active_goals[goal_id]["result_future"] = result_future

                # Set up result completion callback
                result_future.add_done_callback(
                    lambda f: self._handle_goal_completion(goal_id, f)
                )

                self.logger.info(f"Set up result callback for goal {goal_id}")

            else:
                self.logger.warning(
                    f"Goal handle for {goal_id} does not support get_result_async"
                )

        except Exception as e:
            self.logger.error(
                f"Error setting up result callback for goal {goal_id}: {e}"
            )

    def _extract_feedback_data(self, feedback_msg, action_type: str) -> Dict[str, Any]:
        """Extract feedback data from ROS2 feedback message"""
        try:
            feedback_data = {}

            # Common feedback fields that most actions provide
            if hasattr(feedback_msg, "progress"):
                # Convert progress to percentage for display
                progress_value = float(feedback_msg.progress)
                feedback_data["progress"] = f"{progress_value * 100:.1f}%"

            # Action-specific feedback extraction
            if "Move" in action_type:
                if hasattr(feedback_msg, "current_distance"):
                    feedback_data["current_distance"] = (
                        f"{float(feedback_msg.current_distance):.2f}m"
                    )

            elif "Rotate" in action_type:
                # Rotate action typically only has progress
                pass

            # Add any other feedback fields dynamically
            for attr_name in dir(feedback_msg):
                if not attr_name.startswith("_") and attr_name not in [
                    "progress",
                    "current_distance",
                ]:
                    try:
                        attr_value = getattr(feedback_msg, attr_name)
                        # Only include simple types that can be serialized
                        if isinstance(attr_value, (int, float, str, bool)):
                            feedback_data[attr_name] = str(attr_value)
                    except Exception:
                        # Skip attributes that can't be accessed
                        pass

            return feedback_data

        except Exception as e:
            self.logger.warning(f"Error extracting feedback data: {e}")
            return {}

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

                # Extract the actual ROS2 action status from the result
                ros2_status = result.status
                status_string = self.status_strings.get(
                    ros2_status, f"UNKNOWN_STATUS_{ros2_status}"
                )

                self.logger.info(
                    f"Goal {goal_id} completed with status: {status_string}"
                )

                # Map ROS2 status to our internal status
                if ros2_status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
                    internal_status = "succeeded"
                elif ros2_status == action_msgs.msg.GoalStatus.STATUS_ABORTED:
                    internal_status = "aborted"
                elif ros2_status == action_msgs.msg.GoalStatus.STATUS_CANCELED:
                    internal_status = "canceled"
                else:
                    internal_status = "completed"  # fallback

                status = {
                    "goal_id": goal_id,
                    "action_name": goal_info["action_name"],
                    "action_type": goal_info["action_type"],
                    "status": internal_status,
                    "ros2_status": status_string,
                }

                # Add result data if available
                if hasattr(result, "result") and result.result:
                    status["result"] = self._extract_result_data(
                        result.result, goal_info["action_type"]
                    )

                # Emit events and remove goal
                self._schedule_async_task(self._emit_goal_update(status))
                self._schedule_async_task(self._emit_goal_removed(goal_id))

            except Exception as e:
                # Goal failed due to exception
                self.logger.error(f"Goal {goal_id} failed with exception: {e}")

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

    def _extract_result_data(self, result_msg, action_type: str) -> Dict[str, Any]:
        """Extract result data from ROS2 result message"""
        try:
            result_data = {}

            # Action-specific result extraction
            if "Move" in action_type:
                if hasattr(result_msg, "total_time_sec"):
                    result_data["total_time"] = (
                        f"{float(result_msg.total_time_sec):.2f}s"
                    )
                if hasattr(result_msg, "distance_traveled"):
                    result_data["distance_traveled"] = (
                        f"{float(result_msg.distance_traveled):.2f}m"
                    )

            elif "Rotate" in action_type:
                if hasattr(result_msg, "total_time_sec"):
                    result_data["total_time"] = (
                        f"{float(result_msg.total_time_sec):.2f}s"
                    )
                if hasattr(result_msg, "angle_rotated"):
                    result_data["angle_rotated"] = (
                        f"{float(result_msg.angle_rotated):.2f}rad"
                    )

            # Add any other result fields dynamically
            for attr_name in dir(result_msg):
                if not attr_name.startswith("_") and attr_name not in [
                    "total_time_sec",
                    "distance_traveled",
                    "angle_rotated",
                ]:
                    try:
                        attr_value = getattr(result_msg, attr_name)
                        # Only include simple types that can be serialized
                        if isinstance(attr_value, (int, float, str, bool)):
                            result_data[attr_name] = str(attr_value)
                    except Exception:
                        # Skip attributes that can't be accessed
                        pass

            return result_data

        except Exception as e:
            self.logger.warning(f"Error extracting result data: {e}")
            return {}

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
            "action_name": goal_info[
                "action_name"
            ],  # This is the full ROS2 action name like "/turtle1/move"
            "action_type": goal_info["action_type"],
            "status": goal_info.get("status", "pending"),
        }

        # Add error information if present
        if "error" in goal_info:
            status["error"] = goal_info["error"]

        # Add feedback if present
        if "latest_feedback" in goal_info:
            status["feedback"] = goal_info["latest_feedback"]

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
