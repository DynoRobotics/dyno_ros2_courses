"""
Actions API router
Handles ROS2 action discovery and management endpoints
"""

import logging
from typing import List
from fastapi import APIRouter, HTTPException, BackgroundTasks, Depends

from ..models import (
    ActionInfo,
    ActionListResponse,
    ActionSendGoalRequest,
    ActionSendGoalResponse,
)
from .dependencies import get_ros2_node

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/actions", tags=["actions"])


@router.get(
    "",
    response_model=ActionListResponse,
    summary="List all available actions",
)
async def list_actions(ros2_node=Depends(get_ros2_node)):
    """Get list of all available ROS2 actions"""
    try:
        if not ros2_node:
            raise HTTPException(status_code=503, detail="ROS2 node not available")

        actions = ros2_node.discover_actions()

        return ActionListResponse(actions=actions, total_count=len(actions))

    except Exception as e:
        logger.error(f"Error listing actions: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to list actions: {str(e)}")


@router.get(
    "/{action_name}",
    response_model=ActionInfo,
    summary="Get specific action details",
)
async def get_action(action_name: str, ros2_node=Depends(get_ros2_node)):
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


@router.post(
    "/{action_path:path}/send_goal",
    response_model=ActionSendGoalResponse,
    summary="Send goal to an action",
)
async def send_goal(
    action_path: str,
    request: ActionSendGoalRequest,
    background_tasks: BackgroundTasks,
    ros2_node=Depends(get_ros2_node),
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
