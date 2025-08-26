"""
Goals API router
Handles ROS2 action goal management endpoints
"""

import logging
from fastapi import APIRouter, HTTPException, Depends

from ..models import ActionCancelGoalResponse
from .dependencies import get_ros2_node

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/goals", tags=["goals"])


@router.get(
    "/{goal_id}/status",
    summary="Get goal status",
)
async def get_goal_status(goal_id: str, ros2_node=Depends(get_ros2_node)):
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


@router.post(
    "/{goal_id}/cancel",
    response_model=ActionCancelGoalResponse,
    summary="Cancel a running goal",
)
async def cancel_goal(goal_id: str, ros2_node=Depends(get_ros2_node)):
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


@router.get(
    "/active",
    summary="Get all active goals",
)
async def get_active_goals(ros2_node=Depends(get_ros2_node)):
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
