"""
BT Snapshots API router
Handles behavior tree snapshot management endpoints
"""

import logging
from typing import List, Optional
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel

from .dependencies import get_ros2_node

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/bt", tags=["behavior_trees"])


class BTSnapshotCreateRequest(BaseModel):
    """Request model for creating BT snapshots"""

    tree_name: str = "default"


class BTSnapshotResponse(BaseModel):
    """Response model for BT snapshot operations"""

    success: bool
    message: str
    snapshot_id: Optional[str] = None


class BTMonitoringRequest(BaseModel):
    """Request model for BT monitoring operations"""

    tree_names: List[str] = ["default"]


@router.get("/status", summary="Get BT system status")
async def get_bt_status(ros2_node=Depends(get_ros2_node)):
    """Get behavior tree system status and availability"""
    try:
        # Check if BT manager is available
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            return {
                "available": False,
                "message": "BT Manager not initialized",
                "py_trees_available": False,
            }

        status = bt_manager.get_monitoring_status()
        return {
            "available": bt_manager.is_available(),
            "message": (
                "BT system operational"
                if bt_manager.is_available()
                else "py_trees not available"
            ),
            **status,
        }

    except Exception as e:
        logger.error(f"Error getting BT status: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to get BT status: {str(e)}"
        )


@router.post(
    "/snapshots", response_model=BTSnapshotResponse, summary="Create BT snapshot"
)
async def create_snapshot(
    request: BTSnapshotCreateRequest, ros2_node=Depends(get_ros2_node)
):
    """Create a new behavior tree snapshot"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        if not bt_manager.is_available():
            raise HTTPException(status_code=503, detail="py_trees not available")

        snapshot_id = await bt_manager.create_snapshot(request.tree_name)

        if snapshot_id:
            return BTSnapshotResponse(
                success=True,
                message=f"Snapshot created successfully for tree '{request.tree_name}'",
                snapshot_id=snapshot_id,
            )
        else:
            raise HTTPException(status_code=500, detail="Failed to create snapshot")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error creating BT snapshot: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to create snapshot: {str(e)}"
        )


@router.get("/snapshots", summary="List all BT snapshots")
async def list_snapshots(ros2_node=Depends(get_ros2_node)):
    """Get list of all available behavior tree snapshots"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        snapshots = bt_manager.list_snapshots()
        return {"snapshots": snapshots, "total_count": len(snapshots)}

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error listing BT snapshots: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to list snapshots: {str(e)}"
        )


@router.get("/snapshots/{snapshot_id}", summary="Get specific BT snapshot")
async def get_snapshot(snapshot_id: str, ros2_node=Depends(get_ros2_node)):
    """Get details of a specific behavior tree snapshot"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        snapshot = bt_manager.get_snapshot(snapshot_id)
        if not snapshot:
            raise HTTPException(
                status_code=404, detail=f"Snapshot '{snapshot_id}' not found"
            )

        return snapshot.to_dict()

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting BT snapshot {snapshot_id}: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to get snapshot: {str(e)}")


@router.delete(
    "/snapshots/{snapshot_id}",
    response_model=BTSnapshotResponse,
    summary="Delete BT snapshot",
)
async def delete_snapshot(snapshot_id: str, ros2_node=Depends(get_ros2_node)):
    """Delete a specific behavior tree snapshot"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        success = bt_manager.delete_snapshot(snapshot_id)
        if success:
            return BTSnapshotResponse(
                success=True, message=f"Snapshot '{snapshot_id}' deleted successfully"
            )
        else:
            raise HTTPException(
                status_code=404, detail=f"Snapshot '{snapshot_id}' not found"
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting BT snapshot {snapshot_id}: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to delete snapshot: {str(e)}"
        )


@router.delete(
    "/snapshots", response_model=BTSnapshotResponse, summary="Clear all BT snapshots"
)
async def clear_all_snapshots(ros2_node=Depends(get_ros2_node)):
    """Delete all behavior tree snapshots"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        count = bt_manager.clear_all_snapshots()
        return BTSnapshotResponse(
            success=True, message=f"Cleared {count} snapshots successfully"
        )

    except Exception as e:
        logger.error(f"Error clearing BT snapshots: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to clear snapshots: {str(e)}"
        )


@router.post(
    "/monitoring/start",
    response_model=BTSnapshotResponse,
    summary="Start BT monitoring",
)
async def start_monitoring(
    request: BTMonitoringRequest, ros2_node=Depends(get_ros2_node)
):
    """Start monitoring behavior trees for automatic snapshots"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        if not bt_manager.is_available():
            raise HTTPException(status_code=503, detail="py_trees not available")

        success = await bt_manager.start_monitoring(request.tree_names)
        if success:
            return BTSnapshotResponse(
                success=True, message=f"Started monitoring trees: {request.tree_names}"
            )
        else:
            raise HTTPException(status_code=500, detail="Failed to start monitoring")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error starting BT monitoring: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to start monitoring: {str(e)}"
        )


@router.post(
    "/monitoring/stop", response_model=BTSnapshotResponse, summary="Stop BT monitoring"
)
async def stop_monitoring(ros2_node=Depends(get_ros2_node)):
    """Stop behavior tree monitoring"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        success = await bt_manager.stop_monitoring()
        if success:
            return BTSnapshotResponse(
                success=True, message="Stopped BT monitoring successfully"
            )
        else:
            raise HTTPException(status_code=500, detail="Failed to stop monitoring")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error stopping BT monitoring: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to stop monitoring: {str(e)}"
        )


@router.get("/monitoring/status", summary="Get BT monitoring status")
async def get_monitoring_status(ros2_node=Depends(get_ros2_node)):
    """Get current behavior tree monitoring status"""
    try:
        bt_manager = getattr(ros2_node, "bt_manager", None)
        if not bt_manager:
            raise HTTPException(status_code=503, detail="BT Manager not available")

        return bt_manager.get_monitoring_status()

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting BT monitoring status: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to get monitoring status: {str(e)}"
        )
