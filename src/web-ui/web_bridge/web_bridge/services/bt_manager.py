"""
Behavior Tree Manager
Handles py_trees behavior tree snapshot integration and monitoring
"""

import logging
import asyncio
import json
from typing import Dict, List, Any, Optional
from datetime import datetime

import rclpy
from rclpy.node import Node

# py_trees imports for BT integration
try:
    import py_trees
    import py_trees_ros
    from py_trees_ros import trees
    from py_trees.console import green, cyan, yellow, red

    PY_TREES_AVAILABLE = True
except ImportError:
    PY_TREES_AVAILABLE = False
    logging.warning("py_trees not available - BT snapshot features will be disabled")

logger = logging.getLogger(__name__)


class BTSnapshot:
    """Represents a behavior tree snapshot"""

    def __init__(
        self, snapshot_id: str, tree_data: Dict[str, Any], timestamp: datetime = None
    ):
        self.snapshot_id = snapshot_id
        self.tree_data = tree_data
        self.timestamp = timestamp or datetime.now()
        self.metadata = {
            "node_count": self._count_nodes(tree_data),
            "status": tree_data.get("status", "UNKNOWN"),
            "name": tree_data.get("name", "Unknown Tree"),
        }

    def _count_nodes(self, tree_data: Dict[str, Any]) -> int:
        """Count total nodes in the tree"""
        count = 1  # Current node
        children = tree_data.get("children", [])
        for child in children:
            count += self._count_nodes(child)
        return count

    def to_dict(self) -> Dict[str, Any]:
        """Convert snapshot to dictionary for API serialization"""
        return {
            "snapshot_id": self.snapshot_id,
            "timestamp": self.timestamp.isoformat(),
            "metadata": self.metadata,
            "tree_data": self.tree_data,
        }


class BTManager(Node):
    """Manages behavior tree snapshots and monitoring"""

    def __init__(self, event_bus=None):
        super().__init__("bt_manager")
        self.logger = self.get_logger()
        self.event_bus = event_bus

        # Storage for snapshots
        self._snapshots: Dict[str, BTSnapshot] = {}
        self._max_snapshots = 100  # Limit to prevent memory issues

        # BT monitoring state
        self._monitoring_active = False
        self._monitored_trees: Dict[str, Any] = {}

        if not PY_TREES_AVAILABLE:
            self.logger.warning("py_trees not available - BT features disabled")
            return

        self.logger.info("BT Manager initialized successfully")

    def is_available(self) -> bool:
        """Check if BT functionality is available"""
        return PY_TREES_AVAILABLE

    async def create_snapshot(self, tree_name: str = "default") -> Optional[str]:
        """Create a snapshot of the current behavior tree state"""
        if not self.is_available():
            self.logger.warning("Cannot create snapshot - py_trees not available")
            return None

        try:
            # Generate unique snapshot ID
            snapshot_id = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"

            # For now, create a mock tree structure
            # In a real implementation, this would capture actual BT state
            tree_data = self._create_mock_tree_data(tree_name)

            # Create snapshot
            snapshot = BTSnapshot(snapshot_id, tree_data)

            # Store snapshot (with size limit)
            self._snapshots[snapshot_id] = snapshot
            self._enforce_snapshot_limit()

            self.logger.info(f"Created BT snapshot: {snapshot_id}")

            # Emit event if event bus is available
            if self.event_bus:
                await self.event_bus.broadcast(
                    "bt_snapshot_created",
                    {
                        "snapshot_id": snapshot_id,
                        "tree_name": tree_name,
                        "timestamp": snapshot.timestamp.isoformat(),
                        "metadata": snapshot.metadata,
                    },
                )

            return snapshot_id

        except Exception as e:
            self.logger.error(f"Failed to create BT snapshot: {e}")
            return None

    def _create_mock_tree_data(self, tree_name: str) -> Dict[str, Any]:
        """Create mock tree data for demonstration purposes"""
        return {
            "name": tree_name,
            "type": "Sequence",
            "status": "RUNNING",
            "id": "root",
            "children": [
                {
                    "name": "Check Preconditions",
                    "type": "Condition",
                    "status": "SUCCESS",
                    "id": "check_preconditions",
                    "children": [],
                },
                {
                    "name": "Execute Action",
                    "type": "Action",
                    "status": "RUNNING",
                    "id": "execute_action",
                    "children": [],
                },
                {
                    "name": "Cleanup",
                    "type": "Action",
                    "status": "IDLE",
                    "id": "cleanup",
                    "children": [],
                },
            ],
        }

    def _enforce_snapshot_limit(self):
        """Remove oldest snapshots if limit exceeded"""
        if len(self._snapshots) > self._max_snapshots:
            # Sort by timestamp and remove oldest
            sorted_snapshots = sorted(
                self._snapshots.items(), key=lambda x: x[1].timestamp
            )

            # Remove oldest snapshots
            excess_count = len(self._snapshots) - self._max_snapshots
            for i in range(excess_count):
                snapshot_id = sorted_snapshots[i][0]
                del self._snapshots[snapshot_id]
                self.logger.debug(f"Removed old snapshot: {snapshot_id}")

    def get_snapshot(self, snapshot_id: str) -> Optional[BTSnapshot]:
        """Get a specific snapshot by ID"""
        return self._snapshots.get(snapshot_id)

    def list_snapshots(self) -> List[Dict[str, Any]]:
        """List all available snapshots with metadata"""
        snapshots = []
        for snapshot in self._snapshots.values():
            snapshots.append(
                {
                    "snapshot_id": snapshot.snapshot_id,
                    "timestamp": snapshot.timestamp.isoformat(),
                    "metadata": snapshot.metadata,
                }
            )

        # Sort by timestamp (newest first)
        snapshots.sort(key=lambda x: x["timestamp"], reverse=True)
        return snapshots

    def delete_snapshot(self, snapshot_id: str) -> bool:
        """Delete a specific snapshot"""
        if snapshot_id in self._snapshots:
            del self._snapshots[snapshot_id]
            self.logger.info(f"Deleted snapshot: {snapshot_id}")
            return True
        return False

    def clear_all_snapshots(self) -> int:
        """Clear all snapshots and return count of deleted snapshots"""
        count = len(self._snapshots)
        self._snapshots.clear()
        self.logger.info(f"Cleared {count} snapshots")
        return count

    async def start_monitoring(self, tree_names: List[str] = None) -> bool:
        """Start monitoring behavior trees for automatic snapshots"""
        if not self.is_available():
            return False

        try:
            self._monitoring_active = True
            tree_names = tree_names or ["default"]

            self.logger.info(f"Started BT monitoring for trees: {tree_names}")

            # Emit event
            if self.event_bus:
                await self.event_bus.broadcast(
                    "bt_monitoring_started",
                    {"tree_names": tree_names, "timestamp": datetime.now().isoformat()},
                )

            return True

        except Exception as e:
            self.logger.error(f"Failed to start BT monitoring: {e}")
            return False

    async def stop_monitoring(self) -> bool:
        """Stop behavior tree monitoring"""
        try:
            self._monitoring_active = False
            self._monitored_trees.clear()

            self.logger.info("Stopped BT monitoring")

            # Emit event
            if self.event_bus:
                await self.event_bus.broadcast(
                    "bt_monitoring_stopped", {"timestamp": datetime.now().isoformat()}
                )

            return True

        except Exception as e:
            self.logger.error(f"Failed to stop BT monitoring: {e}")
            return False

    def is_monitoring_active(self) -> bool:
        """Check if monitoring is currently active"""
        return self._monitoring_active

    def get_monitoring_status(self) -> Dict[str, Any]:
        """Get current monitoring status"""
        return {
            "active": self._monitoring_active,
            "monitored_trees": list(self._monitored_trees.keys()),
            "snapshot_count": len(self._snapshots),
            "py_trees_available": self.is_available(),
        }
