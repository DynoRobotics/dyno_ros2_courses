"""
FastAPI dependency injection for shared resources
"""

from fastapi import Depends, HTTPException
from typing import Optional

# Global reference to ROS2 node (set by main.py)
_ros2_node = None


def set_ros2_node(node):
    """Set the global ROS2 node reference"""
    global _ros2_node
    _ros2_node = node


def get_ros2_node():
    """Dependency to get the ROS2 node instance"""
    if _ros2_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    return _ros2_node
