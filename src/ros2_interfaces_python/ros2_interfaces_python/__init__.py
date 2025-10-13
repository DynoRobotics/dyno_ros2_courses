"""
ROS 2 Interfaces Python Package

Simplified Python dataclasses for ROS 2 interface types (messages, services, actions).
This package provides lightweight alternatives to ROS 2 message definitions that don't require ROS 2 installation.
"""

from .msg import *
# Future: from .srv import *
# Future: from .action import *

__version__ = "0.1.0"
__all__ = [
    # Message types are exported from msg/__init__.py
]