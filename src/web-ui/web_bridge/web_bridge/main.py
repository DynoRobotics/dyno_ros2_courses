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

# Local imports
from web_bridge.models import (
    ActionGoalField,
    ActionInfo,
    ActionListResponse,
    ActionSendGoalRequest,
    ActionSendGoalResponse,
    ActionCancelGoalResponse,
)
from web_bridge.services.interface_parser import InterfaceTextLine
from web_bridge.services.action_manager import ROS2ActionManager
from web_bridge.services.bt_manager import BTManager
from web_bridge.websocket.event_bus import WebSocketEventBus
from web_bridge.websocket.handlers import websocket_events_handler
from web_bridge.api.dependencies import set_ros2_node
from web_bridge.api import actions, goals, bt_snapshots

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


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

    # Initialize BT manager and attach to node
    bt_manager = BTManager()
    ros2_node.bt_manager = bt_manager

    # Set up dependency injection
    set_ros2_node(ros2_node)

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

# Include API routers
app.include_router(actions.router)
app.include_router(goals.router)
app.include_router(bt_snapshots.router)


@app.get("/", summary="Health check")
async def root():
    """Health check endpoint"""
    return {"message": "ROS2 Web Bridge API is running", "status": "healthy"}


@app.websocket("/ws/events")
async def websocket_endpoint(websocket: WebSocket):
    """Unified WebSocket endpoint for real-time events"""
    await websocket_events_handler(websocket, event_bus, ros2_node)


def main():
    """Main entry point for the FastAPI application"""
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
