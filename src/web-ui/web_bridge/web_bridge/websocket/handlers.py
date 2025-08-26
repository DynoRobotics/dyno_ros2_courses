#!/usr/bin/env python3
"""
WebSocket endpoint handlers
Handles WebSocket connections and message routing
"""

import asyncio
import logging

from fastapi import WebSocket, WebSocketDisconnect

from .event_bus import WebSocketEventBus

logger = logging.getLogger(__name__)


async def websocket_events_handler(
    websocket: WebSocket, event_bus: WebSocketEventBus, ros2_node=None
):
    """Unified WebSocket endpoint handler for real-time events"""
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
