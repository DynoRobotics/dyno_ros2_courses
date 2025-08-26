#!/usr/bin/env python3
"""
WebSocket Event Bus for real-time communication
Manages WebSocket connections and event broadcasting
"""

import asyncio
import logging
from typing import Set, Dict, List, Any
from collections import defaultdict

from fastapi import WebSocket

logger = logging.getLogger(__name__)


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
