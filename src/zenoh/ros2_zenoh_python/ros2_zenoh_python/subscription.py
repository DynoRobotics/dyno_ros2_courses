"""
Subscription Module

ROS 2-compatible subscription using Zenoh as the transport layer.
Supports both sync and async callbacks.
"""

from __future__ import annotations

import asyncio
import inspect
import logging
import struct
import time
import zenoh
from typing import Any, Callable, Optional, Type

from .liveliness_manager import LivelinessManager
from .name_utils import resolve_topic_name

logger = logging.getLogger(__name__)


class Subscription:
    """ROS 2-compatible subscription using Zenoh transport."""
    
    def __init__(self, msg_type: Type, topic: str, callback: Callable[[Any], None],
                 node: Optional[Node] = None, qos_profile: Optional[dict] = None, encoding: str = 'cdr'):
        """
        Initialize a ROS 2-compatible subscription.
        
        Args:
            msg_type: Message type class
            topic: Topic name (e.g., "/turtle1/cmd_vel")
            callback: Callback function to handle received messages
            node: Parent node instance (if None, creates its own session)
            qos_profile: QoS profile settings
            encoding: Deserialization encoding ('cdr', 'json', 'msgpack')
        """
        self.msg_type = msg_type
        self.callback = callback
        self.qos_profile = qos_profile or {}
        self.encoding = encoding
        
        # Get deserializer function reference ONCE for zero overhead
        self._deserialize = msg_type.get_deserializer(encoding)
        
        logger.debug(f"Subscription using {encoding} encoding")
        
        # Detect if callback is async
        self.is_async_callback = inspect.iscoroutinefunction(callback)
        
        # Store event loop reference for async callbacks
        if self.is_async_callback:
            try:
                self.loop = asyncio.get_running_loop()
            except RuntimeError:
                # No running loop yet, will be set when we enter async context
                self.loop = None
        
        # Use provided node or create our own session
        if node is not None:
            self.node = node
            self.session = node.session
            self.liveliness_manager = node.liveliness_manager
            self.node_name = node.node_name
            self.namespace = node.namespace
            self._own_session = False
        else:
            # Create our own session (legacy mode)
            import zenoh
            self.config = zenoh.Config()
            self.session = zenoh.open(self.config)
            self.liveliness_manager = LivelinessManager(self.session)
            self.node_name = "zenoh_subscriber"
            self.namespace = ""
            self._own_session = True
            self.node = None
        
        # Resolve topic name with namespace
        self.topic = resolve_topic_name(topic, self.namespace)
        logger.debug(f"Resolved topic: '{topic}' -> '{self.topic}' (namespace: '{self.namespace}')")
        
        # Create DDS interop key for the topic
        self.dds_key = self._create_dds_interop_key()
        
        # Declare liveliness token for ROS 2 metadata
        self.liveliness_token = self._declare_liveliness_token()
        
        # Create subscription
        self.subscription = self.session.declare_subscriber(
            self.dds_key,
            self._message_handler
        )
        
        logger.debug(f"Subscriber created for topic '{self.topic}'")
    
    def _create_dds_interop_key(self) -> str:
        """Create DDS interop key for the topic."""
        # Extract topic name without leading slash
        topic_part = self.topic.lstrip("/")
        
        # Create DDS interop key format
        # Format: "0/<topic>/<message_type>/<type_hash>"
        message_type_str = self._get_message_type_string()
        type_hash = self._get_type_hash()
        
        return f"0/{topic_part}/{message_type_str}/{type_hash}"
    
    def _get_message_type_string(self) -> str:
        """Get the message type string for DDS interop."""
        # All bundled and generated messages must have a DDS_TYPE_NAME constant
        if hasattr(self.msg_type, 'DDS_TYPE_NAME'):
            return self.msg_type.DDS_TYPE_NAME
        
        # If DDS_TYPE_NAME is not defined, the message type is invalid
        raise ValueError(
            f"Message type {self.msg_type} must have a DDS_TYPE_NAME attribute. "
            f"Use bundled messages from _bundled_msgs or generated messages from ros2_interfaces_py. "
            f"For custom messages, use the generator in tools/ to create proper CDR types."
        )
    
    def _get_type_hash(self) -> str:
        """Get the type hash for the message type."""
        # All bundled and generated messages must have a TYPE_HASH constant
        if hasattr(self.msg_type, 'TYPE_HASH'):
            return self.msg_type.TYPE_HASH
        
        # If TYPE_HASH is not defined, the message type is invalid
        raise ValueError(
            f"Message type {self.msg_type} must have a TYPE_HASH attribute. "
            f"Use bundled messages from _bundled_msgs or generated messages from ros2_interfaces_py. "
            f"For custom messages, use the generator in tools/ to create proper CDR types."
        )
    
    def _declare_liveliness_token(self) -> zenoh.LivelinessToken:
        """Declare liveliness token for ROS 2 metadata."""
        message_type_str = self._get_message_type_string()
        type_hash = self._get_type_hash()
        
        return self.liveliness_manager.declare_subscriber_token(
            topic_name=self.topic,
            message_type=message_type_str,
            type_hash=type_hash,
            node_name=self.node_name,
            node_namespace=self.namespace
        )
    
    def _parse_attachment(self, attachment) -> dict:
        """
        Parse rmw_zenoh_cpp-compatible attachment (version 3 format).
        
        Format: sequence (8 bytes) + timestamp_ns (8 bytes) + VarInt(16) + gid (16 bytes)
        
        Args:
            attachment: Attachment bytes
            
        Returns:
            Dictionary with sequence, timestamp_ns, and gid
        """
        try:
            # Convert ZBytes to bytes if needed
            if hasattr(attachment, 'payload'):
                attachment_bytes = bytes(attachment.payload)
            else:
                attachment_bytes = bytes(attachment)
            
            # Version 3 format: seq + ts + VarInt(16) + gid
            if len(attachment_bytes) >= 33:  # 8 + 8 + 1 + 16
                seq, ts_ns = struct.unpack("<qq", attachment_bytes[:16])
                gid = attachment_bytes[17:33]  # Skip VarInt(16) byte
                return {"sequence": seq, "timestamp_ns": ts_ns, "gid": gid}
                
        except Exception as e:
            logger.debug(f"Error parsing attachment: {e}")
        
        return {"sequence": None, "timestamp_ns": None, "gid": None}
    
    def _message_handler(self, sample: zenoh.Sample):
        """Handle incoming Zenoh messages."""
        try:
            # Extract payload
            payload = bytes(sample.payload)
            
            # Deserialize using pre-bound function (zero overhead!)
            msg = self._deserialize(payload)
            
            # Call user callback (async or sync)
            if self.is_async_callback:
                # Zenoh callbacks come from a different thread
                # We need to schedule the async callback in the event loop thread
                if self.loop is None:
                    # Try to get the loop again
                    try:
                        self.loop = asyncio.get_running_loop()
                    except RuntimeError:
                        logger.warning("No event loop running for async callback")
                        return
                
                # Schedule the coroutine in the event loop from this thread
                asyncio.run_coroutine_threadsafe(self.callback(msg), self.loop)
            else:
                # Call sync callback directly
                self.callback(msg)
            
        except Exception as e:
            logger.error(f"Error handling message: {e}", exc_info=True)
    
    def spin(self):
        """Keep the subscriber alive and processing messages."""
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    
    def destroy(self):
        """Destroy the subscriber and clean up resources."""
        if hasattr(self, 'subscription'):
            self.subscription.undeclare()
        if hasattr(self, 'liveliness_token'):
            self.liveliness_token.undeclare()
        
        # Only close session if we own it
        if self._own_session and hasattr(self, 'session'):
            self.session.close()
        
        logger.debug(f"Subscriber for topic '{self.topic}' destroyed")
    
    async def adestroy(self):
        """Async version of destroy for use in async context managers."""
        self.destroy()  # Cleanup is already non-blocking
    
    def close(self):
        """Legacy method - use destroy() instead."""
        self.destroy()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
    
    def __del__(self):
        """Destructor: ensure cleanup."""
        try:
            self.close()
        except:
            pass  # Ignore errors during cleanup
