"""
Subscriber Module

ROS 2-compatible subscriber using Zenoh as the transport layer.
Supports both sync and async callbacks.
"""

import asyncio
import inspect
import logging
import struct
import time
import zenoh
from typing import Any, Callable, Optional, Type

from .liveliness_manager import LivelinessManager

logger = logging.getLogger(__name__)


class Subscriber:
    """ROS 2-compatible subscriber using Zenoh transport."""
    
    def __init__(self, msg_type: Type, topic: str, callback: Callable[[Any], None],
                 node: Optional['Node'] = None, qos_profile: Optional[dict] = None):
        """
        Initialize a ROS 2-compatible subscriber.
        
        Args:
            msg_type: Message type class
            topic: Topic name (e.g., "/turtle1/cmd_vel")
            callback: Callback function to handle received messages
            node: Parent node instance (if None, creates its own session)
            qos_profile: QoS profile settings
        """
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile or {}
        
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
        # Check if the message type has a DDS_TYPE_NAME constant (unified CDR types)
        if hasattr(self.msg_type, 'DDS_TYPE_NAME'):
            return self.msg_type.DDS_TYPE_NAME
        
        # Fallback to common message type mappings
        type_mappings = {
            'geometry_msgs.msg.Twist': 'geometry_msgs::msg::dds_::Twist_',
            'geometry_msgs.msg.Vector3': 'geometry_msgs::msg::dds_::Vector3_',
            'builtin_interfaces.msg.Time': 'builtin_interfaces::msg::dds_::Time_',
            'rcl_interfaces.msg.Log': 'rcl_interfaces::msg::dds_::Log_',
        }
        
        # Try different ways to get the message type name
        message_type_name = f"{self.msg_type.__module__}.{self.msg_type.__name__}"
        
        # Handle cases where the module name might be different
        if 'geometry_msgs.msg._twist' in message_type_name:
            message_type_name = 'geometry_msgs.msg.Twist'
        elif 'geometry_msgs.msg._vector3' in message_type_name:
            message_type_name = 'geometry_msgs.msg.Vector3'
        elif 'builtin_interfaces.msg._time' in message_type_name:
            message_type_name = 'builtin_interfaces.msg.Time'
        elif 'rcl_interfaces.msg._log' in message_type_name:
            message_type_name = 'rcl_interfaces.msg.Log'
        
        return type_mappings.get(message_type_name, f"{message_type_name}::dds_")
    
    def _get_type_hash(self) -> str:
        """Get the type hash for the message type."""
        # Check if the message type has a TYPE_HASH constant (unified CDR types)
        if hasattr(self.msg_type, 'TYPE_HASH'):
            return self.msg_type.TYPE_HASH
        
        # Fallback to common type hashes for standard messages
        type_hashes = {
            'geometry_msgs.msg.Twist': 'RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a',
            'geometry_msgs.msg.Vector3': 'RIHS01_4a7b354a29a8a324c9f9ce904d36969a1eb5b805c515e434cbabac4562cb363d',
            'builtin_interfaces.msg.Time': 'RIHS01_4a7b354a29a8a324c9f9ce904d36969a1eb5b805c515e434cbabac4562cb363d',
            'rcl_interfaces.msg.Log': 'RIHS01_4a7b354a29a8a324c9f9ce904d36969a1eb5b805c515e434cbabac4562cb363d',
        }
        
        # Try different ways to get the message type name
        message_type_name = f"{self.msg_type.__module__}.{self.msg_type.__name__}"
        
        # Handle cases where the module name might be different
        if 'geometry_msgs.msg._twist' in message_type_name:
            message_type_name = 'geometry_msgs.msg.Twist'
        elif 'geometry_msgs.msg._vector3' in message_type_name:
            message_type_name = 'geometry_msgs.msg.Vector3'
        elif 'builtin_interfaces.msg._time' in message_type_name:
            message_type_name = 'builtin_interfaces.msg.Time'
        elif 'rcl_interfaces.msg._log' in message_type_name:
            message_type_name = 'rcl_interfaces.msg.Log'
        
        return type_hashes.get(message_type_name, "RIHS01_" + "0" * 64)
    
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
    
    def _parse_attachment(self, attachment, version: int = 3) -> dict:
        """
        Parse rmw_zenoh_cpp-compatible attachment.
        
        Args:
            attachment: Attachment bytes
            version: Expected attachment version
            
        Returns:
            Parsed attachment data
        """
        try:
            # Convert ZBytes to bytes if needed
            if hasattr(attachment, 'payload'):
                attachment_bytes = bytes(attachment.payload)
            else:
                attachment_bytes = bytes(attachment)
            
            if version == 3:
                # seq + ts + VarInt(16) + gid
                if len(attachment_bytes) >= 25:  # 8 + 8 + 1 + 16
                    seq, ts_ns = struct.unpack("<qq", attachment_bytes[:16])
                    gid = attachment_bytes[17:33]  # Skip VarInt(16) byte
                    return {"sequence": seq, "timestamp_ns": ts_ns, "gid": gid}
            elif version == 2:
                # seq + VarInt(16) + gid
                if len(attachment_bytes) >= 17:  # 8 + 1 + 16
                    seq = struct.unpack("<q", attachment_bytes[:8])[0]
                    gid = attachment_bytes[9:25]  # Skip VarInt(16) byte
                    return {"sequence": seq, "timestamp_ns": None, "gid": gid}
            elif version == 1:
                # ts + VarInt(16) + gid
                if len(attachment_bytes) >= 17:  # 8 + 1 + 16
                    ts_ns = struct.unpack("<q", attachment_bytes[:8])[0]
                    gid = attachment_bytes[9:25]  # Skip VarInt(16) byte
                    return {"sequence": None, "timestamp_ns": ts_ns, "gid": gid}
            
            # Fallback parsing
            if len(attachment_bytes) >= 16:
                gid = attachment_bytes[-16:]  # Last 16 bytes
                return {"sequence": None, "timestamp_ns": None, "gid": gid}
                
        except Exception as e:
            logger.debug(f"Error parsing attachment: {e}")
        
        return {"sequence": None, "timestamp_ns": None, "gid": None}
    
    def _message_handler(self, sample: zenoh.Sample):
        """Handle incoming Zenoh messages."""
        try:
            # Extract payload
            payload = bytes(sample.payload)
            
            # Deserialize using the message type's built-in CDR deserialization
            msg = self.msg_type.deserialize(payload)
            
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
