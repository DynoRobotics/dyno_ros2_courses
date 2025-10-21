"""
Publisher Module

ROS 2-compatible publisher using Zenoh as the transport layer.
Async-first design with full asyncio support.
"""

from __future__ import annotations

import logging
import os
import struct
import time
import zenoh
from typing import Any, Optional, Type

from .liveliness_manager import LivelinessManager
from .name_utils import resolve_topic_name

logger = logging.getLogger(__name__)


class Publisher:
    """ROS 2-compatible publisher using Zenoh transport."""
    
    def __init__(self, msg_type: Type, topic: str, node: Optional[Node] = None, 
                 qos_profile: Optional[dict] = None, encoding: str = 'cdr'):
        """
        Initialize a ROS 2-compatible publisher.
        
        Args:
            msg_type: Message type class
            topic: Topic name (e.g., "/turtle1/cmd_vel")
            node: Parent node instance (if None, creates its own session)
            qos_profile: QoS profile settings
            encoding: Serialization encoding ('cdr', 'json', 'msgpack')
        """
        self.msg_type = msg_type
        self.qos_profile = qos_profile or {}
        self.encoding = encoding
        
        # Get serializer function reference ONCE for zero overhead
        self._serialize = msg_type.get_serializer(encoding)
        
        logger.debug(f"Publisher using {encoding} encoding")
        
        # Convert QoS dict to ROS2 numeric values
        # Reliability: 0=BEST_EFFORT, 1=RELIABLE
        # Durability: 0=SYSTEM_DEFAULT, 1=TRANSIENT_LOCAL, 2=VOLATILE
        # History: 1=KEEP_LAST, 2=KEEP_ALL
        self.qos_reliability = 1 if self.qos_profile.get('reliability', 'reliable') == 'reliable' else 0
        self.qos_durability = {'transient_local': 1, 'volatile': 2}.get(self.qos_profile.get('durability', 'volatile'), 2)
        self.qos_history = 1 if self.qos_profile.get('history', 'keep_last') == 'keep_last' else 2
        self.qos_depth = self.qos_profile.get('depth', 10)
        
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
            self.node_name = "zenoh_publisher"
            self.namespace = ""
            self._own_session = True
            self.node = None
        
        # Resolve topic name with namespace
        self.topic = resolve_topic_name(topic, self.namespace)
        logger.debug(f"Resolved topic: '{topic}' -> '{self.topic}' (namespace: '{self.namespace}')")
        
        # Generate stable publisher GID
        self.publisher_gid = os.urandom(16)
        self.sequence_number = 0
        
        # Create DDS interop key for the topic
        self.dds_key = self._create_dds_interop_key()
        
        # Declare liveliness token for ROS 2 metadata
        self.liveliness_token = self._declare_liveliness_token()
        
        logger.debug(f"Publisher created for topic '{self.topic}'")
    
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
        
        # Build QoS string for liveliness token
        qos_str = self.liveliness_manager.qos_to_keyexpr(
            reliability=self.qos_reliability,
            durability=self.qos_durability,
            history=self.qos_history,
            depth=self.qos_depth
        )
        
        return self.liveliness_manager.declare_publisher_token(
            topic_name=self.topic,
            message_type=message_type_str,
            type_hash=type_hash,
            node_name=self.node_name,
            node_namespace=self.namespace,
            qos=qos_str
        )
    
    def _build_attachment(self, sequence: int) -> bytes:
        """
        Build rmw_zenoh_cpp-compatible attachment (version 3 format).
        
        Format: sequence (8 bytes) + timestamp_ns (8 bytes) + VarInt(16) + gid (16 bytes)
        
        Args:
            sequence: Sequence number
            
        Returns:
            Attachment bytes
        """
        ts_ns = int(time.time_ns())
        leb128_len = b'\x10'  # VarInt(16) = 0x10
        return struct.pack("<qq", sequence, ts_ns) + leb128_len + self.publisher_gid
    
    def publish(self, msg: Any):
        """
        Publish a message.
        
        Args:
            msg: Message instance
        """
        # Serialize using pre-bound function (zero overhead!)
        payload = self._serialize(msg)
        
        # Build attachment
        attachment = self._build_attachment(self.sequence_number)
        self.sequence_number += 1
        
        # Publish via Zenoh
        self.session.put(
            self.dds_key,
            payload,
            encoding=zenoh.Encoding("application/x-cdr"),
            attachment=attachment
        )
    
    def destroy(self):
        """Destroy the publisher and clean up resources."""
        if hasattr(self, 'liveliness_token'):
            self.liveliness_token.undeclare()
        
        # Only close session if we own it
        if self._own_session and hasattr(self, 'session'):
            self.session.close()
        
        logger.debug(f"Publisher for topic '{self.topic}' destroyed")
    
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
