"""
Subscriber Module

ROS 2-compatible subscriber using Zenoh as the transport layer.
"""

import struct
import time
import zenoh
from typing import Any, Callable, Optional, Type

from .message_serializer import MessageSerializer
from .liveliness_manager import LivelinessManager


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
        
        # Initialize components
        self.serializer = MessageSerializer()
        
        # Create DDS interop key for the topic
        self.dds_key = self._create_dds_interop_key()
        
        # Declare liveliness token for ROS 2 metadata
        self.liveliness_token = self._declare_liveliness_token()
        
        # Create subscription
        self.subscription = self.session.declare_subscriber(
            self.dds_key,
            self._message_handler
        )
        
        print(f"Subscriber created for topic '{self.topic}'")
    
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
        # Common message type mappings
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
        # Common type hashes for standard messages
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
            print(f"Error parsing attachment: {e}")
        
        return {"sequence": None, "timestamp_ns": None, "gid": None}
    
    def _message_handler(self, sample: zenoh.Sample):
        """Handle incoming Zenoh messages."""
        try:
            # Parse attachment
            attachment_data = {}
            if sample.attachment:
                attachment_data = self._parse_attachment(sample.attachment)
            
            # Extract payload
            payload = bytes(sample.payload)
            
            # For now, we'll just print the message info
            # In a full implementation, we'd deserialize the payload
            print(f"Received message on topic '{self.topic}':")
            print(f"  Sequence: {attachment_data.get('sequence', 'N/A')}")
            print(f"  Timestamp: {attachment_data.get('timestamp_ns', 'N/A')}")
            print(f"  GID: {attachment_data.get('gid', b'N/A').hex() if attachment_data.get('gid') else 'N/A'}")
            print(f"  Payload length: {len(payload)} bytes")
            
            # Call user callback with raw data for now
            # In a full implementation, we'd deserialize and pass the message object
            self.callback({
                'topic': self.topic,
                'attachment': attachment_data,
                'payload': payload,
                'timestamp': time.time()
            })
            
        except Exception as e:
            print(f"Error handling message: {e}")
    
    def destroy(self):
        """Destroy the subscriber and clean up resources."""
        if hasattr(self, 'subscription'):
            self.subscription.undeclare()
        if hasattr(self, 'liveliness_token'):
            self.liveliness_token.undeclare()
        
        # Only close session if we own it
        if self._own_session and hasattr(self, 'session'):
            self.session.close()
        
        print(f"Subscriber for topic '{self.topic}' destroyed")
    
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
