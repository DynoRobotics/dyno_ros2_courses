"""
Publisher Module

ROS 2-compatible publisher using Zenoh as the transport layer.
"""

import os
import struct
import time
import zenoh
from typing import Any, Callable, Optional, Type, Union

from .message_serializer import MessageSerializer
from .liveliness_manager import LivelinessManager


class Publisher:
    """ROS 2-compatible publisher using Zenoh transport."""
    
    def __init__(self, msg_type: Type, topic: str, node: Optional['Node'] = None, 
                 qos_profile: Optional[dict] = None):
        """
        Initialize a ROS 2-compatible publisher.
        
        Args:
            msg_type: Message type class
            topic: Topic name (e.g., "/turtle1/cmd_vel")
            node: Parent node instance (if None, creates its own session)
            qos_profile: QoS profile settings
        """
        self.msg_type = msg_type
        self.topic = topic
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
            self.node_name = "zenoh_publisher"
            self.namespace = ""
            self._own_session = True
            self.node = None
        
        # Initialize components
        self.serializer = MessageSerializer()
        
        # Generate stable publisher GID
        self.publisher_gid = os.urandom(16)
        self.sequence_number = 0
        
        # Create DDS interop key for the topic
        self.dds_key = self._create_dds_interop_key()
        
        # Declare liveliness token for ROS 2 metadata
        self.liveliness_token = self._declare_liveliness_token()
        
        print(f"Publisher created for topic '{self.topic}'")
    
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
        
        return self.liveliness_manager.declare_publisher_token(
            topic_name=self.topic,
            message_type=message_type_str,
            type_hash=type_hash,
            node_name=self.node_name,
            node_namespace=self.namespace
        )
    
    def _build_attachment(self, sequence: int, version: int = 3) -> bytes:
        """
        Build rmw_zenoh_cpp-compatible attachment.
        
        Args:
            sequence: Sequence number
            version: Attachment version (1, 2, or 3)
            
        Returns:
            Attachment bytes
        """
        ts_ns = int(time.time_ns())
        
        if version == 3:
            # Zenoh serialization format: seq + ts + VarInt(16) + gid
            leb128_len = b'\x10'  # VarInt(16) = 0x10
            return struct.pack("<qq", sequence, ts_ns) + leb128_len + self.publisher_gid
        elif version == 2:
            # seq + VarInt(16) + gid
            leb128_len = b'\x10'
            return struct.pack("<q", sequence) + leb128_len + self.publisher_gid
        elif version == 1:
            # ts + VarInt(16) + gid
            leb128_len = b'\x10'
            return struct.pack("<q", ts_ns) + leb128_len + self.publisher_gid
        else:
            # Fallback to v3
            leb128_len = b'\x10'
            return struct.pack("<qq", sequence, ts_ns) + leb128_len + self.publisher_gid
    
    def publish(self, msg: Any):
        """
        Publish a message.
        
        Args:
            msg: Message instance (ROS 2 or simplified)
        """
        # Convert to ROS 2 message if needed
        if hasattr(msg, '__module__') and 'geometry_msgs' in str(msg.__module__):
            # Already a ROS 2 message
            ros2_msg = msg
        else:
            # Convert simplified message to ROS 2
            from .converter import MessageConverter
            ros2_msg = MessageConverter.to_ros2(msg, self.msg_type)
        
        # Serialize the message
        payload = self.serializer.serialize_message(ros2_msg)
        
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
    
    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, linear_z: float = 0.0,
                     angular_x: float = 0.0, angular_y: float = 0.0, angular_z: float = 0.0):
        """
        Convenience method to publish a Twist message.
        
        Args:
            linear_x, linear_y, linear_z: Linear velocity components
            angular_x, angular_y, angular_z: Angular velocity components
        """
        if self.message_type.__name__ != 'Twist':
            raise ValueError(f"Expected Twist message type, got {self.message_type.__name__}")
        
        twist_message = self.serializer.create_twist_message(
            linear_x=linear_x, linear_y=linear_y, linear_z=linear_z,
            angular_x=angular_x, angular_y=angular_y, angular_z=angular_z
        )
        self.publish(twist_message)
    
    def destroy(self):
        """Destroy the publisher and clean up resources."""
        if hasattr(self, 'liveliness_token'):
            self.liveliness_token.undeclare()
        
        # Only close session if we own it
        if self._own_session and hasattr(self, 'session'):
            self.session.close()
        
        print(f"Publisher for topic '{self.topic}' destroyed")
    
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
