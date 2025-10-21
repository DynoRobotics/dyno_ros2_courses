"""
Encoding Utilities for ROS2 Messages

Common serialization/deserialization functions used by all generated messages.
This avoids code duplication across hundreds of message classes.
"""

from typing import Any, Type
import json

# Import encoding libraries at module level (not per-function call!)
try:
    from pycdr2 import IdlStruct
    PYCDR2_AVAILABLE = True
except ImportError:
    PYCDR2_AVAILABLE = False
    IdlStruct = None

try:
    import msgpack
    MSGPACK_AVAILABLE = True
except ImportError:
    MSGPACK_AVAILABLE = False
    msgpack = None


class EncodingUtils:
    """Shared encoding utilities for all message types."""
    
    @staticmethod
    def serialize_cdr(msg: Any, typename: str) -> bytes:
        """
        Serialize message to CDR format using pycdr2.
        
        Args:
            msg: Message instance (must be IdlStruct)
            typename: Type name (unused, kept for API consistency)
            
        Returns:
            CDR-encoded bytes
        """
        if not PYCDR2_AVAILABLE:
            raise RuntimeError(
                "pycdr2 is required for CDR serialization. "
                "Install with: pip install pycdr2"
            )
        return IdlStruct.serialize(msg)
    
    @staticmethod
    def deserialize_cdr(data: bytes, typename: str, cls: Type) -> Any:
        """
        Deserialize CDR bytes to message using pycdr2.
        
        Args:
            data: CDR-encoded bytes
            typename: Type name (unused, kept for API consistency)
            cls: Message class (must be IdlStruct subclass)
            
        Returns:
            Message instance
        """
        if not PYCDR2_AVAILABLE:
            raise RuntimeError(
                "pycdr2 is required for CDR deserialization. "
                "Install with: pip install pycdr2"
            )
        # Call IdlStruct's deserialize directly to avoid infinite recursion
        # with the convenience deserialize method in generated classes
        return IdlStruct.deserialize.__func__(cls, data)
    
    @staticmethod
    def serialize_json(msg: Any) -> bytes:
        """
        Serialize message to JSON format.
        
        Args:
            msg: Message instance (must have to_dict() method)
            
        Returns:
            JSON-encoded bytes (UTF-8)
        """
        return json.dumps(msg.to_dict()).encode('utf-8')
    
    @staticmethod
    def deserialize_json(data: bytes, cls: Type) -> Any:
        """
        Deserialize JSON bytes to message.
        
        Args:
            data: JSON-encoded bytes
            cls: Message class (must have from_dict() classmethod)
            
        Returns:
            Message instance
        """
        return cls.from_dict(json.loads(data.decode('utf-8')))
    
    @staticmethod
    def serialize_msgpack(msg: Any) -> bytes:
        """
        Serialize message to MessagePack format.
        
        Args:
            msg: Message instance (must have to_dict() method)
            
        Returns:
            MessagePack-encoded bytes
        """
        if not MSGPACK_AVAILABLE:
            raise RuntimeError(
                "msgpack is required for MessagePack serialization. "
                "Install with: pip install msgpack"
            )
        return msgpack.packb(msg.to_dict())
    
    @staticmethod
    def deserialize_msgpack(data: bytes, cls: Type) -> Any:
        """
        Deserialize MessagePack bytes to message.
        
        Args:
            data: MessagePack-encoded bytes
            cls: Message class (must have from_dict() classmethod)
            
        Returns:
            Message instance
        """
        if not MSGPACK_AVAILABLE:
            raise RuntimeError(
                "msgpack is required for MessagePack deserialization. "
                "Install with: pip install msgpack"
            )
        return cls.from_dict(msgpack.unpackb(data, raw=False))


# Convenience: expose functions at module level
serialize_cdr = EncodingUtils.serialize_cdr
deserialize_cdr = EncodingUtils.deserialize_cdr
serialize_json = EncodingUtils.serialize_json
deserialize_json = EncodingUtils.deserialize_json
serialize_msgpack = EncodingUtils.serialize_msgpack
deserialize_msgpack = EncodingUtils.deserialize_msgpack
