from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING


try:
    from pycdr2 import IdlStruct
    from pycdr2.types import int8, uint8, int16, uint16, int32, uint32, int64, uint64, float32, float64
    PYCDR2_AVAILABLE = True
except ImportError:
    PYCDR2_AVAILABLE = False
    IdlStruct = object
    int8 = uint8 = int16 = uint16 = int32 = uint32 = int64 = uint64 = int
    float32 = float64 = float



if TYPE_CHECKING:
    pass


@dataclass
class ParameterDescriptor(IdlStruct, typename="rcl_interfaces/ParameterDescriptor"):

    """rcl_interfaces/ParameterDescriptor message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_ebdf2718c530f69c384cb5225285d01688b38caf3c17e822fb443212c997fe18


    DDS type name: rcl_interfaces::msg::dds_::ParameterDescriptor_

    """

    name: str = ""

    type: uint8 = 0

    description: str = ""

    additional_constraints: str = ""

    read_only: bool = False

    dynamic_typing: bool = False

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_ebdf2718c530f69c384cb5225285d01688b38caf3c17e822fb443212c997fe18"


    # DDS Type Name
    DDS_TYPE_NAME = "rcl_interfaces::msg::dds_::ParameterDescriptor_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['name'] = self.name



        result['type'] = self.type



        result['description'] = self.description



        result['additional_constraints'] = self.additional_constraints



        result['read_only'] = self.read_only



        result['dynamic_typing'] = self.dynamic_typing


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'name' in data:

            kwargs['name'] = data['name']


        if 'type' in data:

            kwargs['type'] = data['type']


        if 'description' in data:

            kwargs['description'] = data['description']


        if 'additional_constraints' in data:

            kwargs['additional_constraints'] = data['additional_constraints']


        if 'read_only' in data:

            kwargs['read_only'] = data['read_only']


        if 'dynamic_typing' in data:

            kwargs['dynamic_typing'] = data['dynamic_typing']


        return cls(**kwargs)
    
    # Get encoding function references (zero overhead!)
    @classmethod
    def get_serializer(cls, encoding: str = 'cdr'):
        """
        Get serializer function for the specified encoding.
        
        Returns a function that serializes instances of this message type.
        Use this for zero-overhead serialization in hot paths.
        
        Args:
            encoding: One of 'cdr', 'json', 'msgpack'
            
        Returns:
            Function that takes a message instance and returns bytes
            
        Example:
            serialize = Twist.get_serializer('cdr')
            data = serialize(msg)  # Zero overhead!
        """
        from functools import partial
        from ..._encodings import serialize_cdr, serialize_json, serialize_msgpack
        
        serializers = {
            'cdr': partial(serialize_cdr, typename=cls.__name__),
            'json': serialize_json,
            'msgpack': serialize_msgpack,
        }
        if encoding not in serializers:
            raise ValueError(f"Unknown encoding '{encoding}'. Available: {list(serializers.keys())}")
        return serializers[encoding]
    
    @classmethod
    def get_deserializer(cls, encoding: str = 'cdr'):
        """
        Get deserializer function for the specified encoding.
        
        Returns a function that deserializes bytes to instances of this message type.
        Use this for zero-overhead deserialization in hot paths.
        
        Args:
            encoding: One of 'cdr', 'json', 'msgpack'
            
        Returns:
            Function that takes bytes and returns a message instance
            
        Example:
            deserialize = Twist.get_deserializer('cdr')
            msg = deserialize(data)  # Zero overhead!
        """
        from functools import partial
        from ..._encodings import deserialize_cdr, deserialize_json, deserialize_msgpack
        
        deserializers = {
            'cdr': partial(deserialize_cdr, typename=cls.__name__, cls=cls),
            'json': partial(deserialize_json, cls=cls),
            'msgpack': partial(deserialize_msgpack, cls=cls),
        }
        if encoding not in deserializers:
            raise ValueError(f"Unknown encoding '{encoding}'. Available: {list(deserializers.keys())}")
        return deserializers[encoding]
    
    # Convenience methods (optional - adds ~10ns overhead)
    def serialize(self, encoding: str = 'cdr') -> bytes:
        """Serialize message with specified encoding."""
        serializer = self.get_serializer(encoding)
        return serializer(self)
    
    @classmethod
    def deserialize(cls, data: bytes, encoding: str = 'cdr'):
        """Deserialize message with specified encoding."""
        deserializer = cls.get_deserializer(encoding)
        return deserializer(data)

