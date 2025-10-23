from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING


try:
    from pycdr2 import IdlStruct
    from pycdr2.types import int8, uint8, int16, uint16, int32, uint32, int64, uint64, float32, float64, array
    PYCDR2_AVAILABLE = True
except ImportError:
    PYCDR2_AVAILABLE = False
    IdlStruct = object
    int8 = uint8 = int16 = uint16 = int32 = uint32 = int64 = uint64 = int
    float32 = float64 = float
    array = list  # Fallback



# Import other package modules (not individual classes to avoid circular imports)

from ...builtin_interfaces import msg as builtin_interfaces_msg



if TYPE_CHECKING:
    pass




@dataclass
class Builtins(IdlStruct, typename="test_msgs/Builtins"):

    """test_msgs/Builtins message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_9e61888e7521dda35c21ac6b6cabbcaff8dae88b6d67b25b9078fdb9abf56303


    DDS type name: test_msgs::msg::dds_::Builtins_

    """


    duration_value: builtin_interfaces_msg.Duration = field(default_factory=lambda: builtin_interfaces_msg.Duration())

    time_value: builtin_interfaces_msg.Time = field(default_factory=lambda: builtin_interfaces_msg.Time())

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_9e61888e7521dda35c21ac6b6cabbcaff8dae88b6d67b25b9078fdb9abf56303"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::msg::dds_::Builtins_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['duration_value'] = self.duration_value.to_dict()



        result['time_value'] = self.time_value.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'duration_value' in data:

            
            kwargs['duration_value'] = builtin_interfaces_msg.Duration.from_dict(data['duration_value'])
            


        if 'time_value' in data:

            
            kwargs['time_value'] = builtin_interfaces_msg.Time.from_dict(data['time_value'])
            


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

