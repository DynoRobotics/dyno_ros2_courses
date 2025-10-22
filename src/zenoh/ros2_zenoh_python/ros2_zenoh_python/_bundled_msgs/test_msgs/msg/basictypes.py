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



if TYPE_CHECKING:
    pass




@dataclass
class BasicTypes(IdlStruct, typename="test_msgs/BasicTypes"):

    """test_msgs/BasicTypes message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_51eee69530ad562d07a90650bd604269c7378cb3d07865da51a199d9416a3371


    DDS type name: test_msgs::msg::dds_::BasicTypes_

    """


    bool_value: bool = False

    byte_value: uint8 = 0

    char_value: uint8 = 0

    float32_value: float32 = 0.0

    float64_value: float64 = 0.0

    int8_value: int8 = 0

    uint8_value: uint8 = 0

    int16_value: int16 = 0

    uint16_value: uint16 = 0

    int32_value: int32 = 0

    uint32_value: uint32 = 0

    int64_value: int64 = 0

    uint64_value: uint64 = 0

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_51eee69530ad562d07a90650bd604269c7378cb3d07865da51a199d9416a3371"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::msg::dds_::BasicTypes_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['bool_value'] = self.bool_value



        result['byte_value'] = self.byte_value



        result['char_value'] = self.char_value



        result['float32_value'] = self.float32_value



        result['float64_value'] = self.float64_value



        result['int8_value'] = self.int8_value



        result['uint8_value'] = self.uint8_value



        result['int16_value'] = self.int16_value



        result['uint16_value'] = self.uint16_value



        result['int32_value'] = self.int32_value



        result['uint32_value'] = self.uint32_value



        result['int64_value'] = self.int64_value



        result['uint64_value'] = self.uint64_value


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'bool_value' in data:

            kwargs['bool_value'] = data['bool_value']


        if 'byte_value' in data:

            kwargs['byte_value'] = data['byte_value']


        if 'char_value' in data:

            kwargs['char_value'] = data['char_value']


        if 'float32_value' in data:

            kwargs['float32_value'] = data['float32_value']


        if 'float64_value' in data:

            kwargs['float64_value'] = data['float64_value']


        if 'int8_value' in data:

            kwargs['int8_value'] = data['int8_value']


        if 'uint8_value' in data:

            kwargs['uint8_value'] = data['uint8_value']


        if 'int16_value' in data:

            kwargs['int16_value'] = data['int16_value']


        if 'uint16_value' in data:

            kwargs['uint16_value'] = data['uint16_value']


        if 'int32_value' in data:

            kwargs['int32_value'] = data['int32_value']


        if 'uint32_value' in data:

            kwargs['uint32_value'] = data['uint32_value']


        if 'int64_value' in data:

            kwargs['int64_value'] = data['int64_value']


        if 'uint64_value' in data:

            kwargs['uint64_value'] = data['uint64_value']


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

