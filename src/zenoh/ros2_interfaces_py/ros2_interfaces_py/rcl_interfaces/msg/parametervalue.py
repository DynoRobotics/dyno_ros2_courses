from dataclasses import dataclass
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
class ParameterValue(IdlStruct, typename="rcl_interfaces/ParameterValue"):

    """rcl_interfaces/ParameterValue message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_862534def30ec11ac5860cce7f69c93db3d4fed5ff3c77fb22d6fd729420d234


    DDS type name: rcl_interfaces::msg::dds_::ParameterValue_

    """

    type: uint8

    bool_value: bool

    integer_value: int64

    double_value: float64

    string_value: str

    byte_array_value: List[uint8]

    bool_array_value: List[bool]

    integer_array_value: List[int64]

    double_array_value: List[float64]

    string_array_value: List[str]

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_862534def30ec11ac5860cce7f69c93db3d4fed5ff3c77fb22d6fd729420d234"


    # DDS Type Name
    DDS_TYPE_NAME = "rcl_interfaces::msg::dds_::ParameterValue_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['type'] = self.type



        result['bool_value'] = self.bool_value



        result['integer_value'] = self.integer_value



        result['double_value'] = self.double_value



        result['string_value'] = self.string_value



        result['byte_array_value'] = self.byte_array_value



        result['bool_array_value'] = self.bool_array_value



        result['integer_array_value'] = self.integer_array_value



        result['double_array_value'] = self.double_array_value



        result['string_array_value'] = self.string_array_value


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'type' in data:

            kwargs['type'] = data['type']


        if 'bool_value' in data:

            kwargs['bool_value'] = data['bool_value']


        if 'integer_value' in data:

            kwargs['integer_value'] = data['integer_value']


        if 'double_value' in data:

            kwargs['double_value'] = data['double_value']


        if 'string_value' in data:

            kwargs['string_value'] = data['string_value']


        if 'byte_array_value' in data:

            kwargs['byte_array_value'] = data['byte_array_value']


        if 'bool_array_value' in data:

            kwargs['bool_array_value'] = data['bool_array_value']


        if 'integer_array_value' in data:

            kwargs['integer_array_value'] = data['integer_array_value']


        if 'double_array_value' in data:

            kwargs['double_array_value'] = data['double_array_value']


        if 'string_array_value' in data:

            kwargs['string_array_value'] = data['string_array_value']


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

