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


# Import types from same package

from .basictypes import BasicTypes

from .constants import Constants

from .defaults import Defaults




if TYPE_CHECKING:
    pass




@dataclass
class BoundedSequences(IdlStruct, typename="test_msgs/BoundedSequences"):

    """test_msgs/BoundedSequences message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_500f160527d0ceb907bd896937cebcd74d68a59ad4e5601c1c29e73ab8b06bf2


    DDS type name: test_msgs::msg::dds_::BoundedSequences_

    """


    bool_values: List[bool] = field(default_factory=list)

    byte_values: List[uint8] = field(default_factory=list)

    char_values: List[uint8] = field(default_factory=list)

    float32_values: List[float32] = field(default_factory=list)

    float64_values: List[float64] = field(default_factory=list)

    int8_values: List[int8] = field(default_factory=list)

    uint8_values: List[uint8] = field(default_factory=list)

    int16_values: List[int16] = field(default_factory=list)

    uint16_values: List[uint16] = field(default_factory=list)

    int32_values: List[int32] = field(default_factory=list)

    uint32_values: List[uint32] = field(default_factory=list)

    int64_values: List[int64] = field(default_factory=list)

    uint64_values: List[uint64] = field(default_factory=list)

    string_values: List[str] = field(default_factory=list)

    basic_types_values: List[BasicTypes] = field(default_factory=list)

    constants_values: List[Constants] = field(default_factory=list)

    defaults_values: List[Defaults] = field(default_factory=list)

    bool_values_default: List[bool] = field(default_factory=lambda: [False, True, False])

    byte_values_default: List[uint8] = field(default_factory=lambda: [0, 1, 255])

    char_values_default: List[uint8] = field(default_factory=lambda: [0, 1, 127])

    float32_values_default: List[float32] = field(default_factory=lambda: [1.125, 0.0, -1.125])

    float64_values_default: List[float64] = field(default_factory=lambda: [3.1415, 0.0, -3.1415])

    int8_values_default: List[int8] = field(default_factory=lambda: [0, 127, -128])

    uint8_values_default: List[uint8] = field(default_factory=lambda: [0, 1, 255])

    int16_values_default: List[int16] = field(default_factory=lambda: [0, 32767, -32768])

    uint16_values_default: List[uint16] = field(default_factory=lambda: [0, 1, 65535])

    int32_values_default: List[int32] = field(default_factory=lambda: [0, 2147483647, -2147483648])

    uint32_values_default: List[uint32] = field(default_factory=lambda: [0, 1, 4294967295])

    int64_values_default: List[int64] = field(default_factory=lambda: [0, 9223372036854775807, -9223372036854775808])

    uint64_values_default: List[uint64] = field(default_factory=lambda: [0, 1, 18446744073709551615])

    string_values_default: List[str] = field(default_factory=lambda: ["", "max value", "min value"])

    alignment_check: int32 = 0

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_500f160527d0ceb907bd896937cebcd74d68a59ad4e5601c1c29e73ab8b06bf2"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::msg::dds_::BoundedSequences_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['bool_values'] = self.bool_values



        result['byte_values'] = self.byte_values



        result['char_values'] = self.char_values



        result['float32_values'] = self.float32_values



        result['float64_values'] = self.float64_values



        result['int8_values'] = self.int8_values



        result['uint8_values'] = self.uint8_values



        result['int16_values'] = self.int16_values



        result['uint16_values'] = self.uint16_values



        result['int32_values'] = self.int32_values



        result['uint32_values'] = self.uint32_values



        result['int64_values'] = self.int64_values



        result['uint64_values'] = self.uint64_values



        result['string_values'] = self.string_values



        result['basic_types_values'] = [item.to_dict() for item in self.basic_types_values]



        result['constants_values'] = [item.to_dict() for item in self.constants_values]



        result['defaults_values'] = [item.to_dict() for item in self.defaults_values]



        result['bool_values_default'] = self.bool_values_default



        result['byte_values_default'] = self.byte_values_default



        result['char_values_default'] = self.char_values_default



        result['float32_values_default'] = self.float32_values_default



        result['float64_values_default'] = self.float64_values_default



        result['int8_values_default'] = self.int8_values_default



        result['uint8_values_default'] = self.uint8_values_default



        result['int16_values_default'] = self.int16_values_default



        result['uint16_values_default'] = self.uint16_values_default



        result['int32_values_default'] = self.int32_values_default



        result['uint32_values_default'] = self.uint32_values_default



        result['int64_values_default'] = self.int64_values_default



        result['uint64_values_default'] = self.uint64_values_default



        result['string_values_default'] = self.string_values_default



        result['alignment_check'] = self.alignment_check


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'bool_values' in data:

            kwargs['bool_values'] = data['bool_values']


        if 'byte_values' in data:

            kwargs['byte_values'] = data['byte_values']


        if 'char_values' in data:

            kwargs['char_values'] = data['char_values']


        if 'float32_values' in data:

            kwargs['float32_values'] = data['float32_values']


        if 'float64_values' in data:

            kwargs['float64_values'] = data['float64_values']


        if 'int8_values' in data:

            kwargs['int8_values'] = data['int8_values']


        if 'uint8_values' in data:

            kwargs['uint8_values'] = data['uint8_values']


        if 'int16_values' in data:

            kwargs['int16_values'] = data['int16_values']


        if 'uint16_values' in data:

            kwargs['uint16_values'] = data['uint16_values']


        if 'int32_values' in data:

            kwargs['int32_values'] = data['int32_values']


        if 'uint32_values' in data:

            kwargs['uint32_values'] = data['uint32_values']


        if 'int64_values' in data:

            kwargs['int64_values'] = data['int64_values']


        if 'uint64_values' in data:

            kwargs['uint64_values'] = data['uint64_values']


        if 'string_values' in data:

            kwargs['string_values'] = data['string_values']


        if 'basic_types_values' in data:

            
            kwargs['basic_types_values'] = [BasicTypes.from_dict(item) for item in data['basic_types_values']]
            


        if 'constants_values' in data:

            
            kwargs['constants_values'] = [Constants.from_dict(item) for item in data['constants_values']]
            


        if 'defaults_values' in data:

            
            kwargs['defaults_values'] = [Defaults.from_dict(item) for item in data['defaults_values']]
            


        if 'bool_values_default' in data:

            kwargs['bool_values_default'] = data['bool_values_default']


        if 'byte_values_default' in data:

            kwargs['byte_values_default'] = data['byte_values_default']


        if 'char_values_default' in data:

            kwargs['char_values_default'] = data['char_values_default']


        if 'float32_values_default' in data:

            kwargs['float32_values_default'] = data['float32_values_default']


        if 'float64_values_default' in data:

            kwargs['float64_values_default'] = data['float64_values_default']


        if 'int8_values_default' in data:

            kwargs['int8_values_default'] = data['int8_values_default']


        if 'uint8_values_default' in data:

            kwargs['uint8_values_default'] = data['uint8_values_default']


        if 'int16_values_default' in data:

            kwargs['int16_values_default'] = data['int16_values_default']


        if 'uint16_values_default' in data:

            kwargs['uint16_values_default'] = data['uint16_values_default']


        if 'int32_values_default' in data:

            kwargs['int32_values_default'] = data['int32_values_default']


        if 'uint32_values_default' in data:

            kwargs['uint32_values_default'] = data['uint32_values_default']


        if 'int64_values_default' in data:

            kwargs['int64_values_default'] = data['int64_values_default']


        if 'uint64_values_default' in data:

            kwargs['uint64_values_default'] = data['uint64_values_default']


        if 'string_values_default' in data:

            kwargs['string_values_default'] = data['string_values_default']


        if 'alignment_check' in data:

            kwargs['alignment_check'] = data['alignment_check']


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

