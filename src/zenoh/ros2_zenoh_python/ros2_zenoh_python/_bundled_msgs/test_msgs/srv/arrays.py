"""
Arrays service type.

Auto-generated from test_msgs/srv/Arrays.srv
"""







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

from ..msg.basictypes import BasicTypes

from ..msg.constants import Constants

from ..msg.defaults import Defaults




if TYPE_CHECKING:
    pass




@dataclass
class Arrays_Request(IdlStruct, typename="test_msgs/Arrays_Request"):

    """test_msgs/Arrays_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_055147da4e6bef5aa12dac2a1a5e09c89f0292e3d2b8230809c169e70c077796


    DDS type name: test_msgs::srv::dds_::Arrays_Request_

    """


    bool_values: array[bool, 3] = field(default_factory=lambda: [False] * 3)

    byte_values: array[uint8, 3] = field(default_factory=lambda: [0] * 3)

    char_values: array[uint8, 3] = field(default_factory=lambda: [0] * 3)

    float32_values: array[float32, 3] = field(default_factory=lambda: [0.0] * 3)

    float64_values: array[float64, 3] = field(default_factory=lambda: [0.0] * 3)

    int8_values: array[int8, 3] = field(default_factory=lambda: [0] * 3)

    uint8_values: array[uint8, 3] = field(default_factory=lambda: [0] * 3)

    int16_values: array[int16, 3] = field(default_factory=lambda: [0] * 3)

    uint16_values: array[uint16, 3] = field(default_factory=lambda: [0] * 3)

    int32_values: array[int32, 3] = field(default_factory=lambda: [0] * 3)

    uint32_values: array[uint32, 3] = field(default_factory=lambda: [0] * 3)

    int64_values: array[int64, 3] = field(default_factory=lambda: [0] * 3)

    uint64_values: array[uint64, 3] = field(default_factory=lambda: [0] * 3)

    string_values: array[str, 3] = field(default_factory=lambda: [""] * 3)

    basic_types_values: array[BasicTypes, 3] = field(default_factory=lambda: [BasicTypes() for _ in range(3)])

    constants_values: array[Constants, 3] = field(default_factory=lambda: [Constants() for _ in range(3)])

    defaults_values: array[Defaults, 3] = field(default_factory=lambda: [Defaults() for _ in range(3)])

    bool_values_default: array[bool, 3] = field(default_factory=lambda: [False, True, False])

    byte_values_default: array[uint8, 3] = field(default_factory=lambda: [0, 1, 255])

    char_values_default: array[uint8, 3] = field(default_factory=lambda: [0, 1, 127])

    float32_values_default: array[float32, 3] = field(default_factory=lambda: [1.125, 0.0, -1.125])

    float64_values_default: array[float64, 3] = field(default_factory=lambda: [3.1415, 0.0, -3.1415])

    int8_values_default: array[int8, 3] = field(default_factory=lambda: [0, 127, -128])

    uint8_values_default: array[uint8, 3] = field(default_factory=lambda: [0, 1, 255])

    int16_values_default: array[int16, 3] = field(default_factory=lambda: [0, 32767, -32768])

    uint16_values_default: array[uint16, 3] = field(default_factory=lambda: [0, 1, 65535])

    int32_values_default: array[int32, 3] = field(default_factory=lambda: [0, 2147483647, -2147483648])

    uint32_values_default: array[uint32, 3] = field(default_factory=lambda: [0, 1, 4294967295])

    int64_values_default: array[int64, 3] = field(default_factory=lambda: [0, 9223372036854775807, -9223372036854775808])

    uint64_values_default: array[uint64, 3] = field(default_factory=lambda: [0, 1, 18446744073709551615])

    string_values_default: array[str, 3] = field(default_factory=lambda: ["", "max value", "min value"])

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_055147da4e6bef5aa12dac2a1a5e09c89f0292e3d2b8230809c169e70c077796"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::srv::dds_::Arrays_Request_"


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

from ..msg.basictypes import BasicTypes

from ..msg.constants import Constants

from ..msg.defaults import Defaults




if TYPE_CHECKING:
    pass




@dataclass
class Arrays_Response(IdlStruct, typename="test_msgs/Arrays_Response"):

    """test_msgs/Arrays_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_0069fd33cbc1e2cad73cdf7f235e91aee4a54c60674a051cd3af38e33107a697


    DDS type name: test_msgs::srv::dds_::Arrays_Response_

    """


    bool_values: array[bool, 3] = field(default_factory=lambda: [False] * 3)

    byte_values: array[uint8, 3] = field(default_factory=lambda: [0] * 3)

    char_values: array[uint8, 3] = field(default_factory=lambda: [0] * 3)

    float32_values: array[float32, 3] = field(default_factory=lambda: [0.0] * 3)

    float64_values: array[float64, 3] = field(default_factory=lambda: [0.0] * 3)

    int8_values: array[int8, 3] = field(default_factory=lambda: [0] * 3)

    uint8_values: array[uint8, 3] = field(default_factory=lambda: [0] * 3)

    int16_values: array[int16, 3] = field(default_factory=lambda: [0] * 3)

    uint16_values: array[uint16, 3] = field(default_factory=lambda: [0] * 3)

    int32_values: array[int32, 3] = field(default_factory=lambda: [0] * 3)

    uint32_values: array[uint32, 3] = field(default_factory=lambda: [0] * 3)

    int64_values: array[int64, 3] = field(default_factory=lambda: [0] * 3)

    uint64_values: array[uint64, 3] = field(default_factory=lambda: [0] * 3)

    string_values: array[str, 3] = field(default_factory=lambda: [""] * 3)

    basic_types_values: array[BasicTypes, 3] = field(default_factory=lambda: [BasicTypes() for _ in range(3)])

    constants_values: array[Constants, 3] = field(default_factory=lambda: [Constants() for _ in range(3)])

    defaults_values: array[Defaults, 3] = field(default_factory=lambda: [Defaults() for _ in range(3)])

    bool_values_default: array[bool, 3] = field(default_factory=lambda: [False, True, False])

    byte_values_default: array[uint8, 3] = field(default_factory=lambda: [0, 1, 255])

    char_values_default: array[uint8, 3] = field(default_factory=lambda: [0, 1, 127])

    float32_values_default: array[float32, 3] = field(default_factory=lambda: [1.125, 0.0, -1.125])

    float64_values_default: array[float64, 3] = field(default_factory=lambda: [3.1415, 0.0, -3.1415])

    int8_values_default: array[int8, 3] = field(default_factory=lambda: [0, 127, -128])

    uint8_values_default: array[uint8, 3] = field(default_factory=lambda: [0, 1, 255])

    int16_values_default: array[int16, 3] = field(default_factory=lambda: [0, 32767, -32768])

    uint16_values_default: array[uint16, 3] = field(default_factory=lambda: [0, 1, 65535])

    int32_values_default: array[int32, 3] = field(default_factory=lambda: [0, 2147483647, -2147483648])

    uint32_values_default: array[uint32, 3] = field(default_factory=lambda: [0, 1, 4294967295])

    int64_values_default: array[int64, 3] = field(default_factory=lambda: [0, 9223372036854775807, -9223372036854775808])

    uint64_values_default: array[uint64, 3] = field(default_factory=lambda: [0, 1, 18446744073709551615])

    string_values_default: array[str, 3] = field(default_factory=lambda: ["", "max value", "min value"])

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_0069fd33cbc1e2cad73cdf7f235e91aee4a54c60674a051cd3af38e33107a697"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::srv::dds_::Arrays_Response_"


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




class Arrays:
    """
    Service type for test_msgs/Arrays.
    
    Request: Arrays_Request
    Response: Arrays_Response
    """
    Request = Arrays_Request
    Response = Arrays_Response
    
    # ROS2 Service Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_242c6436914e23d3524357e6a5655ccaa64f9a14ca080c963aa72482ebd43429"

