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
class Strings(IdlStruct, typename="test_msgs/Strings"):

    """test_msgs/Strings message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_7c7a232d70cdf571ff74ffc992cbf8e807913618b9a36d9f970bfd628850e743


    DDS type name: test_msgs::msg::dds_::Strings_

    """

    # Constants

    STRING_CONST = "Hello world!"



    string_value: str = ""

    string_value_default1: str = "Hello world!"

    string_value_default2: str = "Hello'world!"

    string_value_default3: str = "Hello\"world!"

    string_value_default4: str = "Hello\\'world!"

    string_value_default5: str = "Hello\\\"world!"

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_7c7a232d70cdf571ff74ffc992cbf8e807913618b9a36d9f970bfd628850e743"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::msg::dds_::Strings_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['string_value'] = self.string_value



        result['string_value_default1'] = self.string_value_default1



        result['string_value_default2'] = self.string_value_default2



        result['string_value_default3'] = self.string_value_default3



        result['string_value_default4'] = self.string_value_default4



        result['string_value_default5'] = self.string_value_default5


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'string_value' in data:

            kwargs['string_value'] = data['string_value']


        if 'string_value_default1' in data:

            kwargs['string_value_default1'] = data['string_value_default1']


        if 'string_value_default2' in data:

            kwargs['string_value_default2'] = data['string_value_default2']


        if 'string_value_default3' in data:

            kwargs['string_value_default3'] = data['string_value_default3']


        if 'string_value_default4' in data:

            kwargs['string_value_default4'] = data['string_value_default4']


        if 'string_value_default5' in data:

            kwargs['string_value_default5'] = data['string_value_default5']


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

