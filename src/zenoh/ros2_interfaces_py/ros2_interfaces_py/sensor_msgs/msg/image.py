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



# Import package.msg modules

import ros2_interfaces_py.std_msgs.msg



if TYPE_CHECKING:
    pass


@dataclass
class Image(IdlStruct, typename="sensor_msgs/Image"):

    """sensor_msgs/Image message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_9d771ac303652dfed570b8438c3b68e358592b2b2c23cacbbbaf1cb836f5e111


    DDS type name: sensor_msgs::msg::dds_::Image_

    """

    header: 'ros2_interfaces_py.std_msgs.msg.Header'

    height: uint32

    width: uint32

    encoding: str

    is_bigendian: uint8

    step: uint32

    data: List[uint8]

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_9d771ac303652dfed570b8438c3b68e358592b2b2c23cacbbbaf1cb836f5e111"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::Image_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['height'] = self.height



        result['width'] = self.width



        result['encoding'] = self.encoding



        result['is_bigendian'] = self.is_bigendian



        result['step'] = self.step



        result['data'] = self.data


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'header' in data:

            kwargs['header'] = Header.from_dict(data['header'])


        if 'height' in data:

            kwargs['height'] = data['height']


        if 'width' in data:

            kwargs['width'] = data['width']


        if 'encoding' in data:

            kwargs['encoding'] = data['encoding']


        if 'is_bigendian' in data:

            kwargs['is_bigendian'] = data['is_bigendian']


        if 'step' in data:

            kwargs['step'] = data['step']


        if 'data' in data:

            kwargs['data'] = data['data']


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

