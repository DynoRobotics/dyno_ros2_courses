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


# Import types from same package

from .pointfield import PointField




# Import package.msg modules

import ros2_interfaces_py.std_msgs.msg



if TYPE_CHECKING:
    pass


@dataclass
class PointCloud2(IdlStruct, typename="sensor_msgs/PointCloud2"):

    """sensor_msgs/PointCloud2 message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_d22a05547cf1e592e08d03628db06423491f7a33a81ffc9e8f67309843345acd


    DDS type name: sensor_msgs::msg::dds_::PointCloud2_

    """

    header: 'ros2_interfaces_py.std_msgs.msg.Header'

    height: uint32

    width: uint32

    fields: List[PointField]

    is_bigendian: bool

    point_step: uint32

    row_step: uint32

    data: List[uint8]

    is_dense: bool

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_d22a05547cf1e592e08d03628db06423491f7a33a81ffc9e8f67309843345acd"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::PointCloud2_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['height'] = self.height



        result['width'] = self.width



        result['fields'] = [item.to_dict() for item in self.fields]



        result['is_bigendian'] = self.is_bigendian



        result['point_step'] = self.point_step



        result['row_step'] = self.row_step



        result['data'] = self.data



        result['is_dense'] = self.is_dense


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


        if 'fields' in data:

            kwargs['fields'] = [PointField.from_dict(item) for item in data['fields']]


        if 'is_bigendian' in data:

            kwargs['is_bigendian'] = data['is_bigendian']


        if 'point_step' in data:

            kwargs['point_step'] = data['point_step']


        if 'row_step' in data:

            kwargs['row_step'] = data['row_step']


        if 'data' in data:

            kwargs['data'] = data['data']


        if 'is_dense' in data:

            kwargs['is_dense'] = data['is_dense']


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

