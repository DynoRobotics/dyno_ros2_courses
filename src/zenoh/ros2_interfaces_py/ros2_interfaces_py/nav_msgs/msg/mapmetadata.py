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

import ros2_interfaces_py.builtin_interfaces.msg

import ros2_interfaces_py.geometry_msgs.msg



if TYPE_CHECKING:
    pass


@dataclass
class MapMetaData(IdlStruct, typename="nav_msgs/MapMetaData"):

    """nav_msgs/MapMetaData message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_9d0033ed8aaccc0b4b82483e11d2d225a1513423d378f2d01637a6f9268da357


    DDS type name: nav_msgs::msg::dds_::MapMetaData_

    """

    map_load_time: 'ros2_interfaces_py.builtin_interfaces.msg.Time'

    resolution: float32

    width: uint32

    height: uint32

    origin: 'ros2_interfaces_py.geometry_msgs.msg.Pose'

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_9d0033ed8aaccc0b4b82483e11d2d225a1513423d378f2d01637a6f9268da357"


    # DDS Type Name
    DDS_TYPE_NAME = "nav_msgs::msg::dds_::MapMetaData_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['map_load_time'] = self.map_load_time.to_dict()



        result['resolution'] = self.resolution



        result['width'] = self.width



        result['height'] = self.height



        result['origin'] = self.origin.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'map_load_time' in data:

            kwargs['map_load_time'] = Time.from_dict(data['map_load_time'])


        if 'resolution' in data:

            kwargs['resolution'] = data['resolution']


        if 'width' in data:

            kwargs['width'] = data['width']


        if 'height' in data:

            kwargs['height'] = data['height']


        if 'origin' in data:

            kwargs['origin'] = Pose.from_dict(data['origin'])


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

