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

from .vector3 import Vector3




if TYPE_CHECKING:
    pass


@dataclass
class Inertia(IdlStruct, typename="geometry_msgs/Inertia"):

    """geometry_msgs/Inertia message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_2ddd5dab5c347825ba2e56c895ddccfd0b8efe53ae931bf67f905529930b4bd7


    DDS type name: geometry_msgs::msg::dds_::Inertia_

    """

    m: float64

    com: Vector3

    ixx: float64

    ixy: float64

    ixz: float64

    iyy: float64

    iyz: float64

    izz: float64

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_2ddd5dab5c347825ba2e56c895ddccfd0b8efe53ae931bf67f905529930b4bd7"


    # DDS Type Name
    DDS_TYPE_NAME = "geometry_msgs::msg::dds_::Inertia_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['m'] = self.m



        result['com'] = self.com.to_dict()



        result['ixx'] = self.ixx



        result['ixy'] = self.ixy



        result['ixz'] = self.ixz



        result['iyy'] = self.iyy



        result['iyz'] = self.iyz



        result['izz'] = self.izz


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'm' in data:

            kwargs['m'] = data['m']


        if 'com' in data:

            kwargs['com'] = Vector3.from_dict(data['com'])


        if 'ixx' in data:

            kwargs['ixx'] = data['ixx']


        if 'ixy' in data:

            kwargs['ixy'] = data['ixy']


        if 'ixz' in data:

            kwargs['ixz'] = data['ixz']


        if 'iyy' in data:

            kwargs['iyy'] = data['iyy']


        if 'iyz' in data:

            kwargs['iyz'] = data['iyz']


        if 'izz' in data:

            kwargs['izz'] = data['izz']


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

