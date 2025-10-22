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

from .vector3 import Vector3




if TYPE_CHECKING:
    pass




@dataclass
class Wrench(IdlStruct, typename="geometry_msgs/Wrench"):

    """geometry_msgs/Wrench message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_018e8519d57c16adbe97c9fe1460ef21fec7e31bc541de3d653a35895677ce52


    DDS type name: geometry_msgs::msg::dds_::Wrench_

    """


    force: Vector3 = field(default_factory=Vector3)

    torque: Vector3 = field(default_factory=Vector3)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_018e8519d57c16adbe97c9fe1460ef21fec7e31bc541de3d653a35895677ce52"


    # DDS Type Name
    DDS_TYPE_NAME = "geometry_msgs::msg::dds_::Wrench_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['force'] = self.force.to_dict()



        result['torque'] = self.torque.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'force' in data:

            
            kwargs['force'] = Vector3.from_dict(data['force'])
            


        if 'torque' in data:

            
            kwargs['torque'] = Vector3.from_dict(data['torque'])
            


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

