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



# Import other package modules (not individual classes to avoid circular imports)

from ...std_msgs import msg as std_msgs_msg



if TYPE_CHECKING:
    pass




@dataclass
class Range(IdlStruct, typename="sensor_msgs/Range"):

    """sensor_msgs/Range message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a


    DDS type name: sensor_msgs::msg::dds_::Range_

    """

    # Constants

    ULTRASOUND = 0

    INFRARED = 1



    header: std_msgs_msg.Header = field(default_factory=lambda: std_msgs_msg.Header())

    radiation_type: uint8 = 0

    field_of_view: float32 = 0.0

    min_range: float32 = 0.0

    max_range: float32 = 0.0

    range: float32 = 0.0

    variance: float32 = 0.0

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::Range_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['radiation_type'] = self.radiation_type



        result['field_of_view'] = self.field_of_view



        result['min_range'] = self.min_range



        result['max_range'] = self.max_range



        result['range'] = self.range



        result['variance'] = self.variance


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'header' in data:

            
            kwargs['header'] = std_msgs_msg.Header.from_dict(data['header'])
            


        if 'radiation_type' in data:

            kwargs['radiation_type'] = data['radiation_type']


        if 'field_of_view' in data:

            kwargs['field_of_view'] = data['field_of_view']


        if 'min_range' in data:

            kwargs['min_range'] = data['min_range']


        if 'max_range' in data:

            kwargs['max_range'] = data['max_range']


        if 'range' in data:

            kwargs['range'] = data['range']


        if 'variance' in data:

            kwargs['variance'] = data['variance']


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

