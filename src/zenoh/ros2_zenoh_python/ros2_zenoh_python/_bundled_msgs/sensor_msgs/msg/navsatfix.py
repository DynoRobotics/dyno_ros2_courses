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

from .navsatstatus import NavSatStatus




# Import other package modules (not individual classes to avoid circular imports)

from ...std_msgs import msg as std_msgs_msg



if TYPE_CHECKING:
    pass




@dataclass
class NavSatFix(IdlStruct, typename="sensor_msgs/NavSatFix"):

    """sensor_msgs/NavSatFix message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_62223ab3fe210a15976021da7afddc9e200dc9ec75231c1b6a557fc598a65404


    DDS type name: sensor_msgs::msg::dds_::NavSatFix_

    """

    # Constants

    COVARIANCE_TYPE_UNKNOWN = 0

    COVARIANCE_TYPE_APPROXIMATED = 1

    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2

    COVARIANCE_TYPE_KNOWN = 3



    header: std_msgs_msg.Header = field(default_factory=lambda: std_msgs_msg.Header())

    status: NavSatStatus = field(default_factory=NavSatStatus)

    latitude: float64 = 0.0

    longitude: float64 = 0.0

    altitude: float64 = 0.0

    position_covariance: array[float64, 9] = field(default_factory=lambda: [0.0] * 9)

    position_covariance_type: uint8 = 0

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_62223ab3fe210a15976021da7afddc9e200dc9ec75231c1b6a557fc598a65404"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::NavSatFix_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['status'] = self.status.to_dict()



        result['latitude'] = self.latitude



        result['longitude'] = self.longitude



        result['altitude'] = self.altitude



        result['position_covariance'] = self.position_covariance



        result['position_covariance_type'] = self.position_covariance_type


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'header' in data:

            
            kwargs['header'] = std_msgs_msg.Header.from_dict(data['header'])
            


        if 'status' in data:

            
            kwargs['status'] = NavSatStatus.from_dict(data['status'])
            


        if 'latitude' in data:

            kwargs['latitude'] = data['latitude']


        if 'longitude' in data:

            kwargs['longitude'] = data['longitude']


        if 'altitude' in data:

            kwargs['altitude'] = data['altitude']


        if 'position_covariance' in data:

            kwargs['position_covariance'] = data['position_covariance']


        if 'position_covariance_type' in data:

            kwargs['position_covariance_type'] = data['position_covariance_type']


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

