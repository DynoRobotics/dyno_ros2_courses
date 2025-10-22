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
class NavSatStatus(IdlStruct, typename="sensor_msgs/NavSatStatus"):

    """sensor_msgs/NavSatStatus message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_d1ed3befa628e09571bd273b888ba1c1fd187c9a5e0006b385d7e5e9095a3204


    DDS type name: sensor_msgs::msg::dds_::NavSatStatus_

    """

    # Constants

    STATUS_UNKNOWN = -2        # status is not yet set

    STATUS_NO_FIX = -1        # unable to fix position

    STATUS_FIX = 0        # unaugmented fix

    STATUS_SBAS_FIX = 1        # with satellite-based augmentation

    STATUS_GBAS_FIX = 2        # with ground-based augmentation

    SERVICE_UNKNOWN = 0  # Remember service is a bitfield, so checking (service & SERVICE_UNKNOWN) will not work. Use == instead.

    SERVICE_GPS = 1

    SERVICE_GLONASS = 2

    SERVICE_COMPASS = 4      # includes BeiDou.

    SERVICE_GALILEO = 8



    status: int8 = -2

    service: uint16 = 0

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_d1ed3befa628e09571bd273b888ba1c1fd187c9a5e0006b385d7e5e9095a3204"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::NavSatStatus_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['status'] = self.status



        result['service'] = self.service


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'status' in data:

            kwargs['status'] = data['status']


        if 'service' in data:

            kwargs['service'] = data['service']


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

