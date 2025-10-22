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

from ...geometry_msgs import msg as geometry_msgs_msg



if TYPE_CHECKING:
    pass




@dataclass
class Imu(IdlStruct, typename="sensor_msgs/Imu"):

    """sensor_msgs/Imu message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b


    DDS type name: sensor_msgs::msg::dds_::Imu_

    """


    header: std_msgs_msg.Header = field(default_factory=lambda: std_msgs_msg.Header())

    orientation: geometry_msgs_msg.Quaternion = field(default_factory=lambda: geometry_msgs_msg.Quaternion())

    orientation_covariance: array[float64, 9] = field(default_factory=lambda: [0.0] * 9)

    angular_velocity: geometry_msgs_msg.Vector3 = field(default_factory=lambda: geometry_msgs_msg.Vector3())

    angular_velocity_covariance: array[float64, 9] = field(default_factory=lambda: [0.0] * 9)

    linear_acceleration: geometry_msgs_msg.Vector3 = field(default_factory=lambda: geometry_msgs_msg.Vector3())

    linear_acceleration_covariance: array[float64, 9] = field(default_factory=lambda: [0.0] * 9)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::Imu_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['orientation'] = self.orientation.to_dict()



        result['orientation_covariance'] = self.orientation_covariance



        result['angular_velocity'] = self.angular_velocity.to_dict()



        result['angular_velocity_covariance'] = self.angular_velocity_covariance



        result['linear_acceleration'] = self.linear_acceleration.to_dict()



        result['linear_acceleration_covariance'] = self.linear_acceleration_covariance


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'header' in data:

            
            kwargs['header'] = std_msgs_msg.Header.from_dict(data['header'])
            


        if 'orientation' in data:

            
            kwargs['orientation'] = geometry_msgs_msg.Quaternion.from_dict(data['orientation'])
            


        if 'orientation_covariance' in data:

            kwargs['orientation_covariance'] = data['orientation_covariance']


        if 'angular_velocity' in data:

            
            kwargs['angular_velocity'] = geometry_msgs_msg.Vector3.from_dict(data['angular_velocity'])
            


        if 'angular_velocity_covariance' in data:

            kwargs['angular_velocity_covariance'] = data['angular_velocity_covariance']


        if 'linear_acceleration' in data:

            
            kwargs['linear_acceleration'] = geometry_msgs_msg.Vector3.from_dict(data['linear_acceleration'])
            


        if 'linear_acceleration_covariance' in data:

            kwargs['linear_acceleration_covariance'] = data['linear_acceleration_covariance']


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

