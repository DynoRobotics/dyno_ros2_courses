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



if TYPE_CHECKING:
    pass


@dataclass
class JointTrajectoryPoint(IdlStruct, typename="trajectory_msgs/JointTrajectoryPoint"):

    """trajectory_msgs/JointTrajectoryPoint message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_52202a5e457cf7f7888c78c605430ccdd89b7962f3ae92cc5338c4a62881dba1


    DDS type name: trajectory_msgs::msg::dds_::JointTrajectoryPoint_

    """

    positions: List[float64]

    velocities: List[float64]

    accelerations: List[float64]

    effort: List[float64]

    time_from_start: 'ros2_interfaces_py.builtin_interfaces.msg.Duration'

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_52202a5e457cf7f7888c78c605430ccdd89b7962f3ae92cc5338c4a62881dba1"


    # DDS Type Name
    DDS_TYPE_NAME = "trajectory_msgs::msg::dds_::JointTrajectoryPoint_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['positions'] = self.positions



        result['velocities'] = self.velocities



        result['accelerations'] = self.accelerations



        result['effort'] = self.effort



        result['time_from_start'] = self.time_from_start.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'positions' in data:

            kwargs['positions'] = data['positions']


        if 'velocities' in data:

            kwargs['velocities'] = data['velocities']


        if 'accelerations' in data:

            kwargs['accelerations'] = data['accelerations']


        if 'effort' in data:

            kwargs['effort'] = data['effort']


        if 'time_from_start' in data:

            kwargs['time_from_start'] = Duration.from_dict(data['time_from_start'])


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

