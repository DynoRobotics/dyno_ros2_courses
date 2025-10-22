"""
CancelGoal service type.

Auto-generated from action_msgs/srv/CancelGoal.srv
"""







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

from ..msg.goalinfo import GoalInfo




if TYPE_CHECKING:
    pass




@dataclass
class CancelGoal_Request(IdlStruct, typename="action_msgs/CancelGoal_Request"):

    """action_msgs/CancelGoal_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_3d3c84653c1f96918086887e1dcb236faec88b81a5b14fd4cf4840065bcdf8af


    DDS type name: action_msgs::srv::dds_::CancelGoal_Request_

    """


    goal_info: GoalInfo = field(default_factory=GoalInfo)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_3d3c84653c1f96918086887e1dcb236faec88b81a5b14fd4cf4840065bcdf8af"


    # DDS Type Name
    DDS_TYPE_NAME = "action_msgs::srv::dds_::CancelGoal_Request_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['goal_info'] = self.goal_info.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'goal_info' in data:

            
            kwargs['goal_info'] = GoalInfo.from_dict(data['goal_info'])
            


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

from ..msg.goalinfo import GoalInfo




if TYPE_CHECKING:
    pass




@dataclass
class CancelGoal_Response(IdlStruct, typename="action_msgs/CancelGoal_Response"):

    """action_msgs/CancelGoal_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_35e682cf3f510e83c70a82a4aac888496dedee56773bf9d8e5e0aa81f9e1c960


    DDS type name: action_msgs::srv::dds_::CancelGoal_Response_

    """

    # Constants

    ERROR_NONE = 0

    ERROR_REJECTED = 1

    ERROR_UNKNOWN_GOAL_ID = 2

    ERROR_GOAL_TERMINATED = 3



    return_code: int8 = 0

    goals_canceling: List[GoalInfo] = field(default_factory=list)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_35e682cf3f510e83c70a82a4aac888496dedee56773bf9d8e5e0aa81f9e1c960"


    # DDS Type Name
    DDS_TYPE_NAME = "action_msgs::srv::dds_::CancelGoal_Response_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['return_code'] = self.return_code



        result['goals_canceling'] = [item.to_dict() for item in self.goals_canceling]


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'return_code' in data:

            kwargs['return_code'] = data['return_code']


        if 'goals_canceling' in data:

            
            kwargs['goals_canceling'] = [GoalInfo.from_dict(item) for item in data['goals_canceling']]
            


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




class CancelGoal:
    """
    Service type for action_msgs/CancelGoal.
    
    Request: CancelGoal_Request
    Response: CancelGoal_Response
    """
    Request = CancelGoal_Request
    Response = CancelGoal_Response
    
    # ROS2 Service Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_573d8b0a534451d7bc2ac8c5ffde8ac14b8593b7001175d0cd6516dcbeb8689a"

