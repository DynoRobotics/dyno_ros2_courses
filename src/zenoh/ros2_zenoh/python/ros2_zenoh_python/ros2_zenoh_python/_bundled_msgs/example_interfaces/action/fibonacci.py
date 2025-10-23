"""
Fibonacci action type.

Auto-generated from example_interfaces/action/Fibonacci.action
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


# Import specific types from sibling packages (import actual classes, not modules)


from ...unique_identifier_msgs.msg.uuid import UUID



from ...builtin_interfaces.msg.time import Time





if TYPE_CHECKING:
    pass


# ============================================================================
# Goal Message
# ============================================================================


@dataclass
class Fibonacci_Goal(IdlStruct, typename="example_interfaces/Fibonacci_Goal"):

    """example_interfaces/Fibonacci_Goal message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_226cb437e4355dcd3e914f930382a3b0cc1da81545bd319ed554e95a03255f51


    DDS type name: example_interfaces::action::dds_::Fibonacci_Goal_

    """


    order: int32 = 0

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_226cb437e4355dcd3e914f930382a3b0cc1da81545bd319ed554e95a03255f51"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_Goal_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['order'] = self.order


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'order' in data:

            kwargs['order'] = data['order']


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




# ============================================================================
# Result Message
# ============================================================================


@dataclass
class Fibonacci_Result(IdlStruct, typename="example_interfaces/Fibonacci_Result"):

    """example_interfaces/Fibonacci_Result message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_fea81394f25aa4502217953f1a021fb750e79c10bbd43f13dd94632da6569649


    DDS type name: example_interfaces::action::dds_::Fibonacci_Result_

    """


    sequence: List[int32] = field(default_factory=list)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_fea81394f25aa4502217953f1a021fb750e79c10bbd43f13dd94632da6569649"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_Result_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['sequence'] = self.sequence


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'sequence' in data:

            kwargs['sequence'] = data['sequence']


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




# ============================================================================
# Feedback Message
# ============================================================================


@dataclass
class Fibonacci_Feedback(IdlStruct, typename="example_interfaces/Fibonacci_Feedback"):

    """example_interfaces/Fibonacci_Feedback message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_2b12e37361da6f408d4c85bc24a18de64333f29082f2ca34b5ee33dc4c8b42a9


    DDS type name: example_interfaces::action::dds_::Fibonacci_Feedback_

    """


    sequence: List[int32] = field(default_factory=list)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_2b12e37361da6f408d4c85bc24a18de64333f29082f2ca34b5ee33dc4c8b42a9"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_Feedback_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['sequence'] = self.sequence


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'sequence' in data:

            kwargs['sequence'] = data['sequence']


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




# ============================================================================
# FeedbackMessage (with GoalInfo)
# ============================================================================


@dataclass
class Fibonacci_FeedbackMessage(IdlStruct, typename="example_interfaces/Fibonacci_FeedbackMessage"):

    """example_interfaces/Fibonacci_FeedbackMessage message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_c1de71afd52e49a89c53d8262366884185bc0a02f78ce051c4e46b0a7fe59bb2


    DDS type name: example_interfaces::action::dds_::Fibonacci_FeedbackMessage_

    """


    goal_id: UUID = field(default_factory=UUID)

    feedback: Fibonacci_Feedback = field(default_factory=Fibonacci_Feedback)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_c1de71afd52e49a89c53d8262366884185bc0a02f78ce051c4e46b0a7fe59bb2"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_FeedbackMessage_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['goal_id'] = self.goal_id.to_dict()



        result['feedback'] = self.feedback.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'goal_id' in data:

            
            kwargs['goal_id'] = unique_identifier_msgs_msg.UUID.from_dict(data['goal_id'])
            


        if 'feedback' in data:

            
            kwargs['feedback'] = Fibonacci_Feedback.from_dict(data['feedback'])
            


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




# ============================================================================
# SendGoal Service
# ============================================================================


@dataclass
class Fibonacci_SendGoal_Request(IdlStruct, typename="example_interfaces/Fibonacci_SendGoal_Request"):

    """example_interfaces/Fibonacci_SendGoal_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_3d088942b413247db536576f0286768c6be8fcd5d0c9a5d544f359fba090a238


    DDS type name: example_interfaces::action::dds_::Fibonacci_SendGoal_Request_

    """


    goal_id: UUID = field(default_factory=UUID)

    goal: Fibonacci_Goal = field(default_factory=Fibonacci_Goal)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_3d088942b413247db536576f0286768c6be8fcd5d0c9a5d544f359fba090a238"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_SendGoal_Request_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['goal_id'] = self.goal_id.to_dict()



        result['goal'] = self.goal.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'goal_id' in data:

            
            kwargs['goal_id'] = unique_identifier_msgs_msg.UUID.from_dict(data['goal_id'])
            


        if 'goal' in data:

            
            kwargs['goal'] = Fibonacci_Goal.from_dict(data['goal'])
            


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




@dataclass
class Fibonacci_SendGoal_Response(IdlStruct, typename="example_interfaces/Fibonacci_SendGoal_Response"):

    """example_interfaces/Fibonacci_SendGoal_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_d8c07bb3d5b766fe4b43159c9a5222af5214e2fcc29229b991d826166c512be1


    DDS type name: example_interfaces::action::dds_::Fibonacci_SendGoal_Response_

    """


    accepted: bool = False

    stamp: Time = field(default_factory=Time)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_d8c07bb3d5b766fe4b43159c9a5222af5214e2fcc29229b991d826166c512be1"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_SendGoal_Response_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['accepted'] = self.accepted



        result['stamp'] = self.stamp.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'accepted' in data:

            kwargs['accepted'] = data['accepted']


        if 'stamp' in data:

            
            kwargs['stamp'] = builtin_interfaces_msg.Time.from_dict(data['stamp'])
            


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




class Fibonacci_SendGoal:
    """SendGoal service for Fibonacci action."""
    Request = Fibonacci_SendGoal_Request
    Response = Fibonacci_SendGoal_Response
    TYPE_HASH = "RIHS01_d1a57fb2a4afe8c21e34fb10db206f16ce6729b28531141472df92277c55b557"


# ============================================================================
# GetResult Service
# ============================================================================


@dataclass
class Fibonacci_GetResult_Request(IdlStruct, typename="example_interfaces/Fibonacci_GetResult_Request"):

    """example_interfaces/Fibonacci_GetResult_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_c8a4f5e7d13b81286ee1043e2ecd084281cecf1ff06aaa799464f5f15479f003


    DDS type name: example_interfaces::action::dds_::Fibonacci_GetResult_Request_

    """


    goal_id: UUID = field(default_factory=UUID)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_c8a4f5e7d13b81286ee1043e2ecd084281cecf1ff06aaa799464f5f15479f003"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_GetResult_Request_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['goal_id'] = self.goal_id.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'goal_id' in data:

            
            kwargs['goal_id'] = unique_identifier_msgs_msg.UUID.from_dict(data['goal_id'])
            


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




@dataclass
class Fibonacci_GetResult_Response(IdlStruct, typename="example_interfaces/Fibonacci_GetResult_Response"):

    """example_interfaces/Fibonacci_GetResult_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_6021dc98ab9b4bbe395e48aa4de81ee5f68eb570f88358affcc648146668b24f


    DDS type name: example_interfaces::action::dds_::Fibonacci_GetResult_Response_

    """


    status: int8 = 0

    result: Fibonacci_Result = field(default_factory=Fibonacci_Result)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_6021dc98ab9b4bbe395e48aa4de81ee5f68eb570f88358affcc648146668b24f"


    # DDS Type Name
    DDS_TYPE_NAME = "example_interfaces::action::dds_::Fibonacci_GetResult_Response_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['status'] = self.status



        result['result'] = self.result.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'status' in data:

            kwargs['status'] = data['status']


        if 'result' in data:

            
            kwargs['result'] = Fibonacci_Result.from_dict(data['result'])
            


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




class Fibonacci_GetResult:
    """GetResult service for Fibonacci action."""
    Request = Fibonacci_GetResult_Request
    Response = Fibonacci_GetResult_Response
    TYPE_HASH = "RIHS01_1b0de0d5d29dc955d92f546706568428632771db13ec84c15ec1c1a59f424a57"


class Fibonacci:
    """
    Action type for example_interfaces/Fibonacci.
    
    Goal: Fibonacci_Goal
    Result: Fibonacci_Result
    Feedback: Fibonacci_Feedback
    """
    # Main action types
    Goal = Fibonacci_Goal
    Result = Fibonacci_Result
    Feedback = Fibonacci_Feedback
    
    # Generated types for action protocol
    FeedbackMessage = Fibonacci_FeedbackMessage
    SendGoal = Fibonacci_SendGoal
    GetResult = Fibonacci_GetResult
    
    # Action type hash (computed from Goal + Result + Feedback)
    TYPE_HASH = "RIHS01_b05efab29d1143e5d24bab05117f70f35f6c0cffca330fbbb8f3a73b96dedb42"
    
    def __init__(self):
        """Actions are not meant to be instantiated directly."""
        raise TypeError("Action types are not meant to be instantiated. "
                        "Use Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback instead.")


__all__ = ['Fibonacci']

