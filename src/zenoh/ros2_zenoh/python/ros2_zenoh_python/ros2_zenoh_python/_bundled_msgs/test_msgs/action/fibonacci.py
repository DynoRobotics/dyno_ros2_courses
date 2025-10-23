"""
Fibonacci action type.

Auto-generated from test_msgs/action/Fibonacci.action
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
class Fibonacci_Goal(IdlStruct, typename="test_msgs/Fibonacci_Goal"):

    """test_msgs/Fibonacci_Goal message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_e58ef5a8f4f039ab522a740823547c460b1d5aed8031c8b7825d33d345d90df8


    DDS type name: test_msgs::action::dds_::Fibonacci_Goal_

    """


    order: int32 = 0

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_e58ef5a8f4f039ab522a740823547c460b1d5aed8031c8b7825d33d345d90df8"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_Goal_"


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
class Fibonacci_Result(IdlStruct, typename="test_msgs/Fibonacci_Result"):

    """test_msgs/Fibonacci_Result message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_0774b095467813c6904d475498b5486b7ebd4cda6a4569f97e8de8c862689049


    DDS type name: test_msgs::action::dds_::Fibonacci_Result_

    """


    sequence: List[int32] = field(default_factory=list)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_0774b095467813c6904d475498b5486b7ebd4cda6a4569f97e8de8c862689049"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_Result_"


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
class Fibonacci_Feedback(IdlStruct, typename="test_msgs/Fibonacci_Feedback"):

    """test_msgs/Fibonacci_Feedback message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_7d1110f2efa019ad5ad8ad2244db98d862f39dc9fae8c976814b0065fa2f32d5


    DDS type name: test_msgs::action::dds_::Fibonacci_Feedback_

    """


    sequence: List[int32] = field(default_factory=list)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_7d1110f2efa019ad5ad8ad2244db98d862f39dc9fae8c976814b0065fa2f32d5"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_Feedback_"


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
class Fibonacci_FeedbackMessage(IdlStruct, typename="test_msgs/Fibonacci_FeedbackMessage"):

    """test_msgs/Fibonacci_FeedbackMessage message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_37487bae8cea358366b97003c1ef1ca7ce7c08df36f9e8cca496624226aee64e


    DDS type name: test_msgs::action::dds_::Fibonacci_FeedbackMessage_

    """


    goal_id: UUID = field(default_factory=UUID)

    feedback: Fibonacci_Feedback = field(default_factory=Fibonacci_Feedback)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_37487bae8cea358366b97003c1ef1ca7ce7c08df36f9e8cca496624226aee64e"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_FeedbackMessage_"


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
class Fibonacci_SendGoal_Request(IdlStruct, typename="test_msgs/Fibonacci_SendGoal_Request"):

    """test_msgs/Fibonacci_SendGoal_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_dfe75c73dc2c090cbc8fd8c45039bfa279d16aa6aadd348f812c6adad26e669e


    DDS type name: test_msgs::action::dds_::Fibonacci_SendGoal_Request_

    """


    goal_id: UUID = field(default_factory=UUID)

    goal: Fibonacci_Goal = field(default_factory=Fibonacci_Goal)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_dfe75c73dc2c090cbc8fd8c45039bfa279d16aa6aadd348f812c6adad26e669e"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_SendGoal_Request_"


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
class Fibonacci_SendGoal_Response(IdlStruct, typename="test_msgs/Fibonacci_SendGoal_Response"):

    """test_msgs/Fibonacci_SendGoal_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_1e7e84f7edf1fb020d9e51c7842bceef4cbdbd847f354125f3227cf4f314a683


    DDS type name: test_msgs::action::dds_::Fibonacci_SendGoal_Response_

    """


    accepted: bool = False

    stamp: Time = field(default_factory=Time)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_1e7e84f7edf1fb020d9e51c7842bceef4cbdbd847f354125f3227cf4f314a683"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_SendGoal_Response_"


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
    TYPE_HASH = "RIHS01_d807633803e38e164276573cd97e9fbd71e109c290bbf4cc312a0a1c9da7bbd9"


# ============================================================================
# GetResult Service
# ============================================================================


@dataclass
class Fibonacci_GetResult_Request(IdlStruct, typename="test_msgs/Fibonacci_GetResult_Request"):

    """test_msgs/Fibonacci_GetResult_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_c65e93c4c4328d7f1abd6ace83dba8e5fa0f32de76b4d2c96b5c655d94803b92


    DDS type name: test_msgs::action::dds_::Fibonacci_GetResult_Request_

    """


    goal_id: UUID = field(default_factory=UUID)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_c65e93c4c4328d7f1abd6ace83dba8e5fa0f32de76b4d2c96b5c655d94803b92"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_GetResult_Request_"


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
class Fibonacci_GetResult_Response(IdlStruct, typename="test_msgs/Fibonacci_GetResult_Response"):

    """test_msgs/Fibonacci_GetResult_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_ef88159d47a3d21d9bf4bec690cf668939d9542db28a9cfd77dbcd97eaf1add2


    DDS type name: test_msgs::action::dds_::Fibonacci_GetResult_Response_

    """


    status: int8 = 0

    result: Fibonacci_Result = field(default_factory=Fibonacci_Result)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_ef88159d47a3d21d9bf4bec690cf668939d9542db28a9cfd77dbcd97eaf1add2"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::Fibonacci_GetResult_Response_"


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
    TYPE_HASH = "RIHS01_5d30fdeb21d20e117aa5f36c7b4f0b6874f1a8cf7a9c038d58f1156c5737b209"


class Fibonacci:
    """
    Action type for test_msgs/Fibonacci.
    
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
    TYPE_HASH = "RIHS01_52dd94fea566c00ef96e572247ecf1ea6ef80a962e09e8d9f1ae788e9f818e8d"
    
    def __init__(self):
        """Actions are not meant to be instantiated directly."""
        raise TypeError("Action types are not meant to be instantiated. "
                        "Use Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback instead.")


__all__ = ['Fibonacci']

