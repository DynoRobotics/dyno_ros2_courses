"""
NestedMessage action type.

Auto-generated from test_msgs/action/NestedMessage.action
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


from ...builtin_interfaces.msg.time import Time



from ...unique_identifier_msgs.msg.uuid import UUID





# Import same-package message types

from ..msg.basictypes import BasicTypes

from ..msg.builtins import Builtins



if TYPE_CHECKING:
    pass


# ============================================================================
# Goal Message
# ============================================================================


@dataclass
class NestedMessage_Goal(IdlStruct, typename="test_msgs/NestedMessage_Goal"):

    """test_msgs/NestedMessage_Goal message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_4b5994765660c76a1b5f2a1452e537603f84babfb4f65b983f7f61d17b4e967e


    DDS type name: test_msgs::action::dds_::NestedMessage_Goal_

    """


    nested_field_no_pkg: Builtins = field(default_factory=Builtins)

    nested_field: BasicTypes = field(default_factory=BasicTypes)

    nested_different_pkg: Time = field(default_factory=Time)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_4b5994765660c76a1b5f2a1452e537603f84babfb4f65b983f7f61d17b4e967e"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_Goal_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['nested_field_no_pkg'] = self.nested_field_no_pkg.to_dict()



        result['nested_field'] = self.nested_field.to_dict()



        result['nested_different_pkg'] = self.nested_different_pkg.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'nested_field_no_pkg' in data:

            
            kwargs['nested_field_no_pkg'] = Builtins.from_dict(data['nested_field_no_pkg'])
            


        if 'nested_field' in data:

            
            kwargs['nested_field'] = BasicTypes.from_dict(data['nested_field'])
            


        if 'nested_different_pkg' in data:

            
            kwargs['nested_different_pkg'] = builtin_interfaces_msg.Time.from_dict(data['nested_different_pkg'])
            


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
class NestedMessage_Result(IdlStruct, typename="test_msgs/NestedMessage_Result"):

    """test_msgs/NestedMessage_Result message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_011ada6103e185a4ff3b32d2a0d06b7872b7454cf5adcb7939e2c4222a55fc00


    DDS type name: test_msgs::action::dds_::NestedMessage_Result_

    """


    nested_field_no_pkg: Builtins = field(default_factory=Builtins)

    nested_field: BasicTypes = field(default_factory=BasicTypes)

    nested_different_pkg: Time = field(default_factory=Time)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_011ada6103e185a4ff3b32d2a0d06b7872b7454cf5adcb7939e2c4222a55fc00"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_Result_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['nested_field_no_pkg'] = self.nested_field_no_pkg.to_dict()



        result['nested_field'] = self.nested_field.to_dict()



        result['nested_different_pkg'] = self.nested_different_pkg.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'nested_field_no_pkg' in data:

            
            kwargs['nested_field_no_pkg'] = Builtins.from_dict(data['nested_field_no_pkg'])
            


        if 'nested_field' in data:

            
            kwargs['nested_field'] = BasicTypes.from_dict(data['nested_field'])
            


        if 'nested_different_pkg' in data:

            
            kwargs['nested_different_pkg'] = builtin_interfaces_msg.Time.from_dict(data['nested_different_pkg'])
            


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
class NestedMessage_Feedback(IdlStruct, typename="test_msgs/NestedMessage_Feedback"):

    """test_msgs/NestedMessage_Feedback message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_2db4c7281b67f8315f7e44294555a62e4d6419b0fda3377cf384b0a3c0827a99


    DDS type name: test_msgs::action::dds_::NestedMessage_Feedback_

    """


    nested_field_no_pkg: Builtins = field(default_factory=Builtins)

    nested_field: BasicTypes = field(default_factory=BasicTypes)

    nested_different_pkg: Time = field(default_factory=Time)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_2db4c7281b67f8315f7e44294555a62e4d6419b0fda3377cf384b0a3c0827a99"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_Feedback_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['nested_field_no_pkg'] = self.nested_field_no_pkg.to_dict()



        result['nested_field'] = self.nested_field.to_dict()



        result['nested_different_pkg'] = self.nested_different_pkg.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'nested_field_no_pkg' in data:

            
            kwargs['nested_field_no_pkg'] = Builtins.from_dict(data['nested_field_no_pkg'])
            


        if 'nested_field' in data:

            
            kwargs['nested_field'] = BasicTypes.from_dict(data['nested_field'])
            


        if 'nested_different_pkg' in data:

            
            kwargs['nested_different_pkg'] = builtin_interfaces_msg.Time.from_dict(data['nested_different_pkg'])
            


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
class NestedMessage_FeedbackMessage(IdlStruct, typename="test_msgs/NestedMessage_FeedbackMessage"):

    """test_msgs/NestedMessage_FeedbackMessage message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_4fc6f30e52853f995a46313cdc33de52d147e0b6de695b2d8858812fc7e8490b


    DDS type name: test_msgs::action::dds_::NestedMessage_FeedbackMessage_

    """


    goal_id: UUID = field(default_factory=UUID)

    feedback: NestedMessage_Feedback = field(default_factory=NestedMessage_Feedback)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_4fc6f30e52853f995a46313cdc33de52d147e0b6de695b2d8858812fc7e8490b"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_FeedbackMessage_"


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

            
            kwargs['feedback'] = NestedMessage_Feedback.from_dict(data['feedback'])
            


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
class NestedMessage_SendGoal_Request(IdlStruct, typename="test_msgs/NestedMessage_SendGoal_Request"):

    """test_msgs/NestedMessage_SendGoal_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_1db7a9d9b74d5f424de541fa60663cf423d33d92a69b338ce8c733e282bb53ed


    DDS type name: test_msgs::action::dds_::NestedMessage_SendGoal_Request_

    """


    goal_id: UUID = field(default_factory=UUID)

    goal: NestedMessage_Goal = field(default_factory=NestedMessage_Goal)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_1db7a9d9b74d5f424de541fa60663cf423d33d92a69b338ce8c733e282bb53ed"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_SendGoal_Request_"


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

            
            kwargs['goal'] = NestedMessage_Goal.from_dict(data['goal'])
            


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
class NestedMessage_SendGoal_Response(IdlStruct, typename="test_msgs/NestedMessage_SendGoal_Response"):

    """test_msgs/NestedMessage_SendGoal_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_e78d269a75544304ab7f6d700ba1cb718875360bb84007906227267f9b124381


    DDS type name: test_msgs::action::dds_::NestedMessage_SendGoal_Response_

    """


    accepted: bool = False

    stamp: Time = field(default_factory=Time)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_e78d269a75544304ab7f6d700ba1cb718875360bb84007906227267f9b124381"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_SendGoal_Response_"


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




class NestedMessage_SendGoal:
    """SendGoal service for NestedMessage action."""
    Request = NestedMessage_SendGoal_Request
    Response = NestedMessage_SendGoal_Response
    TYPE_HASH = "RIHS01_d8fd7776dd2d5b4e42a6ae7f3894145a12f96c506084ebc71dd615f5c6b3c032"


# ============================================================================
# GetResult Service
# ============================================================================


@dataclass
class NestedMessage_GetResult_Request(IdlStruct, typename="test_msgs/NestedMessage_GetResult_Request"):

    """test_msgs/NestedMessage_GetResult_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_687ebbfa75d95d5d0f23ca800c213b4777fef311028d94a5d353ef7377f25741


    DDS type name: test_msgs::action::dds_::NestedMessage_GetResult_Request_

    """


    goal_id: UUID = field(default_factory=UUID)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_687ebbfa75d95d5d0f23ca800c213b4777fef311028d94a5d353ef7377f25741"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_GetResult_Request_"


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
class NestedMessage_GetResult_Response(IdlStruct, typename="test_msgs/NestedMessage_GetResult_Response"):

    """test_msgs/NestedMessage_GetResult_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_f782802147843a9dce51b1d164fc8fd9d939ca343e251196366d07650ad1d359


    DDS type name: test_msgs::action::dds_::NestedMessage_GetResult_Response_

    """


    status: int8 = 0

    result: NestedMessage_Result = field(default_factory=NestedMessage_Result)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_f782802147843a9dce51b1d164fc8fd9d939ca343e251196366d07650ad1d359"


    # DDS Type Name
    DDS_TYPE_NAME = "test_msgs::action::dds_::NestedMessage_GetResult_Response_"


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

            
            kwargs['result'] = NestedMessage_Result.from_dict(data['result'])
            


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




class NestedMessage_GetResult:
    """GetResult service for NestedMessage action."""
    Request = NestedMessage_GetResult_Request
    Response = NestedMessage_GetResult_Response
    TYPE_HASH = "RIHS01_5b0188d78902ad61bc173ec0ca0cbd877cb54b05a3a7e29686289b1b2cc25d77"


class NestedMessage:
    """
    Action type for test_msgs/NestedMessage.
    
    Goal: NestedMessage_Goal
    Result: NestedMessage_Result
    Feedback: NestedMessage_Feedback
    """
    # Main action types
    Goal = NestedMessage_Goal
    Result = NestedMessage_Result
    Feedback = NestedMessage_Feedback
    
    # Generated types for action protocol
    FeedbackMessage = NestedMessage_FeedbackMessage
    SendGoal = NestedMessage_SendGoal
    GetResult = NestedMessage_GetResult
    
    # Action type hash (computed from Goal + Result + Feedback)
    TYPE_HASH = "RIHS01_157e0f02eb8c56ee7a09dc0e94176693d1d84192d92a4be2c71839c49c33037c"
    
    def __init__(self):
        """Actions are not meant to be instantiated directly."""
        raise TypeError("Action types are not meant to be instantiated. "
                        "Use NestedMessage.Goal, NestedMessage.Result, NestedMessage.Feedback instead.")


__all__ = ['NestedMessage']

