"""
SetCameraInfo service type.

Auto-generated from sensor_msgs/srv/SetCameraInfo.srv
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

from ..msg.camerainfo import CameraInfo




if TYPE_CHECKING:
    pass




@dataclass
class SetCameraInfo_Request(IdlStruct, typename="sensor_msgs/SetCameraInfo_Request"):

    """sensor_msgs/SetCameraInfo_Request message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_27dc5497730fdb6930e66cb879c210e31a83022acc5bbc999f24f55286029f87


    DDS type name: sensor_msgs::srv::dds_::SetCameraInfo_Request_

    """


    camera_info: CameraInfo = field(default_factory=CameraInfo)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_27dc5497730fdb6930e66cb879c210e31a83022acc5bbc999f24f55286029f87"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::srv::dds_::SetCameraInfo_Request_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['camera_info'] = self.camera_info.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'camera_info' in data:

            
            kwargs['camera_info'] = CameraInfo.from_dict(data['camera_info'])
            


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

from ..msg.camerainfo import CameraInfo




if TYPE_CHECKING:
    pass




@dataclass
class SetCameraInfo_Response(IdlStruct, typename="sensor_msgs/SetCameraInfo_Response"):

    """sensor_msgs/SetCameraInfo_Response message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_4db6d7df20cdcdc05430945e76fa58e1eab1285c15f31d26fad76e2c1b10299c


    DDS type name: sensor_msgs::srv::dds_::SetCameraInfo_Response_

    """


    success: bool = False

    status_message: str = ""

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_4db6d7df20cdcdc05430945e76fa58e1eab1285c15f31d26fad76e2c1b10299c"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::srv::dds_::SetCameraInfo_Response_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['success'] = self.success



        result['status_message'] = self.status_message


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'success' in data:

            kwargs['success'] = data['success']


        if 'status_message' in data:

            kwargs['status_message'] = data['status_message']


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




class SetCameraInfo:
    """
    Service type for sensor_msgs/SetCameraInfo.
    
    Request: SetCameraInfo_Request
    Response: SetCameraInfo_Response
    """
    Request = SetCameraInfo_Request
    Response = SetCameraInfo_Response
    
    # ROS2 Service Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_a10cca5d33dc637c8d49db50ab288701a3592bb9cd854f2f16a0659613b68984"

