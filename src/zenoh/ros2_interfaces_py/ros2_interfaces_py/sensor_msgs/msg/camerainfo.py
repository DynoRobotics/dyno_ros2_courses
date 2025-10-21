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


# Import types from same package

from .regionofinterest import RegionOfInterest




# Import package.msg modules

import ros2_interfaces_py.std_msgs.msg



if TYPE_CHECKING:
    pass


@dataclass
class CameraInfo(IdlStruct, typename="sensor_msgs/CameraInfo"):

    """sensor_msgs/CameraInfo message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_5d4696861b789afe70a51c78147734418df5eaa53064c8aaec5669be41a6b8b7


    DDS type name: sensor_msgs::msg::dds_::CameraInfo_

    """

    header: 'ros2_interfaces_py.std_msgs.msg.Header'

    height: uint32

    width: uint32

    distortion_model: str

    d: List[float64]

    k: List[float64]

    r: List[float64]

    p: List[float64]

    binning_x: uint32

    binning_y: uint32

    roi: RegionOfInterest

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_5d4696861b789afe70a51c78147734418df5eaa53064c8aaec5669be41a6b8b7"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::CameraInfo_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['height'] = self.height



        result['width'] = self.width



        result['distortion_model'] = self.distortion_model



        result['d'] = self.d



        result['k'] = self.k



        result['r'] = self.r



        result['p'] = self.p



        result['binning_x'] = self.binning_x



        result['binning_y'] = self.binning_y



        result['roi'] = self.roi.to_dict()


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'header' in data:

            kwargs['header'] = Header.from_dict(data['header'])


        if 'height' in data:

            kwargs['height'] = data['height']


        if 'width' in data:

            kwargs['width'] = data['width']


        if 'distortion_model' in data:

            kwargs['distortion_model'] = data['distortion_model']


        if 'd' in data:

            kwargs['d'] = data['d']


        if 'k' in data:

            kwargs['k'] = data['k']


        if 'r' in data:

            kwargs['r'] = data['r']


        if 'p' in data:

            kwargs['p'] = data['p']


        if 'binning_x' in data:

            kwargs['binning_x'] = data['binning_x']


        if 'binning_y' in data:

            kwargs['binning_y'] = data['binning_y']


        if 'roi' in data:

            kwargs['roi'] = RegionOfInterest.from_dict(data['roi'])


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

