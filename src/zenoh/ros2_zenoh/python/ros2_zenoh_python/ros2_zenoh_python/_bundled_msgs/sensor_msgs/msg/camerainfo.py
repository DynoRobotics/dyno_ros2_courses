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

from .regionofinterest import RegionOfInterest




# Import other package modules (not individual classes to avoid circular imports)

from ...std_msgs import msg as std_msgs_msg



if TYPE_CHECKING:
    pass




@dataclass
class CameraInfo(IdlStruct, typename="sensor_msgs/CameraInfo"):

    """sensor_msgs/CameraInfo message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275


    DDS type name: sensor_msgs::msg::dds_::CameraInfo_

    """


    header: std_msgs_msg.Header = field(default_factory=lambda: std_msgs_msg.Header())

    height: uint32 = 0

    width: uint32 = 0

    distortion_model: str = ""

    d: List[float64] = field(default_factory=list)

    k: array[float64, 9] = field(default_factory=lambda: [0.0] * 9)

    r: array[float64, 9] = field(default_factory=lambda: [0.0] * 9)

    p: array[float64, 12] = field(default_factory=lambda: [0.0] * 12)

    binning_x: uint32 = 0

    binning_y: uint32 = 0

    roi: RegionOfInterest = field(default_factory=RegionOfInterest)

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275"


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

            
            kwargs['header'] = std_msgs_msg.Header.from_dict(data['header'])
            


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

