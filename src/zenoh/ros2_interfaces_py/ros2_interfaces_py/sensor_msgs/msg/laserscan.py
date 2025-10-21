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

import ros2_interfaces_py.std_msgs.msg



if TYPE_CHECKING:
    pass


@dataclass
class LaserScan(IdlStruct, typename="sensor_msgs/LaserScan"):

    """sensor_msgs/LaserScan message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_194898a7d0517a052a5bac6c36a36307084f4437d55fd47aaa24b23b44b8b641


    DDS type name: sensor_msgs::msg::dds_::LaserScan_

    """

    header: 'ros2_interfaces_py.std_msgs.msg.Header'

    angle_min: float32

    angle_max: float32

    angle_increment: float32

    time_increment: float32

    scan_time: float32

    range_min: float32

    range_max: float32

    ranges: List[float32]

    intensities: List[float32]

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_194898a7d0517a052a5bac6c36a36307084f4437d55fd47aaa24b23b44b8b641"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::LaserScan_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['angle_min'] = self.angle_min



        result['angle_max'] = self.angle_max



        result['angle_increment'] = self.angle_increment



        result['time_increment'] = self.time_increment



        result['scan_time'] = self.scan_time



        result['range_min'] = self.range_min



        result['range_max'] = self.range_max



        result['ranges'] = self.ranges



        result['intensities'] = self.intensities


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'header' in data:

            kwargs['header'] = Header.from_dict(data['header'])


        if 'angle_min' in data:

            kwargs['angle_min'] = data['angle_min']


        if 'angle_max' in data:

            kwargs['angle_max'] = data['angle_max']


        if 'angle_increment' in data:

            kwargs['angle_increment'] = data['angle_increment']


        if 'time_increment' in data:

            kwargs['time_increment'] = data['time_increment']


        if 'scan_time' in data:

            kwargs['scan_time'] = data['scan_time']


        if 'range_min' in data:

            kwargs['range_min'] = data['range_min']


        if 'range_max' in data:

            kwargs['range_max'] = data['range_max']


        if 'ranges' in data:

            kwargs['ranges'] = data['ranges']


        if 'intensities' in data:

            kwargs['intensities'] = data['intensities']


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

