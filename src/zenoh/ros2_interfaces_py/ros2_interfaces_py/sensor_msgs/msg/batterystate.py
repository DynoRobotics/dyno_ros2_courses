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
class BatteryState(IdlStruct, typename="sensor_msgs/BatteryState"):

    """sensor_msgs/BatteryState message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_01467e6e664de7d8e9b8b6dcac2ee14d44a1693f23d4c330403d55cda7925e98


    DDS type name: sensor_msgs::msg::dds_::BatteryState_

    """

    header: 'ros2_interfaces_py.std_msgs.msg.Header'

    voltage: float32

    temperature: float32

    current: float32

    charge: float32

    capacity: float32

    design_capacity: float32

    percentage: float32

    power_supply_status: uint8

    power_supply_health: uint8

    power_supply_technology: uint8

    present: bool

    cell_voltage: List[float32]

    cell_temperature: List[float32]

    location: str

    serial_number: str

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_01467e6e664de7d8e9b8b6dcac2ee14d44a1693f23d4c330403d55cda7925e98"


    # DDS Type Name
    DDS_TYPE_NAME = "sensor_msgs::msg::dds_::BatteryState_"


    def to_dict(self) -> dict:
        """Convert to dictionary, recursively converting nested messages."""
        result = {}


        result['header'] = self.header.to_dict()



        result['voltage'] = self.voltage



        result['temperature'] = self.temperature



        result['current'] = self.current



        result['charge'] = self.charge



        result['capacity'] = self.capacity



        result['design_capacity'] = self.design_capacity



        result['percentage'] = self.percentage



        result['power_supply_status'] = self.power_supply_status



        result['power_supply_health'] = self.power_supply_health



        result['power_supply_technology'] = self.power_supply_technology



        result['present'] = self.present



        result['cell_voltage'] = self.cell_voltage



        result['cell_temperature'] = self.cell_temperature



        result['location'] = self.location



        result['serial_number'] = self.serial_number


        return result
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary, recursively creating nested messages."""
        kwargs = {}

        if 'header' in data:

            kwargs['header'] = Header.from_dict(data['header'])


        if 'voltage' in data:

            kwargs['voltage'] = data['voltage']


        if 'temperature' in data:

            kwargs['temperature'] = data['temperature']


        if 'current' in data:

            kwargs['current'] = data['current']


        if 'charge' in data:

            kwargs['charge'] = data['charge']


        if 'capacity' in data:

            kwargs['capacity'] = data['capacity']


        if 'design_capacity' in data:

            kwargs['design_capacity'] = data['design_capacity']


        if 'percentage' in data:

            kwargs['percentage'] = data['percentage']


        if 'power_supply_status' in data:

            kwargs['power_supply_status'] = data['power_supply_status']


        if 'power_supply_health' in data:

            kwargs['power_supply_health'] = data['power_supply_health']


        if 'power_supply_technology' in data:

            kwargs['power_supply_technology'] = data['power_supply_technology']


        if 'present' in data:

            kwargs['present'] = data['present']


        if 'cell_voltage' in data:

            kwargs['cell_voltage'] = data['cell_voltage']


        if 'cell_temperature' in data:

            kwargs['cell_temperature'] = data['cell_temperature']


        if 'location' in data:

            kwargs['location'] = data['location']


        if 'serial_number' in data:

            kwargs['serial_number'] = data['serial_number']


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

