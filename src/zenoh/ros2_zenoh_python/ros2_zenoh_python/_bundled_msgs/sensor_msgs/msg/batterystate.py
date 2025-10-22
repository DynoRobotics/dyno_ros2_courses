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



# Import other package modules (not individual classes to avoid circular imports)

from ...std_msgs import msg as std_msgs_msg



if TYPE_CHECKING:
    pass




@dataclass
class BatteryState(IdlStruct, typename="sensor_msgs/BatteryState"):

    """sensor_msgs/BatteryState message.
    
    Encoding: CDR

    ROS 2 type hash: RIHS01_4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741


    DDS type name: sensor_msgs::msg::dds_::BatteryState_

    """

    # Constants

    POWER_SUPPLY_STATUS_UNKNOWN = 0

    POWER_SUPPLY_STATUS_CHARGING = 1

    POWER_SUPPLY_STATUS_DISCHARGING = 2

    POWER_SUPPLY_STATUS_NOT_CHARGING = 3

    POWER_SUPPLY_STATUS_FULL = 4

    POWER_SUPPLY_HEALTH_UNKNOWN = 0

    POWER_SUPPLY_HEALTH_GOOD = 1

    POWER_SUPPLY_HEALTH_OVERHEAT = 2

    POWER_SUPPLY_HEALTH_DEAD = 3

    POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4

    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5

    POWER_SUPPLY_HEALTH_COLD = 6

    POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7

    POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

    POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0 # Unknown battery technology

    POWER_SUPPLY_TECHNOLOGY_NIMH = 1    # Nickel-Metal Hydride battery

    POWER_SUPPLY_TECHNOLOGY_LION = 2    # Lithium-ion battery

    POWER_SUPPLY_TECHNOLOGY_LIPO = 3    # Lithium Polymer battery

    POWER_SUPPLY_TECHNOLOGY_LIFE = 4    # Lithium Iron Phosphate battery

    POWER_SUPPLY_TECHNOLOGY_NICD = 5    # Nickel-Cadmium battery

    POWER_SUPPLY_TECHNOLOGY_LIMN = 6    # Lithium Manganese Dioxide battery

    POWER_SUPPLY_TECHNOLOGY_TERNARY = 7 # Ternary Lithium battery

    POWER_SUPPLY_TECHNOLOGY_VRLA = 8    # Valve Regulated Lead-Acid battery



    header: std_msgs_msg.Header = field(default_factory=lambda: std_msgs_msg.Header())

    voltage: float32 = 0.0

    temperature: float32 = 0.0

    current: float32 = 0.0

    charge: float32 = 0.0

    capacity: float32 = 0.0

    design_capacity: float32 = 0.0

    percentage: float32 = 0.0

    power_supply_status: uint8 = 0

    power_supply_health: uint8 = 0

    power_supply_technology: uint8 = 0

    present: bool = False

    cell_voltage: List[float32] = field(default_factory=list)

    cell_temperature: List[float32] = field(default_factory=list)

    location: str = ""

    serial_number: str = ""

    

    # ROS2 Type Hash (RIHS01)
    TYPE_HASH = "RIHS01_4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741"


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

            
            kwargs['header'] = std_msgs_msg.Header.from_dict(data['header'])
            


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

