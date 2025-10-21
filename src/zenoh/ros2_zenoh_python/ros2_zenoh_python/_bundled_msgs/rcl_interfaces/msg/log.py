from dataclasses import dataclass
from typing import List, Optional, TYPE_CHECKING

try:
    from pycdr2 import IdlStruct
    from pycdr2.types import int8, uint8, int16, uint16, int32, uint32, int64, uint64, float32, float64
    PYCDR2_AVAILABLE = True
except ImportError:
    PYCDR2_AVAILABLE = False
    IdlStruct = object  # Use object as base class when pycdr2 is not available
    # Define dummy types for type hints
    int8 = uint8 = int16 = uint16 = int32 = uint32 = int64 = uint64 = int
    float32 = float64 = float



# Import package.msg modules for cross-package types
# Use bundled messages with relative imports (3 levels up: msg -> rcl_interfaces -> _bundled_msgs)
from ...builtin_interfaces import msg as builtin_interfaces_msg



if TYPE_CHECKING:
    # Import needed for type hints but avoid circular imports
    pass

@dataclass
class Log(IdlStruct, typename="rcl_interfaces/Log"):
    """rcl_interfaces/Log message.
    
    Supports CDR serialization via pycdr2 when available.
    Type annotations use direct types (not strings) for pycdr2 compatibility.
    
    ROS 2 type hash: RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac
    DDS type name: rcl_interfaces::msg::dds_::Log_
    """

    stamp: builtin_interfaces_msg.Time

    level: uint8

    name: str

    msg: str

    file: str

    function: str

    line: uint32

    
    # Class constants (defined after fields for dataclass compatibility)
    TYPE_HASH = "RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac"
    DDS_TYPE_NAME = "rcl_interfaces::msg::dds_::Log_"

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {

            'stamp': self.stamp,

            'level': self.level,

            'name': self.name,

            'msg': self.msg,

            'file': self.file,

            'function': self.function,

            'line': self.line,

        }
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary."""
        return cls(**data)
    
    def serialize(self) -> bytes:
        """
        Serialize to CDR format.
        
        Returns:
            bytes: Serialized CDR data
            
        Raises:
            RuntimeError: If pycdr2 is not available
        """
        if not PYCDR2_AVAILABLE:
            raise RuntimeError("pycdr2 is required for serialization. Install with: pip install pycdr2")
        return IdlStruct.serialize(self)
    
    @classmethod
    def deserialize(cls, data: bytes):
        """
        Deserialize from CDR format.
        
        Args:
            data: Serialized CDR bytes
            
        Returns:
            Deserialized message instance
            
        Raises:
            RuntimeError: If pycdr2 is not available
        """
        if not PYCDR2_AVAILABLE:
            raise RuntimeError("pycdr2 is required for deserialization. Install with: pip install pycdr2")
        # Use the classmethod from IdlStruct properly
        return super(Log, cls).deserialize(data)