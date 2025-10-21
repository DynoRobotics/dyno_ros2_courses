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



if TYPE_CHECKING:
    # Import needed for type hints but avoid circular imports
    pass

@dataclass
class Vector3(IdlStruct, typename="geometry_msgs/Vector3"):
    """geometry_msgs/Vector3 message.
    
    Supports CDR serialization via pycdr2 when available.
    Type annotations use direct types (not strings) for pycdr2 compatibility.
    
    ROS 2 type hash: RIHS01_b67f3a45d128e9126848bb9810321c719d8833741c7c9cd48bcbc83ab974cd48
    DDS type name: geometry_msgs::msg::dds_::Vector3_
    """

    x: float64

    y: float64

    z: float64

    
    # Class constants (defined after fields for dataclass compatibility)
    TYPE_HASH = "RIHS01_b67f3a45d128e9126848bb9810321c719d8833741c7c9cd48bcbc83ab974cd48"
    DDS_TYPE_NAME = "geometry_msgs::msg::dds_::Vector3_"

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {

            'x': self.x,

            'y': self.y,

            'z': self.z,

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
        return super(Vector3, cls).deserialize(data)