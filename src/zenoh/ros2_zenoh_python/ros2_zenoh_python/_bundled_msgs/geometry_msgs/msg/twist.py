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


# Import types from same package (relative imports)

from .vector3 import Vector3




if TYPE_CHECKING:
    # Import needed for type hints but avoid circular imports
    pass

@dataclass
class Twist(IdlStruct, typename="geometry_msgs/Twist"):
    """geometry_msgs/Twist message.
    
    Supports CDR serialization via pycdr2 when available.
    Type annotations use direct types (not strings) for pycdr2 compatibility.
    
    ROS 2 type hash: RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a
    DDS type name: geometry_msgs::msg::dds_::Twist_
    """

    linear: Vector3

    angular: Vector3

    
    # Class constants (defined after fields for dataclass compatibility)
    TYPE_HASH = "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a"
    DDS_TYPE_NAME = "geometry_msgs::msg::dds_::Twist_"

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {

            'linear': self.linear,

            'angular': self.angular,

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
        return super(Twist, cls).deserialize(data)