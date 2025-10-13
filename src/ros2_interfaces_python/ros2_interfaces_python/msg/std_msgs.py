"""
Simplified std_msgs types
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any


@dataclass
class Header:
    """Simplified std_msgs.msg.Header message."""
    stamp: Optional[Dict[str, int]] = None
    frame_id: str = ""
    
    def __post_init__(self):
        if self.stamp is None:
            self.stamp = {"sec": 0, "nanosec": 0}


@dataclass
class Empty:
    """Simplified std_msgs.msg.Empty message."""
    pass


@dataclass
class Bool:
    """Simplified std_msgs.msg.Bool message."""
    data: bool = False


@dataclass
class String:
    """Simplified std_msgs.msg.String message."""
    data: str = ""


@dataclass
class Int8:
    """Simplified std_msgs.msg.Int8 message."""
    data: int = 0


@dataclass
class Int16:
    """Simplified std_msgs.msg.Int16 message."""
    data: int = 0


@dataclass
class Int32:
    """Simplified std_msgs.msg.Int32 message."""
    data: int = 0


@dataclass
class Int64:
    """Simplified std_msgs.msg.Int64 message."""
    data: int = 0


@dataclass
class UInt8:
    """Simplified std_msgs.msg.UInt8 message."""
    data: int = 0


@dataclass
class UInt16:
    """Simplified std_msgs.msg.UInt16 message."""
    data: int = 0


@dataclass
class UInt32:
    """Simplified std_msgs.msg.UInt32 message."""
    data: int = 0


@dataclass
class UInt64:
    """Simplified std_msgs.msg.UInt64 message."""
    data: int = 0


@dataclass
class Float32:
    """Simplified std_msgs.msg.Float32 message."""
    data: float = 0.0


@dataclass
class Float64:
    """Simplified std_msgs.msg.Float64 message."""
    data: float = 0.0


@dataclass
class Byte:
    """Simplified std_msgs.msg.Byte message."""
    data: int = 0


@dataclass
class Char:
    """Simplified std_msgs.msg.Char message."""
    data: str = ""


@dataclass
class ColorRGBA:
    """Simplified std_msgs.msg.ColorRGBA message."""
    r: float = 0.0
    g: float = 0.0
    b: float = 0.0
    a: float = 0.0


# MultiArray types
@dataclass
class MultiArrayDimension:
    """Simplified std_msgs.msg.MultiArrayDimension message."""
    label: str = ""
    size: int = 0
    stride: int = 0


@dataclass
class MultiArrayLayout:
    """Simplified std_msgs.msg.MultiArrayLayout message."""
    dim: List[MultiArrayDimension] = field(default_factory=list)
    data_offset: int = 0


@dataclass
class Int8MultiArray:
    """Simplified std_msgs.msg.Int8MultiArray message."""
    layout: MultiArrayLayout = None
    data: List[int] = field(default_factory=list)
    
    def __post_init__(self):
        if self.layout is None:
            self.layout = MultiArrayLayout()


@dataclass
class UInt8MultiArray:
    """Simplified std_msgs.msg.UInt8MultiArray message."""
    layout: MultiArrayLayout = None
    data: List[int] = field(default_factory=list)
    
    def __post_init__(self):
        if self.layout is None:
            self.layout = MultiArrayLayout()


@dataclass
class Float32MultiArray:
    """Simplified std_msgs.msg.Float32MultiArray message."""
    layout: MultiArrayLayout = None
    data: List[float] = field(default_factory=list)
    
    def __post_init__(self):
        if self.layout is None:
            self.layout = MultiArrayLayout()


# Export all types
__all__ = [
    'Header', 'Empty', 'Bool', 'String', 'Int8', 'Int16', 'Int32', 'Int64',
    'UInt8', 'UInt16', 'UInt32', 'UInt64', 'Float32', 'Float64', 'Byte', 'Char',
    'ColorRGBA', 'MultiArrayDimension', 'MultiArrayLayout', 'Int8MultiArray',
    'UInt8MultiArray', 'Float32MultiArray'
]
