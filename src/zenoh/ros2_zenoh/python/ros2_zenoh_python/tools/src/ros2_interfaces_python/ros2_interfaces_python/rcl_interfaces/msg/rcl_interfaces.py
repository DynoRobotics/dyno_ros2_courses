"""Simplified rcl_interfaces types"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

# Import dependencies
from ...builtin_interfaces.msg.builtin_interfaces import Time

@dataclass
class FloatingPointRange:
    """Simplified rcl_interfaces.msg.FloatingPointRange message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    from_value: float = 0.0
    to_value: float = 0.0
    step: float = 0.0

@dataclass
class LoggerLevel:
    """Simplified rcl_interfaces.msg.LoggerLevel message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    name: str = ""
    level: int = 0

@dataclass
class SetLoggerLevelsResult:
    """Simplified rcl_interfaces.msg.SetLoggerLevelsResult message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    successful: bool = False
    reason: str = ""

@dataclass
class SetParametersResult:
    """Simplified rcl_interfaces.msg.SetParametersResult message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    successful: bool = False
    reason: str = ""

@dataclass
class ListParametersResult:
    """Simplified rcl_interfaces.msg.ListParametersResult message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    names: List[str] = field(default_factory=list)
    prefixes: List[str] = field(default_factory=list)

@dataclass
class ParameterEventDescriptors:
    """Simplified rcl_interfaces.msg.ParameterEventDescriptors message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    new_parameters: List['ParameterDescriptor'] = field(default_factory=list)
    changed_parameters: List['ParameterDescriptor'] = field(default_factory=list)
    deleted_parameters: List['ParameterDescriptor'] = field(default_factory=list)
    
    def __post_init__(self):
        if self.new_parameters is None:
            self.new_parameters = ParameterDescriptor()
        if self.changed_parameters is None:
            self.changed_parameters = ParameterDescriptor()
        if self.deleted_parameters is None:
            self.deleted_parameters = ParameterDescriptor()


@dataclass
class ParameterType:
    """Simplified rcl_interfaces.msg.ParameterType message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    pass

@dataclass
class ParameterValue:
    """Simplified rcl_interfaces.msg.ParameterValue message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    type: int = 0
    bool_value: bool = False
    integer_value: int = 0
    double_value: float = 0.0
    string_value: str = ""
    byte_array_value: List[int] = field(default_factory=list)
    bool_array_value: List[bool] = field(default_factory=list)
    integer_array_value: List[int] = field(default_factory=list)
    double_array_value: List[float] = field(default_factory=list)
    string_array_value: List[str] = field(default_factory=list)

@dataclass
class ParameterDescriptor:
    """Simplified rcl_interfaces.msg.ParameterDescriptor message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    name: str = ""
    type: int = 0
    description: str = ""
    additional_constraints: str = ""
    read_only: bool = False
    dynamic_typing: bool = False
    floating_point_range: List['FloatingPointRange'] = field(default_factory=list)
    integer_range: List['IntegerRange'] = field(default_factory=list)
    
    def __post_init__(self):
        if self.floating_point_range is None:
            self.floating_point_range = FloatingPointRange()
        if self.integer_range is None:
            self.integer_range = IntegerRange()


@dataclass
class ParameterEvent:
    """Simplified rcl_interfaces.msg.ParameterEvent message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    stamp: Optional['Time'] = None
    node: str = ""
    new_parameters: List['Parameter'] = field(default_factory=list)
    changed_parameters: List['Parameter'] = field(default_factory=list)
    deleted_parameters: List['Parameter'] = field(default_factory=list)
    
    def __post_init__(self):
        if self.stamp is None:
            self.stamp = Time()
        if self.new_parameters is None:
            self.new_parameters = Parameter()
        if self.changed_parameters is None:
            self.changed_parameters = Parameter()
        if self.deleted_parameters is None:
            self.deleted_parameters = Parameter()


@dataclass
class IntegerRange:
    """Simplified rcl_interfaces.msg.IntegerRange message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    from_value: int = 0
    to_value: int = 0
    step: int = 0

@dataclass
class Parameter:
    """Simplified rcl_interfaces.msg.Parameter message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    name: str = ""
    value: Optional['ParameterValue'] = None
    
    def __post_init__(self):
        if self.value is None:
            self.value = ParameterValue()


@dataclass
class Log:
    """Simplified rcl_interfaces.msg.Log message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
    stamp: Optional['Time'] = None
    level: int = 0
    name: str = ""
    msg: str = ""
    file: str = ""
    function: str = ""
    line: int = 0
    
    def __post_init__(self):
        if self.stamp is None:
            self.stamp = Time()


__all__ = ['FloatingPointRange', 'LoggerLevel', 'SetLoggerLevelsResult', 'SetParametersResult', 'ListParametersResult', 'ParameterEventDescriptors', 'ParameterType', 'ParameterValue', 'ParameterDescriptor', 'ParameterEvent', 'IntegerRange', 'Parameter', 'Log']