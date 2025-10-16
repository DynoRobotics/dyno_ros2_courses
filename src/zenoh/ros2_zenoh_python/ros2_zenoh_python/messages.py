"""
Core Message Types

Basic message types that are used across the package.
"""

from dataclasses import dataclass


@dataclass
class Time:
    """Simplified time message."""
    sec: int = 0
    nanosec: int = 0


@dataclass
class Log:
    """Simplified log message."""
    stamp: Time = None
    level: int = 0
    name: str = ""
    msg: str = ""
    file: str = ""
    function: str = ""
    line: int = 0
    
    def __post_init__(self):
        if self.stamp is None:
            self.stamp = Time()


# Export all message types
__all__ = [
    'Time', 'Log'
]
