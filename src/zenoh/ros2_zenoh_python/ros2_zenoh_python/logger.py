"""
Logger Module

Simple logger that matches rclpy logger interface.
"""

from typing import Any


class Logger:
    """Simple logger that matches rclpy logger interface."""
    
    def __init__(self, node_name: str):
        """Initialize logger."""
        self.node_name = node_name
    
    def debug(self, msg: str, *args, **kwargs):
        """Log debug message."""
        print(f"[DEBUG] [{self.node_name}] {msg % args if args else msg}")
    
    def info(self, msg: str, *args, **kwargs):
        """Log info message."""
        print(f"[INFO] [{self.node_name}] {msg % args if args else msg}")
    
    def warn(self, msg: str, *args, **kwargs):
        """Log warning message."""
        print(f"[WARN] [{self.node_name}] {msg % args if args else msg}")
    
    def error(self, msg: str, *args, **kwargs):
        """Log error message."""
        print(f"[ERROR] [{self.node_name}] {msg % args if args else msg}")
    
    def fatal(self, msg: str, *args, **kwargs):
        """Log fatal message."""
        print(f"[FATAL] [{self.node_name}] {msg % args if args else msg}")
    
    # Alias for warn
    warning = warn
