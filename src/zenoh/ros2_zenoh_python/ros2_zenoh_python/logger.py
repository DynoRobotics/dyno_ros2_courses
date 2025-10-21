"""
Logger Module

Provides a RosoutHandler for Python's standard logging module.

Instead of ROS2's get_logger() pattern, use Python's standard:
    import logging
    logger = logging.getLogger(__name__)
    logger.info("Hello")

To enable /rosout publishing, add the handler:
    from ros2_zenoh_python.logger import RosoutHandler
    logging.root.addHandler(RosoutHandler(node))
"""

import logging
import sys
import time
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .node import Node


class RosoutHandler(logging.Handler):
    """
    Custom logging handler that publishes to /rosout topic.
    
    This allows all logs to be monitored centrally via:
    - ros2 topic echo /rosout
    - rqt_console
    - rosbag recordings
    """
    
    # ROS2 log level mapping (from rcl_interfaces/msg/Log)
    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50
    
    def __init__(self, node: 'Node'):
        """Initialize rosout handler."""
        super().__init__()
        self.node = node
        self.rosout_publisher = None
        
        # Try to import Log message - first from bundled, then from full package
        try:
            # Try bundled messages first (always available)
            try:
                from ._bundled_msgs.rcl_interfaces.msg.log import Log
                from ._bundled_msgs.builtin_interfaces.msg.time import Time
            except ImportError:
                # Fall back to full ros2_interfaces_py package if installed
                from ros2_interfaces_py.rcl_interfaces.msg.log import Log
                from ros2_interfaces_py.builtin_interfaces.msg.time import Time
            
            self.Log = Log
            self.Time = Time
            
            # Create rosout publisher with ROS2-compatible QoS
            # /rosout uses TRANSIENT_LOCAL durability so late joiners can see old logs
            rosout_qos = {
                'reliability': 'reliable',  # RELIABLE
                'durability': 'transient_local',  # TRANSIENT_LOCAL (important for /rosout!)
                'history': 'keep_last',
                'depth': 1000  # Keep last 1000 log messages
            }
            self.rosout_publisher = node.create_publisher(Log, '/rosout', qos_profile=rosout_qos)
            
        except ImportError:
            # If Log message not available, just print to console
            pass
    
    def emit(self, record: logging.LogRecord):
        """Emit a log record to /rosout topic."""
        try:
            # Publish to /rosout (console output is handled by separate StreamHandler)
            if self.rosout_publisher and self.Log and self.Time:
                # Map Python log levels to ROS2 log levels
                level_map = {
                    logging.DEBUG: self.DEBUG,
                    logging.INFO: self.INFO,
                    logging.WARNING: self.WARN,
                    logging.ERROR: self.ERROR,
                    logging.CRITICAL: self.FATAL,
                }
                
                # Get timestamp
                now = time.time()
                stamp = self.Time(
                    sec=int(now),
                    nanosec=int((now % 1) * 1e9)
                )
                
                # Create Log message
                log_msg = self.Log(
                    stamp=stamp,
                    level=level_map.get(record.levelno, self.INFO),
                    name=record.name,
                    msg=record.getMessage(),
                    file=record.pathname,
                    function=record.funcName,
                    line=record.lineno
                )
                
                # Publish to /rosout
                self.rosout_publisher.publish(log_msg)
                
        except Exception as e:
            # If publishing fails, at least print the error
            print(f"Failed to publish to /rosout: {e}", file=sys.stderr)


def setup_logging(node: 'Node', level: int = logging.INFO, publish_to_rosout: bool = True):
    """
    Setup standard Python logging with optional /rosout publishing.
    
    This is the recommended way to configure logging for your ROS2 Zenoh nodes.
    
    Args:
        node: Node instance (for publishing to /rosout)
        level: Logging level (e.g., logging.INFO, logging.DEBUG)
        publish_to_rosout: If True, add handler that publishes to /rosout topic
    
    Example:
        from ros2_zenoh_python import Node
        from ros2_zenoh_python.logger import setup_logging
        import logging
        
        node = Node('my_node')
        setup_logging(node, level=logging.DEBUG)
        
        logger = logging.getLogger(__name__)
        logger.info("This goes to console AND /rosout!")
    """
    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    
    # Add console handler if not already present (check for non-RosoutHandler StreamHandlers)
    has_console_handler = any(
        isinstance(h, logging.StreamHandler) and not isinstance(h, RosoutHandler)
        for h in root_logger.handlers
    )
    if not has_console_handler:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(logging.Formatter('[%(levelname)s] [%(name)s] %(message)s'))
        root_logger.addHandler(console_handler)
    
    # Add rosout handler if requested and not already present
    if publish_to_rosout:
        has_rosout_handler = any(isinstance(h, RosoutHandler) for h in root_logger.handlers)
        if not has_rosout_handler:
            try:
                rosout_handler = RosoutHandler(node)
                rosout_handler.setLevel(logging.DEBUG)  # Pass everything to /rosout
                root_logger.addHandler(rosout_handler)
            except Exception as e:
                logging.warning(f"Failed to setup /rosout publishing: {e}")
