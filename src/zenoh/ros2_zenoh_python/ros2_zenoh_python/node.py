"""
Node Module

ROS 2-compatible node using Zenoh as the transport layer.
Async-first design with full asyncio support.
"""

import asyncio
import logging
import zenoh
from typing import Optional, Dict, Any, Type, Callable, Union
from .liveliness_manager import LivelinessManager

logger = logging.getLogger(__name__)


class Node:
    """ROS 2-compatible node using Zenoh transport."""
    
    def __init__(self, node_name: str, 
                 zenoh_session: Optional[zenoh.Session] = None,
                 namespace: str = "",
                 zenoh_config: Optional[zenoh.Config] = None,
                 enable_rosout: bool = True):
        """
        Initialize a ROS 2-compatible node.
        
        Args:
            node_name: ROS 2 node name
            zenoh_session: Shared Zenoh session (preferred). If provided, zenoh_config is ignored.
            namespace: ROS 2 node namespace
            zenoh_config: Zenoh configuration (only used if zenoh_session is None).
                         If not provided, defaults to client mode connecting to localhost:7447
                         for ROS2 interoperability with rmw_zenoh_cpp.
            enable_rosout: If True, automatically setup logging to publish to /rosout
        """
        self.node_name = node_name
        self.namespace = namespace
        
        # Use provided session or create a new one
        if zenoh_session is not None:
            self.session = zenoh_session
            self._owns_session = False
        else:
            # Default config: client mode connecting to ROS2 Zenoh router
            if zenoh_config is None:
                config = zenoh.Config()
                config.insert_json5("mode", '"client"')
                config.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
            else:
                config = zenoh_config
            self.session = zenoh.open(config)
            self._owns_session = True
        
        # Initialize liveliness manager
        self.liveliness_manager = LivelinessManager(self.session)
        
        # Track publishers and subscribers
        self.publishers: Dict[str, Any] = {}
        self.subscribers: Dict[str, Any] = {}
        
        # Shutdown event for async lifecycle
        self._shutdown_event: Optional[asyncio.Event] = None
        self._signal_handlers_installed = False
        
        # Automatically setup logging to /rosout if enabled
        if enable_rosout:
            from .logger import setup_logging
            setup_logging(self, level=logging.INFO)
        
        logger.debug(f"Node '{self.node_name}' created (owns_session={self._owns_session})")
    
    def create_publisher(self, msg_type: Type, topic: str, qos_profile: Optional[dict] = None) -> 'Publisher':
        """
        Create a publisher for this node.
        
        Args:
            msg_type: Message type class
            topic: Topic name
            qos_profile: QoS profile settings
            
        Returns:
            Publisher instance
        """
        from .publisher import Publisher
        pub = Publisher(msg_type, topic, node=self, qos_profile=qos_profile)
        self.publishers[topic] = pub
        return pub
    
    def create_subscription(self, msg_type: Type, topic: str, callback: Callable, 
                           qos_profile: Optional[dict] = None) -> 'Subscriber':
        """
        Create a subscription for this node.
        
        Args:
            msg_type: Message type class
            topic: Topic name
            callback: Callback function
            qos_profile: QoS profile settings
            
        Returns:
            Subscriber instance
        """
        from .subscriber import Subscriber
        sub = Subscriber(msg_type, topic, callback, node=self, qos_profile=qos_profile)
        self.subscribers[topic] = sub
        return sub
    
    def destroy_publisher(self, publisher: 'Publisher'):
        """Destroy a publisher."""
        if publisher.topic in self.publishers:
            publisher.destroy()
            del self.publishers[publisher.topic]
    
    def destroy_subscription(self, subscription: 'Subscriber'):
        """Destroy a subscription."""
        if subscription.topic in self.subscribers:
            subscription.destroy()
            del self.subscribers[subscription.topic]
    
    def get_logger(self) -> 'Logger':
        """Get a logger for this node."""
        from .logger import Logger
        return Logger(self.node_name)
    
    def destroy_node(self):
        """Destroy the node and clean up all resources."""
        # Close all publishers
        for pub in list(self.publishers.values()):
            pub.destroy()
        self.publishers.clear()
        
        # Close all subscribers
        for sub in list(self.subscribers.values()):
            sub.destroy()
        self.subscribers.clear()
        
        # Only close Zenoh session if we own it
        if hasattr(self, 'session') and self._owns_session:
            self.session.close()
            logger.debug(f"Node '{self.node_name}' destroyed (closed session)")
        else:
            logger.debug(f"Node '{self.node_name}' destroyed (session shared)")
    
    # Sync context manager (for backward compatibility)
    def __enter__(self):
        """Synchronous context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Synchronous context manager exit."""
        self.destroy_node()
    
    # Async context manager (preferred)
    async def __aenter__(self):
        """Async context manager entry with automatic signal handling."""
        # Setup signal handlers for clean shutdown
        if not self._signal_handlers_installed:
            self._shutdown_event = asyncio.Event()
            loop = asyncio.get_running_loop()
            
            def shutdown():
                logger.info(f"Shutdown signal received for node '{self.node_name}'")
                if self._shutdown_event:
                    self._shutdown_event.set()
            
            # Install signal handlers
            import signal
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.add_signal_handler(sig, shutdown)
            
            self._signal_handlers_installed = True
            logger.debug(f"Installed signal handlers for node '{self.node_name}'")
        
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.adestroy_node()
    
    async def adestroy_node(self):
        """Async version of destroy_node."""
        # Close all publishers
        for pub in list(self.publishers.values()):
            await pub.adestroy()
        self.publishers.clear()
        
        # Close all subscribers
        for sub in list(self.subscribers.values()):
            await sub.adestroy()
        self.subscribers.clear()
        
        # Only close Zenoh session if we own it
        if hasattr(self, 'session') and self._owns_session:
            self.session.close()
            logger.debug(f"Node '{self.node_name}' destroyed (async, closed session)")
        else:
            logger.debug(f"Node '{self.node_name}' destroyed (async, session shared)")
    
    async def spin(self):
        """
        Keep the node alive and process callbacks asynchronously.
        Automatically handles shutdown signals (SIGINT/SIGTERM).
        This is the async equivalent of spinning in rclpy.
        """
        if self._shutdown_event is None:
            # Not in async context manager, create shutdown event
            self._shutdown_event = asyncio.Event()
        
        try:
            # Wait for shutdown signal
            await self._shutdown_event.wait()
            logger.debug(f"Node '{self.node_name}' spin completed")
        except asyncio.CancelledError:
            logger.debug(f"Node '{self.node_name}' spin cancelled")
            raise
    
    @property
    def shutdown_requested(self) -> bool:
        """Check if shutdown has been requested."""
        return self._shutdown_event is not None and self._shutdown_event.is_set()
    
    def request_shutdown(self):
        """Request shutdown (can be called from sync code)."""
        if self._shutdown_event:
            self._shutdown_event.set()
    
    def __del__(self):
        """Destructor: ensure cleanup."""
        try:
            self.destroy_node()
        except:
            pass  # Ignore errors during cleanup
