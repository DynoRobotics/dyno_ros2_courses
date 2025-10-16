"""
Node Module

ROS 2-compatible node using Zenoh as the transport layer.
Designed to match rclpy interface as closely as possible.
"""

import zenoh
from typing import Optional, Dict, Any, Type, Callable
from .liveliness_manager import LivelinessManager


class Node:
    """ROS 2-compatible node using Zenoh transport."""
    
    def __init__(self, node_name: str, namespace: str = "", 
                 zenoh_config: Optional[zenoh.Config] = None):
        """
        Initialize a ROS 2-compatible node.
        
        Args:
            node_name: ROS 2 node name
            namespace: ROS 2 node namespace
            zenoh_config: Zenoh configuration
        """
        self.node_name = node_name
        self.namespace = namespace
        
        # Initialize Zenoh session (shared across all publishers/subscribers)
        self.config = zenoh_config or zenoh.Config()
        self.session = zenoh.open(self.config)
        
        # Initialize liveliness manager
        self.liveliness_manager = LivelinessManager(self.session)
        
        # Track publishers and subscribers
        self.publishers: Dict[str, Any] = {}
        self.subscribers: Dict[str, Any] = {}
        
        print(f"Node '{self.node_name}' created with Zenoh session")
    
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
        
        # Close session
        if hasattr(self, 'session'):
            self.session.close()
        
        print(f"Node '{self.node_name}' destroyed")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.destroy_node()
    
    def __del__(self):
        """Destructor: ensure cleanup."""
        try:
            self.destroy_node()
        except:
            pass  # Ignore errors during cleanup
