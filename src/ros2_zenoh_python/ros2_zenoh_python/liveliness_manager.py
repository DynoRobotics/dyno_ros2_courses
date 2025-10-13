"""
Liveliness Manager Module

Handles ROS 2 metadata publishing via Zenoh liveliness tokens.
Manages topic registration and discovery for ROS 2 compatibility.
"""

import zenoh
from typing import Optional


class LivelinessManager:
    """Manages ROS 2 liveliness tokens for metadata publishing."""
    
    # ROS 2 metadata constants
    ADMIN_SPACE = "@ros2_lv"
    DOMAIN_ID = 0
    ENTITY_PUBLISHER = "MP"
    ENTITY_SUBSCRIBER = "MS"
    KEYEXPR_DELIMITER = "/"
    SLASH_REPLACEMENT = "%"
    
    def __init__(self, session: zenoh.Session):
        """
        Initialize the liveliness manager.
        
        Args:
            session: Zenoh session instance
        """
        self.session = session
        self.tokens = {}  # Store active tokens
    
    def mangle_name(self, name: str) -> str:
        """Replace "/" instances with "%" for Zenoh keyexpr compatibility."""
        return name.replace("/", self.SLASH_REPLACEMENT)
    
    def qos_to_keyexpr(self, reliability: int = 1, durability: int = 2, 
                      history: int = 1, depth: int = 10) -> str:
        """
        Convert QoS settings to keyexpr format.
        
        Format: <reliability>:<durability>:<history>,<depth>:<deadline_sec>,<deadline_nsec>:<lifespan_sec>,<lifespan_nsec>:<liveliness>,<liveliness_sec>,<liveliness_nsec>
        """
        return f"{reliability}:{durability}:{history},{depth}:0,0:0,0:1,0,0"
    
    def create_liveliness_keyexpr(self, entity_type: str, topic_name: str, 
                                message_type: str, type_hash: str,
                                node_name: str = "zenoh_node",
                                node_namespace: str = "",
                                qos: Optional[str] = None) -> str:
        """
        Create ROS 2 liveliness token keyexpr for metadata publishing.
        
        Args:
            entity_type: "MP" for publisher, "MS" for subscriber
            topic_name: ROS 2 topic name (e.g., "/turtle1/cmd_vel")
            message_type: ROS 2 message type (e.g., "geometry_msgs::msg::dds_::Twist_")
            type_hash: Message type hash
            node_name: ROS 2 node name
            node_namespace: ROS 2 node namespace
            qos: QoS settings as string
            
        Returns:
            Liveliness token keyexpr
        """
        if qos is None:
            qos = self.qos_to_keyexpr()
        
        zid = str(self.session.info.zid())
        nid = "1"  # Node ID (arbitrary, but consistent)
        entity_id = "1"  # Entity ID (arbitrary, but unique)
        
        # Build keyexpr parts - must include ALL parts for non-node entities
        parts = [
            self.ADMIN_SPACE,                    # 0: AdminSpace
            str(self.DOMAIN_ID),                 # 1: DomainId
            zid,                                # 2: Zid
            nid,                                # 3: Nid
            entity_id,                          # 4: Id
            entity_type,                        # 5: EntityStr
            "_",                                # 6: Enclave (placeholder for empty)
            "_",                                # 7: Namespace (placeholder for empty)
            self.mangle_name(node_name),        # 8: NodeName
            self.mangle_name(topic_name),       # 9: TopicName
            self.mangle_name(message_type),     # 10: TopicType
            self.mangle_name(type_hash),        # 11: TopicTypeHash
            qos                                 # 12: TopicQoS
        ]
        
        return self.KEYEXPR_DELIMITER.join(parts)
    
    def declare_publisher_token(self, topic_name: str, message_type: str, 
                               type_hash: str, node_name: str = "zenoh_publisher",
                               node_namespace: str = "") -> zenoh.LivelinessToken:
        """
        Declare a liveliness token for a publisher.
        
        Args:
            topic_name: ROS 2 topic name
            message_type: ROS 2 message type
            type_hash: Message type hash
            node_name: ROS 2 node name
            node_namespace: ROS 2 node namespace
            
        Returns:
            Liveliness token instance
        """
        keyexpr = self.create_liveliness_keyexpr(
            entity_type=self.ENTITY_PUBLISHER,
            topic_name=topic_name,
            message_type=message_type,
            type_hash=type_hash,
            node_name=node_name,
            node_namespace=node_namespace
        )
        
        token = self.session.liveliness().declare_token(zenoh.KeyExpr(keyexpr))
        self.tokens[keyexpr] = token
        return token
    
    def declare_subscriber_token(self, topic_name: str, message_type: str, 
                                type_hash: str, node_name: str = "zenoh_subscriber",
                                node_namespace: str = "") -> zenoh.LivelinessToken:
        """
        Declare a liveliness token for a subscriber.
        
        Args:
            topic_name: ROS 2 topic name
            message_type: ROS 2 message type
            type_hash: Message type hash
            node_name: ROS 2 node name
            node_namespace: ROS 2 node namespace
            
        Returns:
            Liveliness token instance
        """
        keyexpr = self.create_liveliness_keyexpr(
            entity_type=self.ENTITY_SUBSCRIBER,
            topic_name=topic_name,
            message_type=message_type,
            type_hash=type_hash,
            node_name=node_name,
            node_namespace=node_namespace
        )
        
        token = self.session.liveliness().declare_token(zenoh.KeyExpr(keyexpr))
        self.tokens[keyexpr] = token
        return token
    
    def undeclare_token(self, keyexpr: str):
        """Undeclare a liveliness token."""
        if keyexpr in self.tokens:
            self.tokens[keyexpr].undeclare()
            del self.tokens[keyexpr]
    
    def undeclare_all_tokens(self):
        """Undeclare all active liveliness tokens."""
        for token in self.tokens.values():
            try:
                token.undeclare()
            except zenoh.ZError:
                pass  # Token already undeclared
        self.tokens.clear()
    
    def __del__(self):
        """Cleanup: undeclare all tokens when manager is destroyed."""
        self.undeclare_all_tokens()
