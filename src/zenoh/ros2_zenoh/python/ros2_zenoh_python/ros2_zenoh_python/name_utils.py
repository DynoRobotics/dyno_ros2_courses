"""
Name utilities for ROS2 topic and namespace resolution.

Implements ROS2 naming conventions for namespaces and topics.
"""


def normalize_namespace(namespace: str) -> str:
    """
    Normalize a namespace to ROS2 conventions.
    
    Rules:
    - Empty string stays empty
    - Must start with '/' if not empty
    - Must NOT end with '/' 
    - No double slashes '//'
    
    Args:
        namespace: Raw namespace string
        
    Returns:
        Normalized namespace
        
    Examples:
        >>> normalize_namespace("")
        ""
        >>> normalize_namespace("my_ns")
        "/my_ns"
        >>> normalize_namespace("/my_ns/")
        "/my_ns"
        >>> normalize_namespace("//my_ns//sub")
        "/my_ns/sub"
    """
    if not namespace:
        return ""
    
    # Remove leading/trailing whitespace
    ns = namespace.strip()
    if not ns:
        return ""
    
    # Ensure starts with /
    if not ns.startswith('/'):
        ns = '/' + ns
    
    # Remove trailing /
    while ns.endswith('/') and len(ns) > 1:
        ns = ns[:-1]
    
    # Remove double slashes
    while '//' in ns:
        ns = ns.replace('//', '/')
    
    return ns


def resolve_topic_name(topic: str, namespace: str = "") -> str:
    """
    Resolve a topic name with a namespace following ROS2 conventions.
    
    Rules:
    - Absolute topics (starting with '/') are used as-is
    - Relative topics are prefixed with the namespace
    - Empty namespace means topic must be absolute or defaults to '/'
    
    Args:
        topic: Topic name (absolute or relative)
        namespace: Node namespace (already normalized)
        
    Returns:
        Fully resolved topic name (always starts with '/')
        
    Examples:
        >>> resolve_topic_name("/cmd_vel", "/my_robot")
        "/cmd_vel"
        >>> resolve_topic_name("cmd_vel", "/my_robot")
        "/my_robot/cmd_vel"
        >>> resolve_topic_name("cmd_vel", "")
        "/cmd_vel"
        >>> resolve_topic_name("sensors/imu", "/my_robot")
        "/my_robot/sensors/imu"
    """
    # Strip whitespace
    topic = topic.strip()
    if not topic:
        raise ValueError("Topic name cannot be empty")
    
    # Absolute topics (starting with /) are used as-is
    if topic.startswith('/'):
        return topic
    
    # Relative topics are prefixed with namespace
    ns = normalize_namespace(namespace)
    if ns:
        return f"{ns}/{topic}"
    else:
        return f"/{topic}"


def get_fqn(node_name: str, namespace: str = "") -> str:
    """
    Get the fully qualified name for a node.
    
    Args:
        node_name: Node name
        namespace: Node namespace
        
    Returns:
        Fully qualified node name
        
    Examples:
        >>> get_fqn("my_node", "/my_robot")
        "/my_robot/my_node"
        >>> get_fqn("my_node", "")
        "/my_node"
    """
    ns = normalize_namespace(namespace)
    if ns:
        return f"{ns}/{node_name}"
    else:
        return f"/{node_name}"

