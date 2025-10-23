---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "ros2_zenoh_python/name_utils.py"
last_generated: "2025-10-23T16:51:15.483809"
tags: ["api", "python", "auto-generated"]
---

# name_utils

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `ros2_zenoh_python/name_utils.py`

Name utilities for ROS2 topic and namespace resolution.

Implements ROS2 naming conventions for namespaces and topics.

## Functions

### `normalize_namespace`

```python
normalize_namespace(namespace: str) -> str
```

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

### `resolve_topic_name`

```python
resolve_topic_name(topic: str, namespace: str) -> str
```

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

### `get_fqn`

```python
get_fqn(node_name: str, namespace: str) -> str
```

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

