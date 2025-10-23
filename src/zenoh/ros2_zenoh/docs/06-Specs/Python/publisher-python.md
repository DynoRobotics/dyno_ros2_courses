---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: spec
id: "spec-python-publisher"
title: "Publisher Python Binding Specification"
summary: "Python-specific API and idioms for Publisher component"

# Spec-specific fields
status: "implemented"
version: "1.0.0"
component_type: "language-binding"

# Metadata
tags: ["publisher", "python", "api", "binding"]
related_specs: ["Core/Publisher", "Subscription-Python", "Node-Python"]
related_adrs: []
related_patterns: ["dependency-injection", "context-manager"]

# Future-proofing
ros2_zenoh:
  languages: ["python"]
  components: ["publisher"]
  phase: "1-python-core"
  test_coverage: 95
---

# Publisher Python Binding

**For universal behavior specification, see [[Core/Publisher]].**

This document specifies Python-specific API, idioms, and implementation details.

---

## Python API

### Class Signature

```python
from typing import Generic, TypeVar, Optional, Type, Any
import zenoh

T = TypeVar('T')

class Publisher(Generic[T]):
    """ROS 2-compatible publisher using Zenoh transport."""
```

### Constructor

```python
def __init__(
    self,
    msg_type: Type[T],
    topic: str,
    node: Optional['Node'] = None,
    qos_profile: Optional[dict] = None,
    encoding: str = 'cdr'
) -> None:
    """
    Initialize a ROS 2-compatible publisher.
    
    Args:
        msg_type: Message type class (must have DDS_TYPE_NAME and TYPE_HASH)
        topic: Topic name (e.g., "/turtle1/cmd_vel"), resolved with namespace
        node: Parent node instance (if None, creates its own Zenoh session)
        qos_profile: QoS profile settings (dict)
        encoding: Serialization encoding: 'cdr', 'json', or 'msgpack'
    
    Raises:
        ValueError: If msg_type missing DDS_TYPE_NAME or TYPE_HASH
    
    Example:
        >>> from std_msgs.msg import String
        >>> pub = Publisher(String, "/chatter")
        >>> pub.publish(String(data="Hello!"))
    """
```

### QoS Profile Format

```python
qos_profile = {
    'reliability': 'reliable',      # or 'best_effort'
    'durability': 'volatile',       # or 'transient_local'
    'history': 'keep_last',         # or 'keep_all'
    'depth': 10                     # int, only for 'keep_last'
}
```

**Defaults:**
- `reliability`: `'reliable'`
- `durability`: `'volatile'`
- `history`: `'keep_last'`
- `depth`: `10`

### Methods

#### publish

```python
def publish(self, msg: T) -> None:
    """
    Publish a message (synchronous).
    
    Args:
        msg: Message instance (must match msg_type)
    
    Note:
        This is synchronous/fire-and-forget. No blocking or backpressure.
        For reliable delivery, use QoS reliability='reliable'.
    
    Example:
        >>> from geometry_msgs.msg import Twist
        >>> pub = Publisher(Twist, "/cmd_vel")
        >>> msg = Twist()
        >>> msg.linear.x = 1.5
        >>> pub.publish(msg)
    """
```

#### wait_for_subscribers

```python
async def wait_for_subscribers(self, timeout: float = 5.0) -> bool:
    """
    Wait for at least one subscriber to be available.
    
    Args:
        timeout: Maximum time to wait in seconds
    
    Returns:
        True if subscriber found, False if timeout
    
    Note:
        rmw_zenoh may have a race between liveliness and data path readiness.
        Applications should retry publish if first message doesn't arrive.
    
    Example:
        >>> pub = Publisher(String, "/test")
        >>> if await pub.wait_for_subscribers(timeout=5.0):
        ...     pub.publish(String(data="Hello!"))
    """
```

#### destroy

```python
def destroy(self) -> None:
    """
    Destroy the publisher and clean up resources.
    
    - Undeclares liveliness token
    - Closes Zenoh session if owned by publisher
    
    Idempotent: safe to call multiple times.
    
    Example:
        >>> pub = Publisher(String, "/test")
        >>> pub.publish(String(data="test"))
        >>> pub.destroy()
    """
```

#### adestroy

```python
async def adestroy(self) -> None:
    """
    Async version of destroy() for use in async context managers.
    
    Currently just calls destroy() since cleanup is non-blocking.
    Provided for API consistency and future async cleanup.
    """
```

---

## Python Idioms

### Context Manager Support

Publishers support both sync and async context managers:

```python
# Sync context manager
with Publisher(String, "/test") as pub:
    pub.publish(String(data="Hello!"))
# Auto-cleanup on exit

# Async context manager
async with ZenohNode("my_node") as node:
    pub = node.create_publisher(String, "/test")
    pub.publish(String(data="Hello!"))
# Auto-cleanup when node exits
```

**Implementation:**
```python
def __enter__(self) -> 'Publisher[T]':
    """Context manager entry."""
    return self

def __exit__(self, exc_type, exc_val, exc_tb) -> None:
    """Context manager exit."""
    self.destroy()

# Note: No __aenter__/__aexit__ on Publisher itself
# (async context is handled by Node)
```

### Type Hints

Publisher is generic over message type:

```python
from std_msgs.msg import String
from typing import reveal_type

pub: Publisher[String] = Publisher(String, "/test")
reveal_type(pub)  # Publisher[String]

# IDE knows that msg must be String
pub.publish(String(data="test"))  # ✓ Type checks
pub.publish(42)  # ✗ Type error
```

### Property Access

State is accessible (but should be treated as read-only):

```python
pub = Publisher(String, "/chatter")

# Read-only access
print(pub.topic)            # "/chatter"
print(pub.sequence_number)  # Current sequence (e.g., 42)
print(pub.encoding)         # "cdr"
print(pub.msg_type)         # <class 'std_msgs.msg.String'>

# Don't modify! (no enforcement, but breaks invariants)
pub.sequence_number = 0  # ⚠️ DON'T DO THIS
```

---

## Usage Patterns

### Standalone Publisher (Legacy)

```python
from std_msgs.msg import String

# Creates own Zenoh session
pub = Publisher(String, "/chatter")

try:
    pub.publish(String(data="Hello World!"))
finally:
    pub.destroy()
```

### Node-based Publisher (Recommended)

```python
from ros2_zenoh_python import ZenohNode
from std_msgs.msg import String

async with ZenohNode("my_node", namespace="/robot1") as node:
    pub = node.create_publisher(String, "/status")
    
    pub.publish(String(data="Ready"))
    
    # Wait for subscriber before critical message
    if await pub.wait_for_subscribers(timeout=5.0):
        pub.publish(String(data="Critical update"))
    else:
        print("Warning: No subscribers found")
# Auto-cleanup
```

### High-Frequency Publishing

```python
from geometry_msgs.msg import Twist
import asyncio

async def publish_control_loop():
    async with ZenohNode("controller") as node:
        pub = node.create_publisher(Twist, "/cmd_vel")
        
        # Wait for subscriber once
        await pub.wait_for_subscribers(timeout=10.0)
        
        # Fast loop
        while True:
            msg = Twist()
            msg.linear.x = compute_velocity()
            pub.publish(msg)  # Synchronous, zero-copy
            
            await asyncio.sleep(0.01)  # 100 Hz
```

### Best Effort QoS

```python
from sensor_msgs.msg import Image

# For sensor data: best_effort + volatile
qos = {
    'reliability': 'best_effort',
    'durability': 'volatile',
    'history': 'keep_last',
    'depth': 1  # Only keep latest
}

pub = Publisher(Image, "/camera/image", qos_profile=qos)
```

### Transient Local (Late Joiners)

```python
from std_msgs.msg import String

# For status topics: reliable + transient_local
qos = {
    'reliability': 'reliable',
    'durability': 'transient_local',
    'history': 'keep_last',
    'depth': 1
}

pub = Publisher(String, "/robot/status", qos_profile=qos)
pub.publish(String(data="ready"))

# Late-joining subscribers receive last message
```

---

## Testing Patterns

### With Mock Node

```python
from ros2_zenoh_python.testing import MockNode
from std_msgs.msg import String

async def test_my_component():
    # Setup
    node = MockNode("test")
    component = MyComponent(node)  # Dependency injection
    
    # Act
    component.publish_status("ready")
    
    # Assert
    messages = node.get_published_messages("/status")
    assert len(messages) == 1
    assert messages[0].data == "ready"
```

### Integration Test with rclpy

```python
import rclpy
from std_msgs.msg import String

async def test_interop():
    # rclpy subscriber
    rclpy.init()
    rclpy_node = rclpy.create_node("test_sub")
    received = []
    sub = rclpy_node.create_subscription(
        String, "/test",
        lambda msg: received.append(msg.data),
        10
    )
    
    # Zenoh publisher
    async with ZenohNode("test_pub") as node:
        pub = node.create_publisher(String, "/test")
        await pub.wait_for_subscribers(timeout=5.0)
        pub.publish(String(data="Hello ROS2!"))
        
        # Spin rclpy to process
        rclpy.spin_once(rclpy_node, timeout_sec=1.0)
        
    assert received == ["Hello ROS2!"]
    rclpy.shutdown()
```

---

## Error Handling

### Missing Message Metadata

```python
class BadMessage:
    pass  # Missing DDS_TYPE_NAME and TYPE_HASH

try:
    pub = Publisher(BadMessage, "/test")
except ValueError as e:
    print(e)  # "Message type must have DDS_TYPE_NAME and TYPE_HASH"
```

### Session Management

```python
# If publisher owns session, it closes it
pub = Publisher(String, "/test")  # Creates own session
pub.destroy()  # Closes session

# If node owns session, publisher doesn't close it
async with ZenohNode("test") as node:
    pub = node.create_publisher(String, "/test")
    pub.destroy()  # Does NOT close node's session
```

---

## Performance Considerations

### Pre-bound Serializer

Serializer is looked up once at construction:

```python
# At __init__ time:
self._serialize = msg_type.get_serializer(encoding)  # Once

# At publish time:
payload = self._serialize(msg)  # Zero overhead!
```

No dictionary lookup or dynamic dispatch on hot path.

### Zero-Copy Publishing

For large messages, consider using Zenoh's shared memory transport (future feature):

```python
# Future API (not yet implemented)
pub = Publisher(Image, "/camera", use_shm=True)
pub.publish(large_image)  # Zero-copy via shared memory
```

---

## Dependencies

**Required:**
- `zenoh` >= 1.0.0
- `ros2_zenoh_python.liveliness_manager`
- `ros2_zenoh_python.name_utils`

**Type-checking only:**
- `typing` (Generic, TypeVar, Optional, Type)

---

## Implementation

**File**: `python/ros2_zenoh_python/ros2_zenoh_python/publisher.py`

**Lines of code**: ~280

**Test coverage**: 95%

**Test file**: `python/ros2_zenoh_python/tests/test_publisher.py`

---

## Migration from rclpy

### API Differences

| rclpy | ros2-zenoh-python |
|-------|-------------------|
| `node.create_publisher(String, '/topic', 10)` | `node.create_publisher(String, '/topic')` |
| QoS profile object | QoS dict |
| `publisher.get_subscription_count()` | Use `wait_for_subscribers()` |
| `publisher.destroy()` | Same |

### Example Migration

**Before (rclpy):**
```python
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('my_node')
pub = node.create_publisher(String, '/chatter', 10)

msg = String()
msg.data = 'Hello'
pub.publish(msg)

node.destroy_node()
rclpy.shutdown()
```

**After (ros2-zenoh-python):**
```python
from ros2_zenoh_python import ZenohNode
from std_msgs.msg import String

async with ZenohNode('my_node') as node:
    pub = node.create_publisher(String, '/chatter')
    pub.publish(String(data='Hello'))
# Auto-cleanup
```

---

## Related Documentation

- [[Core/Publisher]] - Universal behavior specification
- [[Subscription-Python]] - Python subscription API
- [[Node-Python]] - Python node API
- [[../07-API-Reference/Python-Core/Publisher.generated]] - Auto-generated API reference


