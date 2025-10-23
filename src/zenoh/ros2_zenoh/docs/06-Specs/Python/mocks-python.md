---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: spec
id: "spec-mocks-python"
title: "Testing Mocks - Python Binding"
summary: "Python-specific API and implementation details for testing mocks"

# Spec-specific fields
status: "proposed"
version: "1.0.0"
component_type: "testing-utility"

# Metadata
tags: ["testing", "mocking", "dependency-injection", "python"]
related_specs: ["Mocks-Core", "Clock-Python"]
related_adrs: ["ADR-001"]

# Future-proofing
ros2_zenoh:
  languages: ["python"]
  components: ["testing"]
  phase: "1-python-core"
  test_coverage: "100%"
---

# Testing Mocks - Python Binding

**Python-specific implementation of [[Mocks-Core]]**

For universal behavior specification, see **[[Mocks-Core]]**.

---

## Python API

### Module

```python
from ros2_zenoh_python.testing import (
    MockNode,
    MockPublisher,
    MockSubscription,
    MockClient,
    MockService,
)
```

### Class Definitions

#### MockPublisher

```python
from typing import Generic, TypeVar, Optional, Callable, List

T = TypeVar('T')

class MockPublisher(Generic[T]):
    """
    Mock publisher for testing.
    
    Records all published messages for inspection.
    """
    
    def __init__(
        self,
        msg_type: type[T],
        topic: str,
        depth: Optional[int] = None
    ):
        """
        Create mock publisher.
        
        Args:
            msg_type: Message type for validation
            topic: Topic name
            depth: None (unlimited, default) or int (ring buffer)
        """
        ...
    
    def publish(self, msg: T) -> None:
        """
        Record published message.
        
        Args:
            msg: Message to publish
            
        Raises:
            TypeError: If msg is not instance of msg_type
        """
        ...
    
    def get_latest(self) -> Optional[T]:
        """Get most recently published message (or None if empty)."""
        ...
    
    def clear(self) -> None:
        """Clear message history."""
        ...
    
    def get_message_count(self) -> int:
        """Get total number of messages published."""
        ...
    
    def filter_messages(self, predicate: Callable[[T], bool]) -> List[T]:
        """Get messages matching predicate."""
        ...
    
    @property
    def messages(self) -> List[T]:
        """All published messages (chronological order)."""
        ...
```

#### MockSubscription

```python
class MockSubscription(Generic[T]):
    """
    Mock subscription for testing.
    
    Allows injecting messages to trigger callbacks.
    """
    
    def __init__(
        self,
        msg_type: type[T],
        topic: str,
        callback: Callable[[T], None] | Callable[[T], Awaitable[None]],
        depth: Optional[int] = None
    ):
        """
        Create mock subscription.
        
        Args:
            msg_type: Message type for validation
            topic: Topic name
            callback: User callback (sync or async)
            depth: Optional depth (currently unused, for future)
        """
        ...
    
    async def inject_message(self, msg: T) -> None:
        """
        Inject message (calls callback asynchronously).
        
        Args:
            msg: Message to inject
            
        Raises:
            TypeError: If msg is not instance of msg_type
        """
        ...
    
    def inject_message_sync(self, msg: T) -> None:
        """
        Inject message (calls callback synchronously).
        
        Args:
            msg: Message to inject
            
        Raises:
            TypeError: If msg is not instance of msg_type, or callback is async
        """
        ...
    
    @property
    def injected_count(self) -> int:
        """Number of messages injected."""
        ...
```

#### MockClient

```python
class MockClient(Generic[T]):
    """
    Mock service client for testing.
    
    Records requests and provides mock responses.
    """
    
    def __init__(self, srv_type: type[T], service_name: str):
        """Create mock service client."""
        ...
    
    async def call_async(self, request: T.Request) -> T.Response:
        """
        Make service call (returns mock response).
        
        Args:
            request: Service request
            
        Returns:
            Mock response
            
        Raises:
            RuntimeError: If no response configured
        """
        ...
    
    def set_response(self, response: T.Response) -> None:
        """
        Set next response (one-time).
        
        Args:
            response: Response to return on next call
        """
        ...
    
    def set_default_response(
        self, 
        fn: Callable[[T.Request], T.Response]
    ) -> None:
        """
        Set default response generator.
        
        Args:
            fn: Function that generates response from request
        """
        ...
    
    def get_latest_request(self) -> Optional[T.Request]:
        """Get most recent request."""
        ...
    
    @property
    def requests(self) -> List[T.Request]:
        """All requests made."""
        ...
```

#### MockService

```python
class MockService(Generic[T]):
    """
    Mock service server for testing.
    
    Simulates service server with user-provided callback.
    """
    
    def __init__(
        self,
        srv_type: type[T],
        service_name: str,
        callback: Callable[[T.Request], T.Response]
    ):
        """Create mock service server."""
        ...
    
    async def inject_request(self, request: T.Request) -> T.Response:
        """
        Inject request and get response from callback.
        
        Args:
            request: Service request
            
        Returns:
            Response from callback
        """
        ...
    
    def get_latest_request(self) -> Optional[T.Request]:
        """Get most recent request."""
        ...
    
    @property
    def requests(self) -> List[T.Request]:
        """All requests received."""
        ...
```

#### MockNode

```python
class MockNode:
    """
    Mock node for testing.
    
    Central hub for creating and managing all mock entities.
    Mirrors production Node API.
    """
    
    def __init__(self, name: str = "test_node", namespace: str = ""):
        """
        Create mock node.
        
        Args:
            name: Node name
            namespace: Node namespace
        """
        ...
    
    def create_publisher(
        self,
        msg_type: type[T],
        topic: str,
        *,
        qos_profile: Optional[dict] = None,
        depth: Optional[int] = None
    ) -> MockPublisher[T]:
        """
        Create mock publisher.
        
        Args:
            msg_type: Message type
            topic: Topic name (relative or absolute)
            qos_profile: Ignored (for API compatibility)
            depth: Message depth (None = unlimited)
            
        Returns:
            Mock publisher
        """
        ...
    
    def create_subscription(
        self,
        msg_type: type[T],
        topic: str,
        callback: Callable[[T], None] | Callable[[T], Awaitable[None]],
        *,
        qos_profile: Optional[dict] = None,
        depth: Optional[int] = None
    ) -> MockSubscription[T]:
        """
        Create mock subscription.
        
        Args:
            msg_type: Message type
            topic: Topic name (relative or absolute)
            callback: Callback function (sync or async)
            qos_profile: Ignored (for API compatibility)
            depth: Message depth (currently unused)
            
        Returns:
            Mock subscription
        """
        ...
    
    def create_client(
        self,
        srv_type: type[T],
        service_name: str
    ) -> MockClient[T]:
        """Create mock service client."""
        ...
    
    def create_service(
        self,
        srv_type: type[T],
        service_name: str,
        callback: Callable[[T.Request], T.Response]
    ) -> MockService[T]:
        """Create mock service server."""
        ...
    
    # Convenience methods for testing
    
    def get_published_messages(self, topic: str) -> List:
        """
        Get all messages published to topic.
        
        Args:
            topic: Topic name
            
        Returns:
            List of published messages
            
        Raises:
            KeyError: If no publisher for topic
        """
        ...
    
    async def inject_to_subscription(self, topic: str, msg) -> None:
        """
        Inject message to subscription callback.
        
        Args:
            topic: Topic name
            msg: Message to inject
            
        Raises:
            KeyError: If no subscription for topic
        """
        ...
    
    def get_publisher(self, topic: str) -> MockPublisher:
        """
        Get publisher for topic (for inspection).
        
        Args:
            topic: Topic name
            
        Returns:
            Mock publisher
            
        Raises:
            KeyError: If no publisher for topic
        """
        ...
    
    def get_subscription(self, topic: str) -> MockSubscription:
        """
        Get subscription for topic (for injection).
        
        Args:
            topic: Topic name
            
        Returns:
            Mock subscription
            
        Raises:
            KeyError: If no subscription for topic
        """
        ...
```

---

## Implementation Details

### File Location

**Path**: `python/ros2_zenoh_python/ros2_zenoh_python/testing/mocks.py`

### Dependencies

```python
import asyncio
from typing import Generic, TypeVar, Optional, Callable, List, Dict, Any
from collections import deque
from dataclasses import dataclass, field

from ros2_zenoh_python.name_utils import resolve_topic_name, resolve_service_name
```

### Internal Implementation

```python
# Type validation helper
def _validate_type(msg, expected_type, context: str):
    if not isinstance(msg, expected_type):
        raise TypeError(
            f"{context}: Expected {expected_type.__name__}, "
            f"got {type(msg).__name__}"
        )

# MockPublisher implementation
class MockPublisher(Generic[T]):
    def __init__(self, msg_type: type[T], topic: str, depth: Optional[int] = None):
        self.msg_type = msg_type
        self.topic = topic
        self._depth = depth
        self._messages: List[T] = [] if depth is None else deque(maxlen=depth)
    
    def publish(self, msg: T) -> None:
        _validate_type(msg, self.msg_type, "MockPublisher.publish")
        
        if self._depth is None:
            self._messages.append(msg)
        else:
            # deque with maxlen automatically drops oldest
            self._messages.append(msg)
    
    @property
    def messages(self) -> List[T]:
        return list(self._messages)
    
    def get_latest(self) -> Optional[T]:
        return self._messages[-1] if self._messages else None
```

---

## Usage Examples

### Basic Publisher Testing

```python
from ros2_zenoh_python.testing import MockNode
from std_msgs.msg import String
import pytest

async def test_my_component():
    """Test component that publishes messages."""
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

### Subscription Testing

```python
async def test_subscription_callback():
    """Test subscription callback with message injection."""
    # Setup
    node = MockNode("test")
    received = []
    
    sub = node.create_subscription(
        String,
        "/input",
        lambda msg: received.append(msg.data)
    )
    
    # Act
    await node.inject_to_subscription("/input", String(data="test"))
    
    # Assert
    assert received == ["test"]
```

### Service Testing

```python
from example_interfaces.srv import AddTwoInts

async def test_service_client():
    """Test service client with mock responses."""
    # Setup
    client = MockClient(AddTwoInts, "/add")
    client.set_default_response(
        lambda req: AddTwoInts.Response(sum=req.a + req.b)
    )
    
    # Act
    result = await client.call_async(AddTwoInts.Request(a=10, b=32))
    
    # Assert
    assert result.sum == 42
    assert client.get_latest_request().a == 10
```

### Depth Testing

```python
async def test_publisher_overflow():
    """Test ring buffer semantics with depth."""
    pub = MockPublisher(String, "/test", depth=3)
    
    # Publish 5 messages
    for i in range(5):
        pub.publish(String(data=f"msg{i}"))
    
    # Only last 3 retained (oldest 2 dropped)
    assert len(pub.messages) == 3
    assert [m.data for m in pub.messages] == ["msg2", "msg3", "msg4"]
```

### With Clock (Time-Based Testing)

```python
from ros2_zenoh_python.testing import MockNode, Clock, TimeMode
import asyncio

async def test_periodic_publisher():
    """Test component with periodic publishing."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    node = MockNode("test")
    
    # Component publishes every 1.0s
    component = PeriodicPublisher(node, clock, period=1.0)
    
    # Start component
    task = asyncio.create_task(component.run())
    
    # Fast-forward time
    await clock.advance_by(3.0)
    
    # Check 3 messages published
    messages = node.get_published_messages("/periodic")
    assert len(messages) == 3
    
    # Cleanup
    task.cancel()
```

---

## Pytest Fixtures

### Common Fixtures

```python
# conftest.py
import pytest
from ros2_zenoh_python.testing import MockNode

@pytest.fixture
def mock_node():
    """Fixture for mock node."""
    return MockNode(name="test_node", namespace="")

@pytest.fixture
def namespaced_mock_node():
    """Fixture for namespaced mock node."""
    return MockNode(name="test_node", namespace="/robot1")
```

### Usage with Fixtures

```python
def test_with_fixture(mock_node):
    """Test using mock_node fixture."""
    pub = mock_node.create_publisher(String, "/topic")
    pub.publish(String(data="test"))
    
    assert len(mock_node.get_published_messages("/topic")) == 1
```

---

## Type Hints and IDE Support

### Full Type Safety

```python
from std_msgs.msg import String

# Type checker knows pub.messages is List[String]
pub: MockPublisher[String] = MockPublisher(String, "/topic")
pub.publish(String(data="hello"))

# IDE autocompletes .data
latest = pub.get_latest()
if latest:
    print(latest.data)  # Type: str
```

### Protocol for Dependency Injection

```python
from typing import Protocol

class NodeProtocol(Protocol):
    """Type hint for node dependency."""
    
    def create_publisher(self, msg_type, topic, **kwargs): ...
    def create_subscription(self, msg_type, topic, callback, **kwargs): ...

# Works with both MockNode and real Node
class Component:
    def __init__(self, node: NodeProtocol):
        self.node = node
```

---

## Best Practices

### ✅ Do

```python
# Use dependency injection
class MyComponent:
    def __init__(self, node: MockNode):
        self.node = node

# Test with mocks
def test_component():
    node = MockNode()
    component = MyComponent(node)
    # ...

# Check published messages
messages = node.get_published_messages("/topic")
assert messages[0].data == "expected"

# Use type hints
pub: MockPublisher[String] = node.create_publisher(String, "/topic")
```

### ❌ Don't

```python
# Don't create real Zenoh sessions in unit tests
session = zenoh.open(...)  # Slow, requires infrastructure

# Don't use unittest.mock.Mock for ROS2 entities
mock_pub = Mock()  # No type safety, unclear API

# Don't access private fields
pub._messages.append(...)  # Use public API instead
```

---

## Testing the Mocks Themselves

### Meta-Tests

```python
def test_mock_publisher_type_validation():
    """Mocks enforce type safety."""
    from std_msgs.msg import String, Int32
    
    pub = MockPublisher(String, "/test")
    
    # Valid
    pub.publish(String(data="hello"))
    
    # Invalid
    with pytest.raises(TypeError, match="Expected String"):
        pub.publish(Int32(data=42))

def test_mock_publisher_depth_enforcement():
    """Mocks enforce ring buffer semantics."""
    pub = MockPublisher(String, "/test", depth=2)
    
    pub.publish(String(data="1"))
    pub.publish(String(data="2"))
    pub.publish(String(data="3"))  # Drops "1"
    
    assert len(pub.messages) == 2
    assert pub.messages[0].data == "2"
    assert pub.messages[1].data == "3"
```

---

## Related Documentation

- **Core Specification**: [[Mocks-Core]]
- **ADR**: [[ADR-001]] - Custom Mock Classes
- **Tutorial**: `02-Tutorials/Testing/Unit-Testing-With-Mocks.md`
- **Pattern**: `04-How-To/Dependency-Injection-For-Testing.md`

---

## Implementation Status

- [x] MockPublisher
- [x] MockSubscription  
- [x] MockClient
- [x] MockService
- [x] MockNode
- [ ] Unit tests (target: 100% coverage)
- [ ] Integration tests with real components
- [ ] Documentation and tutorials



