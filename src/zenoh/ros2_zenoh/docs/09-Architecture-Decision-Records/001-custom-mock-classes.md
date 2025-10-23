---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: adr
number: "001"
title: "ADR-001: Use Custom Mock Classes for Testing"
summary: "Use custom mock classes instead of unittest.mock for type safety and domain-specific testing APIs"

# ADR-specific fields
status: "accepted"
date: "2025-10-23"
deciders: ["ros2-zenoh-team"]
consulted: []
informed: []

# Optional metadata
tags: ["testing", "mocking", "architecture"]
related_patterns: ["dependency-injection", "testable-components"]
related_specs: ["Testing-Mocks"]
supersedes: null
superseded_by: null

# Future-proofing
ros2_zenoh:
  languages: ["python"]
  components: ["testing"]
  phase: "1-python-core"
---

# ADR-001: Use Custom Mock Classes for Testing

## Status

**Accepted** (2025-10-23)

---

## Context

When testing ROS2 Zenoh components, we need to mock ROS2 entities like `Node`, `Publisher`, `Subscription`, `Client`, `Service`, `ActionClient`, and `ActionServer`. 

There are two main approaches to mocking in Python:

1. **Generic mocking**: Use `unittest.mock.Mock` with helper functions
2. **Custom mock classes**: Create explicit classes like `MockPublisher`, `MockNode`, etc.
3. **Hybrid**: Mix both approaches

### Requirements

- **Type safety**: Tests should catch type errors at development time
- **IDE support**: Autocomplete should work for mock methods
- **Clarity**: Test code should be self-documenting
- **Domain-specific APIs**: Need methods like `inject_message()`, `get_published_messages()` that make sense for ROS2
- **Easy verification**: Should be simple to verify interactions and state

### Problem with `unittest.mock.Mock`

```python
# Using unittest.mock - unclear and untyped
mock_node = Mock()
mock_pub = Mock()
mock_node.create_publisher.return_value = mock_pub

# What methods does mock_pub have? IDE doesn't know.
# What type of messages? Unclear.
mock_pub.publish(SomeMessage())  # No type checking

# Later in test - magic strings
mock_pub.publish.assert_called_once_with(...)
```

**Issues:**
- No IDE autocomplete
- No type checking
- Unclear what the mock represents
- Generic `assert_called_with()` doesn't express domain intent

---

## Decision

**Use custom mock classes that mirror production APIs with domain-specific testing methods.**

All testing mocks will be explicit Python classes with:
- Clear type signatures
- IDE autocomplete support
- Domain-specific helper methods (e.g., `inject_message()`, `get_latest()`)
- Explicit state tracking (e.g., `messages: list[T]`)

---

## Consequences

### Positive

- ✅ **Type safety**: MyPy and IDEs catch errors before runtime
- ✅ **Self-documenting**: `mock.get_published_messages()` is clearer than `mock.publish.call_args_list`
- ✅ **Better DX**: Autocomplete works, reducing cognitive load
- ✅ **Domain-specific**: Methods express ROS2 concepts naturally
- ✅ **Easier to extend**: Adding `filter_messages()`, `wait_for_message()` is straightforward
- ✅ **Discoverable**: New users can explore mock APIs via IDE

### Negative

- ⚠️ **More code to maintain**: Custom classes require implementation and updates
- ⚠️ **Learning curve**: Developers must learn our mock API (though it mirrors production API)
- ⚠️ **Potential drift**: Mock API could diverge from production if not careful

### Neutral

- 🔸 Different from `rclpy` testing patterns (they use `unittest.mock`)
- 🔸 More upfront work, but less debugging later

---

## Alternatives Considered

### Option 1: unittest.mock.Mock

**Pros:**
- Standard library - no new dependencies
- Flexible - can mock anything on the fly
- Familiar to Python developers

**Cons:**
- No type safety
- No IDE support
- Generic API doesn't express ROS2 domain
- Tests become verbose with `call_args_list` inspection

**Why rejected:**
- Type safety and IDE support are critical for developer experience
- Domain-specific methods make tests more readable

### Option 2: Hybrid Approach

**Pros:**
- Use custom mocks for common cases
- Fall back to `Mock` for edge cases

**Cons:**
- Inconsistent testing patterns
- Developers must learn two approaches
- Type safety only works partially

**Why rejected:**
- Inconsistency leads to confusion
- If custom mocks don't handle a case, we should extend them, not bypass them

---

## References

- Python `unittest.mock` documentation: https://docs.python.org/3/library/unittest.mock.html
- Original design discussion: `TESTING_DESIGN_DECISIONS.md`
- Related spec: [[Testing-Mocks]]

---

## Implementation Notes

### Core Mock Classes

```python
from typing import Generic, TypeVar, Optional
from collections.abc import Callable
import asyncio

T = TypeVar('T')

class MockPublisher(Generic[T]):
    """Mock publisher that records all published messages."""
    
    def __init__(self, msg_type: type[T], topic: str):
        self.msg_type = msg_type
        self.topic = topic
        self.messages: list[T] = []
    
    def publish(self, msg: T) -> None:
        """Record published message."""
        if not isinstance(msg, self.msg_type):
            raise TypeError(f"Expected {self.msg_type}, got {type(msg)}")
        self.messages.append(msg)
    
    def get_latest(self) -> Optional[T]:
        """Get most recently published message."""
        return self.messages[-1] if self.messages else None
    
    def clear(self) -> None:
        """Clear message history."""
        self.messages.clear()


class MockSubscription(Generic[T]):
    """Mock subscription that can inject messages."""
    
    def __init__(
        self,
        msg_type: type[T],
        topic: str,
        callback: Callable[[T], None],
    ):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
    
    async def inject_message(self, msg: T) -> None:
        """Inject message to trigger callback."""
        if not isinstance(msg, self.msg_type):
            raise TypeError(f"Expected {self.msg_type}, got {type(msg)}")
        
        if asyncio.iscoroutinefunction(self.callback):
            await self.callback(msg)
        else:
            self.callback(msg)


class MockNode:
    """Mock node for dependency injection in tests."""
    
    def __init__(self, name: str = "test_node"):
        self.name = name
        self._publishers: dict[str, MockPublisher] = {}
        self._subscriptions: dict[str, MockSubscription] = {}
    
    def create_publisher(
        self,
        msg_type: type[T],
        topic: str,
        **kwargs
    ) -> MockPublisher[T]:
        """Create mock publisher."""
        pub = MockPublisher(msg_type, topic)
        self._publishers[topic] = pub
        return pub
    
    def create_subscription(
        self,
        msg_type: type[T],
        topic: str,
        callback: Callable[[T], None],
        **kwargs
    ) -> MockSubscription[T]:
        """Create mock subscription."""
        sub = MockSubscription(msg_type, topic, callback)
        self._subscriptions[topic] = sub
        return sub
    
    def get_published_messages(self, topic: str) -> list:
        """Get all messages published to topic."""
        if topic not in self._publishers:
            raise KeyError(f"No publisher for topic: {topic}")
        return self._publishers[topic].messages
    
    async def inject_to_subscription(self, topic: str, msg) -> None:
        """Inject message to subscription callback."""
        if topic not in self._subscriptions:
            raise KeyError(f"No subscription for topic: {topic}")
        await self._subscriptions[topic].inject_message(msg)
```

### Usage Example

```python
from geometry_msgs.msg import Twist

async def test_velocity_publisher():
    # Setup
    node = MockNode("test_nav")
    component = NavigationComponent(node)  # Dependency injection
    
    # Act
    component.set_target_velocity(1.5)
    
    # Assert - clear, type-safe
    messages = node.get_published_messages("/cmd_vel")
    assert len(messages) == 1
    assert messages[0].linear.x == 1.5
    
    # Or use publisher directly
    pub = node._publishers["/cmd_vel"]
    latest = pub.get_latest()
    assert latest.linear.x == 1.5
```

### Migration Path

Existing code using production classes:
```python
class NavigationComponent:
    def __init__(self, node):
        self.node = node  # Accept any node-like object
        self.pub = node.create_publisher(Twist, "/cmd_vel")
```

No changes needed! Just inject `MockNode` instead of real `Node` in tests.

---

## Review History

| Date | Reviewer | Decision |
|------|----------|----------|
| 2025-10-23 | ros2-zenoh-team | Accepted |


