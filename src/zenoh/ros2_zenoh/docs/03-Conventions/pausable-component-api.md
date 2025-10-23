---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: convention
id: "convention-pausable-api"
title: "Pausable Component API Convention"
summary: "Standard service API for runtime pause/resume control"

# Metadata
tags: ["convention", "api", "pausable", "debugging", "testing"]
related_specs: []
related_adrs: ["ADR-005"]
related_patterns: ["pausable-component"]

# Future-proofing
ros2_zenoh:
  status: "recommended"
  applies_to: ["components", "nodes"]
---

# Pausable Component API Convention

**Standard service API for runtime pause/resume control**

## Overview

Components that support runtime pause/resume **SHOULD** implement this standard API convention for interoperability with debugging tools, test frameworks, and operational dashboards.

**This is a convention, not a requirement**: Implementing these services is optional, but when implemented, they should follow this standard.

---

## Motivation

### Use Cases

1. **Debugging**: Pause camera processing while debugging planning logic
2. **Testing**: Control timing deterministically in integration tests
3. **Reconfiguration**: Pause while updating parameters or models
4. **Resource Management**: Temporarily disable expensive computations
5. **System Coordination**: Orchestrate startup/shutdown sequences

### Why a Convention?

- **Interoperability**: Tools can discover and control any compliant component
- **Consistency**: Same API across all pausable components
- **Discoverability**: `ros2 service list | grep pause` shows pausable components
- **Tooling**: Enables generic pause/resume tools

---

## Standard API

### Required Services

Every pausable component **SHOULD** implement these two services:

| Service | Type | Description |
|---------|------|-------------|
| `~/pause` | `std_srvs/srv/Trigger` | Pause component processing |
| `~/resume` | `std_srvs/srv/Trigger` | Resume component processing |

**Naming convention**: Services use `~` prefix (node-relative).

**Message type**: `std_srvs/srv/Trigger`
```
# Request (empty)
---
# Response
bool success
string message
```

---

### Optional Services

Components **MAY** implement additional services:

| Service | Type | Description |
|---------|------|-------------|
| `~/get_pause_state` | `std_srvs/srv/SetBool` | Query if paused (abuse of SetBool) |
| `~/toggle_pause` | `std_srvs/srv/Trigger` | Toggle pause state |

**Note**: These are less standardized. Prefer `pause`/`resume` for interoperability.

---

## Semantics

### Pause Behavior

**MUST**:
- Stop processing new messages/events
- Respond to service calls with `success=True`
- Be idempotent (pausing when paused is safe no-op)

**MAY**:
- Drop queued messages
- Keep latest message buffered
- Queue messages with bounded buffer
- Actually pause the subscription

**MUST NOT**:
- Block indefinitely
- Lose critical system state
- Crash or raise exceptions

### Resume Behavior

**MUST**:
- Re-enable processing
- Respond to service calls with `success=True`
- Be idempotent (resuming when not paused is safe no-op)

**MAY**:
- Process buffered messages (if any)
- Discard stale buffered messages
- Re-subscribe to topics

**MUST NOT**:
- Process stale data if inappropriate for the component

### State Persistence

**MUST NOT**:
- Persist pause state across restarts
- Components always start in "active" (not paused) state

---

## Discovery

### Finding Pausable Components

```bash
# List all pausable components
ros2 service list | grep "/pause$"

# List all pause/resume services
ros2 service list | grep -E "/(pause|resume)$"

# Get service type
ros2 service type /my_component/pause
```

### Programmatic Discovery

```python
from ros2_zenoh_python import ZenohNode

async with ZenohNode("controller") as node:
    # Discover pausable components
    services = await node.get_service_names_and_types()
    pausable = [
        name.replace("/pause", "")
        for name, types in services
        if name.endswith("/pause") and "std_srvs/srv/Trigger" in types
    ]
    
    print(f"Pausable components: {pausable}")
```

---

## Implementation Strategies

### Strategy 1: Drop Messages (Simplest)

```python
from std_srvs.srv import Trigger

class SimpleComponent:
    def __init__(self, node):
        self.node = node
        self._paused = False
        
        # Standard API
        self.pause_srv = node.create_service(
            Trigger, "~/pause", self._handle_pause
        )
        self.resume_srv = node.create_service(
            Trigger, "~/resume", self._handle_resume
        )
        
        # Subscription
        self.sub = node.create_subscription(
            Image, "/camera", self.on_image
        )
    
    def on_image(self, msg):
        if self._paused:
            return  # Drop message
        
        self.process(msg)
    
    def _handle_pause(self, request, response):
        self._paused = True
        response.success = True
        response.message = "Paused"
        return response
    
    def _handle_resume(self, request, response):
        self._paused = False
        response.success = True
        response.message = "Resumed"
        return response
```

**Pros**: Simple, no buffering issues  
**Cons**: Loses messages while paused

---

### Strategy 2: Keep Latest Only

```python
class LatestOnlyComponent:
    def __init__(self, node):
        self.node = node
        self._paused = False
        self._latest_msg = None
        
        # Standard API services...
        
        self.sub = node.create_subscription(
            Image, "/camera", self.on_image
        )
    
    def on_image(self, msg):
        if self._paused:
            self._latest_msg = msg  # Save latest
            return
        
        self.process(msg)
    
    def _handle_resume(self, request, response):
        self._paused = False
        
        # Process saved message if any
        if self._latest_msg:
            self.process(self._latest_msg)
            self._latest_msg = None
        
        response.success = True
        response.message = "Resumed"
        return response
```

**Pros**: Don't lose final state  
**Cons**: Still lose intermediate messages

---

### Strategy 3: Bounded Queue

```python
from collections import deque

class QueuedComponent:
    def __init__(self, node):
        self.node = node
        self._paused = False
        self._queue = deque(maxlen=100)  # Bounded
        
        # Standard API services...
        
        self.sub = node.create_subscription(
            Image, "/camera", self.on_image
        )
    
    def on_image(self, msg):
        if self._paused:
            self._queue.append(msg)  # Buffer
            return
        
        self.process(msg)
    
    def _handle_resume(self, request, response):
        self._paused = False
        
        # Process queued messages
        while self._queue:
            msg = self._queue.popleft()
            self.process(msg)
        
        response.success = True
        response.message = f"Resumed (processed {len(self._queue)} queued)"
        return response
```

**Pros**: Doesn't lose recent messages  
**Cons**: May flood on resume, uses memory

---

### Strategy 4: Subscription-Level Pause (Cleanest)

```python
class SubscriptionPauseComponent:
    def __init__(self, node):
        self.node = node
        self._paused = False
        
        # Standard API services...
        
        self.sub = node.create_subscription(
            Image, "/camera", self.on_image
        )
    
    def _handle_pause(self, request, response):
        self._paused = True
        self.sub.pause()  # Stop receiving messages
        response.success = True
        response.message = "Paused"
        return response
    
    def _handle_resume(self, request, response):
        self._paused = False
        self.sub.resume()  # Start receiving again
        response.success = True
        response.message = "Resumed"
        return response
```

**Pros**: Clean, no buffering issues  
**Cons**: Requires `Subscription.pause()` support (not yet implemented)

**Note**: This is the ideal future implementation once `Subscription.pause()` is added to the library.

---

## Optional Helper Mixin

For convenience, the library **MAY** provide an optional mixin:

```python
from ros2_zenoh_python.conventions import PausableServiceMixin

class MyComponent(PausableServiceMixin):
    def __init__(self, node):
        super().__init__(node)  # Registers pause/resume services
        
        self.sub = node.create_subscription(
            Image, "/camera", self.on_image
        )
    
    def on_image(self, msg):
        if self.is_paused:
            return  # Drop (or implement custom strategy)
        
        self.process(msg)
```

**Key point**: The mixin is **optional**. Users can implement the API directly.

---

## Usage from ROS2

### Command Line

```bash
# Pause a component
ros2 service call /camera_processor/pause std_srvs/srv/Trigger

# Resume
ros2 service call /camera_processor/resume std_srvs/srv/Trigger

# Check response
ros2 service call /my_component/pause std_srvs/srv/Trigger "{}"
# Response:
# success: True
# message: 'Paused'
```

### From Python (ros2-zenoh)

```python
from ros2_zenoh_python import ZenohNode
from std_srvs.srv import Trigger

async with ZenohNode("controller") as node:
    # Create service client
    pause_client = node.create_client(Trigger, "/camera_processor/pause")
    
    # Wait for service
    await pause_client.wait_for_service(timeout=5.0)
    
    # Call pause
    response = await pause_client.call_async(Trigger.Request())
    
    if response.success:
        print(f"Paused: {response.message}")
    else:
        print(f"Failed to pause: {response.message}")
```

### From Python (rclpy)

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        
        self.pause_client = self.create_client(
            Trigger, "/camera_processor/pause"
        )
        
    async def pause_component(self):
        self.pause_client.wait_for_service(timeout_sec=5.0)
        
        request = Trigger.Request()
        future = self.pause_client.call_async(request)
        
        response = await future
        print(f"Paused: {response.success}")
```

---

## Testing

### Integration Test Example

```python
from ros2_zenoh_python.testing import MockNode
from std_srvs.srv import Trigger
import pytest

@pytest.mark.asyncio
async def test_pausable_component():
    """Test pause/resume functionality."""
    node = MockNode("test")
    component = MyComponent(node)
    
    # Initial state: active
    component.on_message(Message(data="msg1"))
    assert component.processed == ["msg1"]
    
    # Pause
    pause_req = Trigger.Request()
    pause_resp = component._handle_pause(pause_req, Trigger.Response())
    assert pause_resp.success
    
    # Messages dropped while paused
    component.on_message(Message(data="msg2"))
    assert component.processed == ["msg1"]  # msg2 dropped
    
    # Resume
    resume_req = Trigger.Request()
    resume_resp = component._handle_resume(resume_req, Trigger.Response())
    assert resume_resp.success
    
    # Messages processed after resume
    component.on_message(Message(data="msg3"))
    assert component.processed == ["msg1", "msg3"]
```

---

## Tooling Opportunities

### Generic Pause Tool

```bash
#!/bin/bash
# pause-component.sh
COMPONENT=$1
ros2 service call "${COMPONENT}/pause" std_srvs/srv/Trigger
```

### Dashboard Integration

```python
class ComponentDashboard:
    """GUI dashboard for pausing components."""
    
    async def discover_components(self):
        """Find all pausable components."""
        services = await self.node.get_service_names_and_types()
        return [
            name.replace("/pause", "")
            for name, types in services
            if name.endswith("/pause")
        ]
    
    async def pause_all(self):
        """Emergency pause all components."""
        components = await self.discover_components()
        for comp in components:
            await self.call_service(f"{comp}/pause")
```

### Test Fixture

```python
@pytest.fixture
async def paused_component(component):
    """Fixture that pauses component for test."""
    # Pause before test
    await component.pause()
    
    yield component
    
    # Resume after test
    await component.resume()
```

---

## Comparison with ROS2 Lifecycle

| Feature | ROS2 Lifecycle | Pausable Convention |
|---------|----------------|---------------------|
| **States** | 4+ states | Just paused/active |
| **Services** | Complex state machine | Two simple services |
| **Mandatory** | No | No |
| **Standardized** | Yes (rclcpp/rclpy) | Yes (convention) |
| **Complexity** | High | Low |
| **Use Case** | Full lifecycle management | Runtime pause/resume only |

**Pausable is**: Lightweight alternative for simple pause/resume needs.  
**ROS2 Lifecycle is**: Full system for complex initialization/teardown.

---

## Best Practices

### ✅ Do

```python
# Implement standard service names
node.create_service(Trigger, "~/pause", ...)
node.create_service(Trigger, "~/resume", ...)

# Make it idempotent
def pause(self):
    if self._paused:
        return  # Already paused, safe no-op
    self._paused = True

# Return informative messages
response.message = "Paused (3 messages buffered)"

# Choose appropriate buffering strategy
# - Drop for sensor data (latest matters)
# - Queue for commands (all matter)
# - Latest-only for status updates
```

### ❌ Don't

```python
# Don't use non-standard service names
node.create_service(Trigger, "~/stop", ...)  # Use "pause"!

# Don't pause forever
def pause(self):
    while self._paused:
        time.sleep(1)  # Wrong! This blocks the thread

# Don't persist state
def __init__(self):
    self._paused = load_from_file("paused.txt")  # Wrong!

# Don't buffer unbounded
self._queue = []  # Can grow forever!
# Use: self._queue = deque(maxlen=100)
```

---

## Future Considerations

### If Subscription.pause() is Added

Once `Subscription.pause()` is available in the library:

```python
class Component:
    def _handle_pause(self, request, response):
        for sub in self.subscriptions:
            sub.pause()  # Clean subscription-level pause
        response.success = True
        return response
```

This will be the **recommended implementation** as it's cleanest.

### Extended Status Service

If needed, a richer status service could be standardized:

```yaml
# custom_msgs/srv/ComponentStatus
---
bool is_paused
uint64 messages_buffered
float64 paused_duration
string pause_reason
```

But this is **not part of the current convention** - keep it simple for now.

---

## Summary

**What**: Standard `~/pause` and `~/resume` services using `std_srvs/srv/Trigger`

**Why**: Interoperability, discoverability, tooling

**How**: Implement services, choose buffering strategy, test thoroughly

**When**: Optional - implement when pause/resume is useful for your component

**Status**: Recommended convention (not required)

---

## Related

- **ADR-005**: [[ADR-005-No-Lifecycle-System]] - Why no built-in lifecycle
- **Patterns**: `05-Patterns/Pausable-Component.md` (implementation examples)
- **Tutorial**: `02-Tutorials/Advanced/Implementing-Pausable-Components.md`



