---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: adr
id: "adr-005-no-lifecycle-system"
title: "ADR-005: No Built-in Lifecycle System for v1.0"
summary: "Rely on async patterns and best practices instead of implementing a formal lifecycle system"

# ADR-specific fields
status: "accepted"
date: "2025-10-23"
deciders: ["ros2-zenoh-team"]
consulted: []
informed: []

# Optional metadata
tags: ["architecture", "lifecycle", "initialization", "patterns"]
related_patterns: ["async-setup", "context-managers"]
related_specs: ["Core/Publisher", "Python/Publisher-Python"]
supersedes: null
superseded_by: null

# Future-proofing
ros2_zenoh:
  languages: ["python", "rust", "c", "typescript"]
  components: ["core"]
  phase: "1-python-core"
---

# ADR-005: No Built-in Lifecycle System for v1.0

## Status

**Accepted** (2025-10-23)

---

## Context

When creating ROS2 components that receive messages, there's a potential race condition:

```python
class MyComponent:
    def __init__(self, node):
        # Subscription created - callbacks can fire IMMEDIATELY
        self.sub = node.create_subscription(Data, "/input", self.on_data)
        
        # If message arrives here, self.state is None → crash!
        self.state = initialize_complex_state()
    
    def on_data(self, msg):
        self.state.process(msg)  # 💥 NoneType error if called during init
```

### Solutions Considered

1. **No lifecycle** - Document async patterns
2. **Simple enable/disable flag** - Component has `_active` flag
3. **Full ROS2 Lifecycle** - UNCONFIGURED → INACTIVE → ACTIVE → FINALIZED

### Requirements

- Prevent callback execution before component is ready
- Support clean shutdown (destroy subscriptions before state cleanup)
- Keep library simple (Priority 1: rmw_zenoh interop)
- Match rmw_zenoh behavior (which has no lifecycle)

---

## Decision

**Do NOT implement a lifecycle system in v1.0.**

Instead, rely on:
1. **Proper async initialization patterns** (`async def setup()`)
2. **Context managers** for automatic cleanup
3. **Documentation** of best practices
4. **Optional user-level patterns** (developers can add flags if needed)

---

## Consequences

### Positive

- ✅ **Simpler library**: Less abstraction, easier to understand
- ✅ **Less boilerplate**: No forced lifecycle methods
- ✅ **Matches rmw_zenoh**: No lifecycle requirement for interop
- ✅ **Pythonic**: Uses standard async/await patterns
- ✅ **Flexible**: Users can add lifecycle to their components if needed
- ✅ **Focus on core**: Don't over-engineer before knowing pain points

### Negative

- ⚠️ **Developer responsibility**: Must order initialization correctly
- ⚠️ **No library enforcement**: Possible to get ordering wrong
- ⚠️ **Documentation burden**: Must clearly explain patterns
- ⚠️ **Race condition potential**: If patterns not followed

### Neutral

- 🔸 Different from ROS2 Lifecycle (which is optional anyway)
- 🔸 May need to add lifecycle in v1.x if demand emerges

---

## Alternatives Considered

### Option 1: Simple Enable/Disable Flag

```python
class Subscription:
    def __init__(self, ...):
        self._active = False
        self._message_queue = []
    
    def _callback(self, msg):
        if not self._active:
            self._message_queue.append(msg)  # Queue until activated
            return
        self.user_callback(msg)
    
    def activate(self):
        self._active = True
        # Process queued messages
        for msg in self._message_queue:
            self.user_callback(msg)
        self._message_queue.clear()
```

**Pros:**
- Built-in safety
- Messages not dropped

**Cons:**
- Hidden complexity in library
- Memory growth if never activated
- Not clear when to activate

**Why rejected:**
- Adds complexity to core library
- Users can implement if needed
- Queuing semantics unclear (should we queue? drop? error?)

---

### Option 2: Full ROS2 Lifecycle

```python
class LifecycleNode:
    async def configure(self): ...  # Allocate resources
    async def activate(self): ...   # Start processing
    async def deactivate(self): ... # Pause processing
    async def cleanup(self): ...    # Release resources
```

**Pros:**
- Standardized ROS2 pattern
- Clear semantics
- Supports pause/resume

**Cons:**
- Complex for most use cases
- Lots of boilerplate
- rmw_zenoh doesn't require it
- Not all components need pause/resume

**Why rejected:**
- Over-engineering for v1.0
- Most components are simple
- Can add later if needed (as optional base class)

---

## Best Practices (To Document)

### Pattern 1: State Before Subscriptions (Recommended)

```python
class MyComponent:
    def __init__(self, node):
        self.node = node
        # Don't create subscriptions in __init__
    
    async def setup(self):
        """Initialize in correct order"""
        # 1. Initialize state FIRST
        self.state = await initialize_complex_state()
        
        # 2. Create subscriptions AFTER state is ready
        self.sub = self.node.create_subscription(Data, "/input", self.callback)
    
    async def teardown(self):
        """Clean shutdown"""
        # 1. Destroy subscriptions FIRST
        self.sub.destroy()
        
        # 2. Then cleanup state
        await self.state.cleanup()
    
    def callback(self, msg):
        self.state.process(msg)  # ✅ Safe, state always exists

# Usage
async with ZenohNode("my_node") as node:
    component = MyComponent(node)
    await component.setup()
    await node.spin()
# Auto-teardown on exit
```

### Pattern 2: Ready Flag (For Complex State)

```python
class StatefulComponent:
    def __init__(self, node):
        self.node = node
        self._ready = False
        
        # Can create subscriptions in __init__ if using ready flag
        self.sub = node.create_subscription(Data, "/input", self._callback)
    
    async def setup(self):
        # Complex async initialization
        self.database = await connect_to_database()
        self.ml_model = await load_model()
        
        # Mark ready AFTER everything initialized
        self._ready = True
    
    def _callback(self, msg):
        if not self._ready:
            logger.warning("Received message before ready, dropping")
            return
        
        self.callback(msg)  # Delegate to user code
    
    def callback(self, msg):
        # Safe to process
        self.process(msg)
```

### Pattern 3: Context Manager (Simplest)

```python
class SimpleComponent:
    def __init__(self, node):
        self.node = node
    
    async def __aenter__(self):
        # Setup
        self.state = ComplexState()
        self.sub = self.node.create_subscription(Data, "/input", self.callback)
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        # Cleanup
        self.sub.destroy()
        self.state.cleanup()
    
    def callback(self, msg):
        self.state.process(msg)

# Usage
async with SimpleComponent(node) as component:
    await node.spin()
# Auto-cleanup
```

---

## Implementation Guidelines

### Documentation Requirements

1. **Tutorial**: "Building Safe Components" guide
2. **Spec**: Add "Initialization Pattern" section to all component specs
3. **Examples**: Show proper patterns in all examples
4. **Warnings**: Document race conditions in API docs

### Code Review Checklist

When reviewing component code:
- [ ] State initialized before subscriptions created?
- [ ] Cleanup in reverse order (subs → state)?
- [ ] Context manager or explicit setup/teardown?
- [ ] Async initialization uses `async def setup()`?

---

## Pausable Component Convention (Approved for Phase 1.5)

While we don't implement a built-in lifecycle system, we **will define** a standard **Pausable Component API convention** for runtime control:

**Use Cases**:
- **Debugging**: Pause components selectively while debugging others
- **Testing**: Control timing, simulate failures
- **Reconfiguration**: Pause while updating parameters
- **Resource Management**: Temporarily disable expensive computations

**Standard API Convention**:

Every pausable component SHOULD implement:
- **`~/pause`** service (`std_srvs/srv/Trigger`) - Pause processing
- **`~/resume`** service (`std_srvs/srv/Trigger`) - Resume processing

**Usage from ROS2**:
```bash
ros2 service call /my_component/pause std_srvs/srv/Trigger
ros2 service call /my_component/resume std_srvs/srv/Trigger
```

**Implementation Strategy**: User's choice!
- **Drop messages**: Return early if paused (simple)
- **Process latest only**: Keep one message buffered
- **Queue with limit**: Bounded queue (e.g., deque(maxlen=100))
- **Subscription-level**: Actually pause the subscription (cleanest, if supported)

**Optional Helper**: `ros2_zenoh_python.conventions.PausableServiceMixin`
- Implements service boilerplate
- User just checks `self.is_paused` in callbacks
- Totally opt-in (users can implement directly)

**Key Differences from Lifecycle**:
- **Convention, not implementation mandate** (flexibility)
- **Simple API** (two services, not state machine)
- **Implementation freedom** (drop vs queue vs buffer)
- **Compatible** with rmw_zenoh (just services)

**See**: 
- `03-Conventions/Pausable-Component-API.md` (standard)
- `02-Tutorials/Advanced-Patterns/Pausable-Components.md` (implementation guide)

---

## Future Considerations

### If Full Lifecycle Becomes Necessary (v1.x+)

Add **optional** `LifecycleComponent` base class:

```python
from ros2_zenoh_python.lifecycle import LifecycleComponent

class MyComponent(LifecycleComponent):  # Optional!
    async def on_configure(self):
        """Override: Allocate resources"""
        self.sub = self.node.create_subscription(...)
    
    async def on_activate(self):
        """Override: Enable processing"""
        pass  # Default: start processing
    
    async def on_deactivate(self):
        """Override: Pause processing"""
        pass  # Default: stop processing
```

**Key**: Make it **optional**, not required. Simple components shouldn't need it.

### Indicators We Need Full Lifecycle

- [ ] Multiple user reports of initialization races
- [ ] Complex state machines become common (beyond pause/resume)
- [ ] Users writing lifecycle themselves repeatedly

**Until then**: YAGNI (You Aren't Gonna Need It)

---

## References

- ROS2 Lifecycle Design: https://design.ros2.org/articles/node_lifecycle.html
- Python async patterns: https://docs.python.org/3/library/asyncio.html
- Context managers: https://docs.python.org/3/reference/datamodel.html#context-managers

---

## Review History

| Date | Reviewer | Decision |
|------|----------|----------|
| 2025-10-23 | ros2-zenoh-team | Accepted |

