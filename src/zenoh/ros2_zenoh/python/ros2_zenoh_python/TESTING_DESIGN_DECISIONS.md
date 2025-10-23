# Testing Infrastructure Design Decisions

This document captures the design decisions made for the testing infrastructure, including simulation time, mocking, and architecture patterns.

## Table of Contents
1. [Key Design Questions](#key-design-questions)
2. [Mocking Strategy](#mocking-strategy)
3. [Time Management](#time-management)
4. [Subscription Depth Handling](#subscription-depth-handling)
5. [Architecture Patterns](#architecture-patterns)
6. [State Management](#state-management)
7. [Lifecycle Patterns](#lifecycle-patterns)

---

## Key Design Questions

### Q1: Mocking Strategy - unittest.mock or Custom Classes?

**Options considered:**
- a) Use `unittest.mock.Mock` with helper functions
- b) Create custom mock classes (MockPublisher, MockNode, etc.)
- c) Hybrid approach

**Decision: Custom mock classes (Option b)**

**Rationale:**
- Better type safety and IDE autocomplete
- Explicit and self-documenting
- Easier to extend with domain-specific methods (e.g., `inject_message()`, `get_published_messages()`)
- `unittest.mock.Mock` is too generic - loses clarity about what's being mocked
- Python developers benefit from explicit, typed interfaces

### Q2: Timer Implementation - ROS2-style or Pure Asyncio?

**Options considered:**
- a) Pure asyncio: `asyncio.create_task()` + `await clock.sleep()`
- b) ROS2-compatible Timer class using asyncio internally
- c) Both options available

**Decision: Pure asyncio (Option a)**

**Rationale:**
- This is a Python library - embrace Python idioms
- `asyncio.create_task()` + `await clock.sleep()` is simpler and more transparent
- One less abstraction to maintain and document
- Users already understand asyncio; don't need to learn another API
- Example pattern:
  ```python
  async def periodic_task():
      while True:
          await clock.sleep(1.0)
          callback()
  
  asyncio.create_task(periodic_task())
  ```

### Q3: Fast Mode Scope - What to Cover?

**Options considered:**
- a) Just time: advance sim time instantly, process all pending asyncio tasks
- b) Time + pub/sub: ensure all messages delivered before advancing
- c) Time + pub/sub + services/actions: complete processing across all types

**Decision: Time + asyncio task processing only (Option a)**

**Rationale:**
- Simple mental model: "advance time, drain asyncio queue, repeat"
- Handles 99% of cases (async functions, sleeps, background tasks)
- Message delivery ordering should be tested explicitly if it matters
- Avoids complex synchronization logic in the test framework
- Keep test framework simple; complexity belongs in tests themselves

### Q4: Integration Strategy

**Options considered:**
- a) Completely separate (opt-in for new tests)
- b) Replace existing test patterns gradually
- c) Integrate into core classes with `testing_mode` flag

**Decision: Completely separate, opt-in (Option a)**

**Rationale:**
- Don't touch working tests (risk of breakage)
- Keep production code clean (no `testing_mode` flags)
- Clear separation of concerns: production code vs test utilities
- Users can adopt incrementally
- Testing utilities in `ros2_zenoh_python.testing` - import only in tests

### Q5: Do We Need a Custom Executor?

**Decision: No - Trust Asyncio**

**Rationale:**
- Asyncio already is an executor - no need to wrap or replicate it
- Priority scheduling is an anti-pattern in async code
  - If callback order matters for correctness → race condition exists
  - Tests should use explicit synchronization (Events, Queues)
- Every abstraction is technical debt
- Asyncio semantics are well-understood by Python developers
- Tests should expose race conditions, not hide them

**What we DO provide:**
```python
async def process_pending(self):
    """Drain all ready tasks from asyncio event loop."""
    for _ in range(10):
        await asyncio.sleep(0)  # Yield to event loop
```

---

## Mocking Strategy

### Design Philosophy

Custom mock classes that are:
- **Type-safe**: Clear signatures, IDE support
- **Explicit**: You see exactly what's being mocked
- **Domain-specific**: Methods like `inject_message()`, `get_published_messages()`
- **Testable**: Easy to verify interactions

### Core Mock Classes

```python
class MockPublisher:
    """Records all published messages."""
    def publish(self, msg)
    def get_latest() -> Optional[T]
    messages: list[T]

class MockSubscription:
    """Can inject messages to trigger callbacks."""
    async def inject_message(self, msg)

class MockNode:
    """Dependency injection for testing."""
    def create_publisher(...)
    def create_subscription(...)
    async def inject_to_subscription(topic, msg)
    def get_published_messages(topic) -> list
```

---

## Time Management

### Three Distinct, Mutually Exclusive Modes

```python
class TimeMode(Enum):
    WALL_TIME = 1       # Real system time (production default)
    SIM_TIME_LIVE = 2   # External /clock updates (production with simulator)
    SIM_TIME_TEST = 3   # Manual control (testing)
```

### Why Three Modes?

**Prevents confusion and interference:**
- Clear separation of production vs testing concerns
- Each mode has its own valid methods
- Errors if you try to use test methods in production mode
- No `fast_mode` flag or mixed semantics

### Mode Details

**WALL_TIME (Production Default)**
- Uses `time.time()` for `now()`
- Uses real `asyncio.sleep()` for delays
- No simulation time APIs available

**SIM_TIME_LIVE (Production with Simulator)**
- Subscribes to `/clock` topic (from Gazebo, etc.)
- Time advances when simulator publishes to `/clock`
- Blocks on clock updates for sleeps
- No manual time control

**SIM_TIME_TEST (Testing)**
- Manual time control via `set_time()`, `advance_by()`
- No /clock subscription
- Fast execution - no real delays

### API Design

| Method | WALL_TIME | SIM_TIME_LIVE | SIM_TIME_TEST |
|--------|-----------|---------------|---------------|
| `now()` | ✅ sys time | ✅ /clock time | ✅ test time |
| `sleep(duration)` | ✅ real sleep | ✅ waits for /clock | ✅ waits for set_time |
| `set_time(t)` | ❌ Error | ❌ Error | ✅ Manual control |
| `advance_by(d)` | ❌ Error | ❌ Error | ✅ Auto-advance |
| `advance_until(t)` | ❌ Error | ❌ Error | ✅ Jump to time |

### Time Advancement - Handling Multiple Events

**Problem:**
```python
async def task():
    await clock.sleep(3.0)  # First sleep
    print("After 3s")
    await clock.sleep(5.0)  # Second sleep (not registered yet!)
    print("After 8s")
```

If we just `clock.set_time(10.0)`, only the first sleep wakes up!

**Solution: `advance_by()` processes intermediate events**

```python
async def advance_by(self, duration: float):
    """Advance time by duration, processing ALL intermediate events."""
    target_time = self._current_time + duration
    
    while self._sleepers:
        next_wake = min(t for t, _ in self._sleepers)
        if next_wake > target_time:
            break
        
        # Step to next event
        self.set_time(next_wake)
        await self._process_pending()  # Let task continue
    
    # Jump to final target
    self.set_time(target_time)
    await self._process_pending()
```

**Recommended API for tests:**
- `advance_by(duration)` - Most tests (automatic intermediate processing)
- `advance_to_next_event()` - Fine-grained control (one event at a time)
- `set_time()` - Low-level (use carefully)

### /clock Integration

**Production with Simulator (e.g., Gazebo):**
```python
# Clock subscribes to /clock topic
clock = Clock(TimeMode.SIM_TIME_LIVE, node=my_node)
# When simulator publishes time updates, clock advances
```

**Why not scale real-time delays?**
We do NOT do:
```python
scale = sim_time / wall_time
await asyncio.sleep(duration / scale)  # ❌ BAD
```

**Reasons:**
- Sim time can jump (simulator paused then resumed)
- Sim time can run at arbitrary speeds (0.1x to 100x)
- Can't reliably map sim time → wall time

**Instead:**
- Production: Block on `/clock` updates
- Testing: Manual control, zero real delays

---

## Subscription Depth Handling

### Problem: Should Drop Oldest or Newest?

**ROS2 behavior:** "Depth" means "keep the last N messages"
- Queue full + new message → drop **oldest** (keep newest)
- This is "ring buffer" semantics
- Makes sense for sensor data: fresh data > stale data

### Wrong Approach (Drop Newest):
```python
try:
    queue.put_nowait(message)
except QueueFull:
    logger.warning("Dropped NEW message")  # ❌ WRONG
```

### Correct Approach (Drop Oldest):

**Option 1: collections.deque (recommended)**
```python
from collections import deque

class Subscription:
    def __init__(self, depth=10):
        self._message_buffer = deque(maxlen=depth)  # Ring buffer
    
    def _zenoh_callback(self, sample):
        msg = deserialize(sample)
        self._message_buffer.append(msg)  # Auto-drops oldest if full
```

**Option 2: Manual eviction**
```python
if queue.full():
    queue.get_nowait()  # Remove oldest
queue.put_nowait(message)  # Add newest
```

### Testing with Depth

```python
class MockSubscription:
    def __init__(self, depth=None):
        if depth:
            self._buffer = deque(maxlen=depth)  # Ring buffer
        else:
            self._buffer = deque()  # Unlimited for most tests
```

**Most tests:** Use unlimited depth (default)
**Overflow tests:** Explicitly set depth and verify behavior

---

## Architecture Patterns

### Dependency Injection (Required)

❌ **Bad:**
```python
class MyNode:
    def __init__(self):
        self.node = Node('my_node')  # Can't test!
```

✅ **Good:**
```python
class MyNode:
    def __init__(self, node):
        self.node = node  # Can inject MockNode!
```

### Separation of Concerns

**Three layers:**
1. **Business Logic** - Pure functions, zero ROS dependencies
2. **Application Layer** - Thin ROS wrappers, pub/sub setup
3. **Main/Entry Point** - Creates real or mock nodes

Example:
```python
# Layer 1: Pure logic (easily tested)
class NavigationLogic:
    @staticmethod
    def compute_velocity(position_x: float, target_x: float) -> float:
        return 0.0 if position_x >= target_x else 1.0

# Layer 2: ROS wrapper
class NavigationNode:
    def __init__(self, node):
        self.node = node
        self.logic = NavigationLogic()
        self.sub = node.create_subscription(Odometry, '/odom', self.on_odom)
        self.pub = node.create_publisher(Twist, '/cmd_vel')
    
    def on_odom(self, msg):
        # Thin callback - just I/O
        velocity = self.logic.compute_velocity(msg.pose.pose.position.x, 10.0)
        self.pub.publish(Twist(linear=Vector3(x=velocity)))

# Layer 3: Main
async def main():
    async with Node('nav') as node:
        nav = NavigationNode(node)
        await node.spin()
```

### Composition Over Inheritance

❌ **Bad:**
```python
class MyNode(Node):  # Tight coupling
    pass
```

✅ **Good:**
```python
class MyComponent:
    def __init__(self, node):  # Composition
        self.node = node
```

### Clock Injection

❌ **Bad:**
```python
def should_process(self):
    return time.time() - self.last > 1.0  # Can't test!
```

✅ **Good:**
```python
def __init__(self, clock):
    self.clock = clock

def should_process(self):
    return self.clock.now() - self.last > 1.0  # Testable!
```

---

## State Management

### Small State (< 1KB): Immutable Dataclasses

```python
from dataclasses import dataclass, replace

@dataclass(frozen=True)
class RobotState:
    position: tuple[float, float, float]
    velocity: float
    mode: str
    last_update: float

# Update immutably
state = replace(state, velocity=1.5)
```

**Benefits:**
- No side effects
- Easy to test and debug
- Type-safe
- Clear state transitions

### Large Data (Images, Arrays): Separate Metadata from Data

**Problem with immutable large data:**
```python
@dataclass(frozen=True)
class SensorState:
    image: np.ndarray  # 6MB per frame!
    timestamp: float

# Copies 6MB on every update! 💥
state = replace(state, timestamp=new_time)
```

**Solution A: Mutable dataclass with read-only arrays**
```python
from dataclasses import dataclass

@dataclass
class SensorData:
    image: np.ndarray
    depth: np.ndarray
    
    def __post_init__(self):
        # Make arrays read-only
        self.image.flags.writeable = False
        self.depth.flags.writeable = False

@dataclass(frozen=True)
class SensorState:
    data: SensorData  # Reference, not copied!
    timestamp: float
```

**Solution B: Separate metadata from buffer**
```python
@dataclass(frozen=True)
class ImageMetadata:
    timestamp: float
    frame_id: str
    width: int
    height: int

class ImageBuffer:
    def __init__(self):
        self._current: Optional[np.ndarray] = None
    
    def get_readonly(self) -> np.ndarray:
        view = self._current.view()
        view.flags.writeable = False
        return view

class VisionComponent:
    def __init__(self):
        self.metadata = ImageMetadata(...)  # Immutable
        self.buffer = ImageBuffer()         # Mutable container
```

### Guidelines:

1. **Small state (< 1KB)**: Use `frozen=True` dataclasses
2. **Large data (> 1KB)**: Separate metadata (immutable) from data (mutable)
3. **Always**: Make numpy arrays read-only to prevent accidental modification

---

## Lifecycle Patterns

### Simple Pattern (Recommended for Most Cases)

```python
class Component:
    async def setup(self):
        """One-time initialization."""
        pass
    
    async def teardown(self):
        """Cleanup resources."""
        pass

# Usage
component = Component(node)
await component.setup()
try:
    # ... use component ...
finally:
    await component.teardown()
```

### Context Manager Pattern

```python
class Component:
    async def __aenter__(self):
        await self.setup()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.teardown()

# Usage
async with Component(node) as component:
    # ... use component ...
# Auto cleanup
```

### Full Lifecycle (Optional, for Complex Systems)

```python
from enum import Enum

class LifecycleState(Enum):
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4

class LifecycleComponent:
    def __init__(self, node):
        self.node = node
        self.state = LifecycleState.UNCONFIGURED
    
    async def configure(self):
        """Load config, setup resources."""
        assert self.state == LifecycleState.UNCONFIGURED
        # ... setup ...
        self.state = LifecycleState.INACTIVE
    
    async def activate(self):
        """Start processing."""
        assert self.state == LifecycleState.INACTIVE
        # ... start ...
        self.state = LifecycleState.ACTIVE
    
    async def deactivate(self):
        """Stop processing but keep resources."""
        assert self.state == LifecycleState.ACTIVE
        # ... pause ...
        self.state = LifecycleState.INACTIVE
    
    async def cleanup(self):
        """Release all resources."""
        self.state = LifecycleState.FINALIZED
```

**When to use full lifecycle:**
- Systems with pause/resume requirements
- Systems needing graceful degradation
- Complex multi-phase initialization

**Most cases:** Use simple `setup()`/`teardown()` or context managers

---

## Summary of Key Decisions

1. ✅ **Custom mock classes** - type-safe, explicit, domain-specific
2. ✅ **Pure asyncio** - no Timer class, embrace Python idioms
3. ✅ **Three time modes** - mutually exclusive, clear semantics
4. ✅ **advance_by()** - handles multiple events automatically
5. ✅ **Ring buffer semantics** - drop oldest, keep newest
6. ✅ **No custom executor** - trust asyncio
7. ✅ **Dependency injection** - always inject node/clock
8. ✅ **Separate logic from I/O** - pure functions + thin wrappers
9. ✅ **Immutable small state** - dataclasses with `frozen=True`
10. ✅ **Separate large data** - metadata immutable, buffers mutable
11. ✅ **Simple lifecycle** - `setup()`/`teardown()` for most cases
12. ✅ **Opt-in testing** - separate module, no production pollution

---

## Implementation Status

See `testing-infrastructure-plan.plan.md` for detailed implementation plan.

