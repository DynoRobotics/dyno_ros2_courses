---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: spec
id: "spec-clock-python"
title: "Clock Component - Python Binding"
summary: "Python-specific API and implementation details for the Clock component"

# Spec-specific fields
status: "proposed"
version: "1.0.0"
component_type: "testing-utility"

# Metadata
tags: ["testing", "time", "simulation", "clock", "python"]
related_specs: ["Clock-Core"]
related_adrs: ["ADR-002"]

# Future-proofing
ros2_zenoh:
  languages: ["python"]
  components: ["testing"]
  phase: "1-python-core"
  test_coverage: "100%"
---

# Clock Component - Python Binding

**Python-specific implementation of [[Clock-Core]]**

For universal behavior specification, see **[[Clock-Core]]**.

---

## Python API

### Module

```python
from ros2_zenoh_python.testing import Clock, TimeMode, TimeModeError
```

### Class Definition

```python
class TimeMode(enum.Enum):
    """Time management modes."""
    WALL_TIME = "wall_time"
    SIM_TIME_LIVE = "sim_time_live"
    SIM_TIME_TEST = "sim_time_test"


class TimeModeError(Exception):
    """Raised when calling mode-restricted methods in wrong mode."""
    pass


class Clock:
    """
    Time management for production and testing.
    
    See docs/06-Specs/Core/Clock.md for complete behavior specification.
    """
    
    def __init__(
        self,
        mode: TimeMode = TimeMode.WALL_TIME,
        node: Optional[Node] = None,
        initial_time: float = 0.0
    ):
        """
        Initialize clock.
        
        Args:
            mode: Time management mode (cannot be changed after construction)
            node: Required for SIM_TIME_LIVE (subscribes to /clock)
            initial_time: Starting time for SIM_TIME_TEST (ignored in other modes)
            
        Raises:
            ValueError: If SIM_TIME_LIVE mode without node parameter
        """
        ...
    
    def now(self) -> float:
        """
        Get current time in seconds.
        
        Returns:
            Current time (UNIX timestamp for WALL_TIME, sim time for others)
        """
        ...
    
    async def sleep(self, duration: float) -> None:
        """
        Async sleep for duration seconds.
        
        Args:
            duration: Sleep duration in seconds (must be >= 0)
            
        Notes:
            - duration <= 0: Yields to event loop once
            - WALL_TIME: Real async sleep
            - SIM_TIME_LIVE: Waits for /clock to advance
            - SIM_TIME_TEST: Waits for set_time() or advance_by()
        """
        ...
    
    def set_time(self, t: float) -> None:
        """
        Set time directly (SIM_TIME_TEST only).
        
        Args:
            t: New time in seconds
            
        Raises:
            TimeModeError: If not in SIM_TIME_TEST mode
            ValueError: If t < current time (backwards)
        """
        ...
    
    async def advance_by(self, duration: float) -> None:
        """
        Advance time by duration, processing all intermediate events (SIM_TIME_TEST only).
        
        This is critical for tasks with multiple sleep() calls.
        
        Args:
            duration: Time to advance in seconds
            
        Raises:
            TimeModeError: If not in SIM_TIME_TEST mode
            ValueError: If duration < 0
        """
        ...
    
    async def advance_to_next_event(self) -> None:
        """
        Advance to next pending event (SIM_TIME_TEST only).
        
        Raises:
            TimeModeError: If not in SIM_TIME_TEST mode
            ValueError: If no pending events
        """
        ...
    
    async def process_pending(self) -> None:
        """
        Drain all ready tasks from asyncio event loop.
        
        Implementation: await asyncio.sleep(0) repeated 10 times
        """
        ...
```

---

## Implementation Details

### File Location

**Path**: `python/ros2_zenoh_python/ros2_zenoh_python/testing/clock.py`

### Dependencies

```python
import asyncio
import time
import enum
from typing import Optional, List, Tuple
from dataclasses import dataclass, field

from rosgraph_msgs.msg import Clock as ClockMsg
from ros2_zenoh_python.node import Node
```

### Internal State

```python
@dataclass
class _Sleeper:
    """Internal: Pending sleep() call."""
    wake_time: float
    future: asyncio.Future
```

```python
class Clock:
    def __init__(self, ...):
        self.mode: TimeMode = mode
        self.node: Optional[Node] = node
        
        # SIM_TIME_LIVE
        self._clock_time: float = 0.0
        self._clock_subscription: Optional[Subscription] = None
        
        # SIM_TIME_TEST
        self._current_time: float = initial_time
        self._sleepers: List[Tuple[float, asyncio.Future]] = []
```

### Platform-Specific Implementations

```python
def now(self) -> float:
    if self.mode == TimeMode.WALL_TIME:
        return time.time()  # Python standard library
    elif self.mode == TimeMode.SIM_TIME_LIVE:
        return self._clock_time
    else:  # SIM_TIME_TEST
        return self._current_time
```

```python
async def sleep(self, duration: float) -> None:
    if duration <= 0:
        await self.process_pending()
        return
    
    if self.mode == TimeMode.WALL_TIME:
        await asyncio.sleep(duration)  # Python asyncio
    # ... (rest as in Core spec)
```

```python
async def process_pending(self) -> None:
    """Yield to asyncio event loop."""
    for _ in range(10):
        await asyncio.sleep(0)
```

---

## Usage Examples

### Basic Usage

```python
from ros2_zenoh_python.testing import Clock, TimeMode
import asyncio

# Wall time (production)
clock = Clock(TimeMode.WALL_TIME)
print(clock.now())  # 1698765432.123
await clock.sleep(1.0)  # Real 1-second delay

# Test time (fast tests)
clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
print(clock.now())  # 0.0
await clock.advance_by(10.0)  # Instant
print(clock.now())  # 10.0
```

### With Simulator

```python
from ros2_zenoh_python import ZenohNode
from ros2_zenoh_python.testing import Clock, TimeMode

async with ZenohNode("my_robot") as node:
    # Simulator publishes to /clock
    clock = Clock(TimeMode.SIM_TIME_LIVE, node=node)
    
    print(clock.now())  # Simulator time
    await clock.sleep(1.0)  # Waits for sim to advance 1s
```

### Dependency Injection Pattern

```python
class NavigationComponent:
    def __init__(self, node: Node, clock: Clock):
        self.node = node
        self.clock = clock  # Injected
    
    async def navigate_to(self, target: Tuple[float, float]):
        while not self.at_target(target):
            self.move_forward()
            await self.clock.sleep(0.1)  # Uses injected clock

# Production
async with ZenohNode("nav") as node:
    clock = Clock(TimeMode.WALL_TIME)
    nav = NavigationComponent(node, clock)
    await nav.navigate_to((10, 20))

# Testing
async def test_navigation():
    from ros2_zenoh_python.testing import MockNode
    
    node = MockNode()
    clock = Clock(TimeMode.SIM_TIME_TEST)
    nav = NavigationComponent(node, clock)
    
    task = asyncio.create_task(nav.navigate_to((10, 20)))
    await clock.advance_by(10.0)  # Fast-forward
    
    assert nav.at_target((10, 20))
```

---

## Testing

### Pytest Fixtures

```python
# conftest.py
import pytest
from ros2_zenoh_python.testing import Clock, TimeMode

@pytest.fixture
def test_clock():
    """Fixture for test clock."""
    return Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)

@pytest.fixture
def wall_clock():
    """Fixture for wall clock."""
    return Clock(TimeMode.WALL_TIME)
```

### Unit Tests

```python
import pytest
import time
from ros2_zenoh_python.testing import Clock, TimeMode, TimeModeError

def test_wall_time_mode():
    """WALL_TIME delegates to system time."""
    clock = Clock(TimeMode.WALL_TIME)
    
    before = time.time()
    t = clock.now()
    after = time.time()
    
    assert before <= t <= after

@pytest.mark.asyncio
async def test_sim_time_test_set_time():
    """SIM_TIME_TEST allows manual time control."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=100.0)
    
    assert clock.now() == 100.0
    
    clock.set_time(200.0)
    assert clock.now() == 200.0

@pytest.mark.asyncio
async def test_advance_by_handles_multiple_sleeps():
    """advance_by() processes intermediate events."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    events = []
    
    async def task():
        await clock.sleep(3.0)
        events.append("3s")
        await clock.sleep(5.0)  # Not registered until first sleep completes!
        events.append("8s")
    
    asyncio.create_task(task())
    await clock.advance_by(10.0)
    
    assert events == ["3s", "8s"]
    assert clock.now() == 10.0

def test_mode_errors():
    """Invalid method calls raise TimeModeError."""
    clock = Clock(TimeMode.WALL_TIME)
    
    with pytest.raises(TimeModeError, match="set_time.*not available in WALL_TIME"):
        clock.set_time(100.0)
```

---

## Best Practices

### ✅ Do

```python
# Inject Clock in __init__
class Component:
    def __init__(self, node: Node, clock: Clock):
        self.clock = clock

# Use clock.now() for timestamps
timestamp = self.clock.now()

# Use clock.sleep() for delays
await self.clock.sleep(1.0)
```

### ❌ Don't

```python
# Don't use time.time() directly
timestamp = time.time()  # Not testable!

# Don't use asyncio.sleep() directly
await asyncio.sleep(1.0)  # Can't fast-forward in tests!

# Don't create Clock inside component
class Component:
    def __init__(self):
        self.clock = Clock(TimeMode.WALL_TIME)  # Not testable!
```

---

## Performance Considerations

### Memory

- **WALL_TIME**: O(1) - no state
- **SIM_TIME_LIVE**: O(n) where n = concurrent sleepers
- **SIM_TIME_TEST**: O(n) where n = concurrent sleepers

### CPU

- `now()`: O(1) - always fast
- `sleep()`: O(log n) - heap insertion
- `advance_by()`: O(m * log n) where m = events

**Recommendation**: SIM_TIME_TEST can handle thousands of concurrent sleepers efficiently.

---

## Error Handling

### Common Errors

```python
# 1. Mode mismatch
clock = Clock(TimeMode.WALL_TIME)
clock.set_time(100.0)  # TimeModeError

# 2. Missing node for SIM_TIME_LIVE
clock = Clock(TimeMode.SIM_TIME_LIVE)  # ValueError

# 3. Backwards time
clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=100.0)
clock.set_time(50.0)  # ValueError

# 4. Negative duration
await clock.sleep(-1.0)  # Yields immediately (no error)
```

---

## Migration from time/asyncio

### Before

```python
import time
import asyncio

class OldComponent:
    async def run(self):
        start = time.time()
        await asyncio.sleep(1.0)
        elapsed = time.time() - start
```

### After

```python
from ros2_zenoh_python.testing import Clock

class NewComponent:
    def __init__(self, clock: Clock):
        self.clock = clock
    
    async def run(self):
        start = self.clock.now()
        await self.clock.sleep(1.0)
        elapsed = self.clock.now() - start
```

**Benefits:**
- Tests run instantly (no 1s delay)
- Deterministic timing
- Easy to test edge cases

---

## Type Hints

```python
from typing import Protocol

class ClockProtocol(Protocol):
    """Type hint for clock dependency."""
    
    def now(self) -> float: ...
    async def sleep(self, duration: float) -> None: ...

# Use in type hints
class Component:
    def __init__(self, clock: ClockProtocol):
        self.clock = clock
```

---

## Related Documentation

- **Core Specification**: [[Clock-Core]]
- **ADR**: [[ADR-002]] - Three Time Modes
- **Tutorial**: `02-Tutorials/Testing/Fast-Tests-With-Clock.md`
- **Pattern**: `04-How-To/Inject-Clock-For-Testability.md`

---

## Implementation Status

- [x] Core Clock class
- [x] TimeMode enum
- [x] WALL_TIME mode
- [x] SIM_TIME_TEST mode
- [ ] SIM_TIME_LIVE mode (blocked on /clock message generation)
- [ ] Unit tests (target: 100% coverage)
- [ ] Integration tests with real components
- [ ] Documentation and tutorials



