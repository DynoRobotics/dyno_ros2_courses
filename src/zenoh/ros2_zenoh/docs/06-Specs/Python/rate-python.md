---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: spec
id: "spec-rate-python"
title: "Rate Component - Python Binding"
summary: "Python-specific API and implementation details for fixed-rate execution"

# Spec-specific fields
status: "proposed"
version: "1.0.0"
component_type: "utility"

# Metadata
tags: ["rate", "control-loop", "timing", "python"]
related_specs: ["Rate-Core", "Clock-Python"]
related_adrs: []

# Future-proofing
ros2_zenoh:
  languages: ["python"]
  components: ["core"]
  phase: "1.5-patterns"
  test_coverage: "100%"
---

# Rate Component - Python Binding

**Python-specific implementation of [[Rate-Core]]**

For universal behavior specification, see **[[Rate-Core]]**.

---

## Python API

### Module

```python
from ros2_zenoh_python import Rate, fixed_rate_loop, Tick, DeadlineExceededError
from ros2_zenoh_python.testing import Clock
```

### Class Definitions

#### Rate

```python
class Rate:
    """
    Maintain fixed execution rate.
    
    Example:
        rate = Rate(10.0, clock)  # 10 Hz
        while running:
            do_work()
            await rate.sleep()
    """
    
    def __init__(
        self,
        frequency: float,
        clock: Clock,
        warn_on_overrun: bool = True
    ):
        """
        Create rate object.
        
        Args:
            frequency: Target frequency in Hz (must be > 0)
            clock: Clock for time management
            warn_on_overrun: Log warning on deadline misses
            
        Raises:
            ValueError: If frequency <= 0
        """
        ...
    
    async def sleep(self) -> None:
        """
        Sleep remainder of period to maintain rate.
        
        If behind schedule:
        - Sets missed_deadline = True
        - Sets overrun to duration behind
        - Logs warning (if warn_on_overrun=True)
        - Yields to event loop (no sleep)
        
        If on schedule:
        - Sets missed_deadline = False
        - Sets overrun = 0
        - Sleeps remainder of period
        """
        ...
    
    @property
    def frequency(self) -> float:
        """Target frequency in Hz."""
        ...
    
    @property
    def period(self) -> float:
        """Period in seconds (1/frequency)."""
        ...
    
    @property
    def missed_deadline(self) -> bool:
        """True if last iteration missed deadline."""
        ...
    
    @property
    def overrun(self) -> float:
        """Duration of overrun in seconds (0 if on time)."""
        ...
```

#### Tick

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class Tick:
    """
    Timing information for a single iteration.
    
    Attributes:
        iteration: Iteration number (starts at 1)
        expected_time: When this tick should have occurred
        actual_time: When this tick actually occurred
        overrun: Positive if behind, 0 if on time
        missed_deadline: True if overrun > 0
    """
    iteration: int
    expected_time: float
    actual_time: float
    overrun: float
    missed_deadline: bool
```

#### fixed_rate_loop

```python
async def fixed_rate_loop(
    frequency: float,
    clock: Clock,
    warn_on_overrun: bool = True,
    max_overrun: Optional[float] = None
) -> AsyncIterator[Tick]:
    """
    Async iterator that yields at fixed rate.
    
    Args:
        frequency: Loop frequency in Hz
        clock: Clock for time management
        warn_on_overrun: Log warning on missed deadlines
        max_overrun: Raise exception if overrun exceeds this (optional)
    
    Yields:
        Tick: Information about current iteration
    
    Raises:
        DeadlineExceededError: If max_overrun is exceeded
        ValueError: If frequency <= 0
    
    Example:
        async for tick in fixed_rate_loop(10.0, clock):
            process()
            if tick.overrun > 0:
                print(f"Behind by {tick.overrun}s")
            if should_stop():
                break
    """
    ...
```

#### DeadlineExceededError

```python
class DeadlineExceededError(Exception):
    """Raised when deadline overrun exceeds max_overrun."""
    pass
```

---

## Implementation Details

### File Location

**Path**: `python/ros2_zenoh_python/ros2_zenoh_python/rate.py`

### Dependencies

```python
import asyncio
import logging
from typing import AsyncIterator, Optional
from dataclasses import dataclass

from ros2_zenoh_python.testing import Clock
```

### Internal Implementation

```python
class Rate:
    def __init__(
        self,
        frequency: float,
        clock: Clock,
        warn_on_overrun: bool = True
    ):
        if frequency <= 0:
            raise ValueError(f"Frequency must be > 0, got {frequency}")
        
        self._frequency = frequency
        self._period = 1.0 / frequency
        self._clock = clock
        self._warn_on_overrun = warn_on_overrun
        self._last_tick = clock.now()
        self._missed_deadline = False
        self._overrun = 0.0
        self._logger = logging.getLogger(__name__)
    
    async def sleep(self) -> None:
        now = self._clock.now()
        elapsed = now - self._last_tick
        remaining = self._period - elapsed
        
        if remaining > 0:
            # On schedule
            await self._clock.sleep(remaining)
            self._missed_deadline = False
            self._overrun = 0.0
        else:
            # Behind schedule
            self._missed_deadline = True
            self._overrun = -remaining
            
            if self._warn_on_overrun:
                self._logger.warning(
                    f"Rate {self._frequency} Hz: "
                    f"Missed deadline by {self._overrun * 1000:.1f}ms"
                )
            
            # Yield to event loop
            await self._clock.process_pending()
        
        # Update tick time
        self._last_tick = self._clock.now()
    
    @property
    def frequency(self) -> float:
        return self._frequency
    
    @property
    def period(self) -> float:
        return self._period
    
    @property
    def missed_deadline(self) -> bool:
        return self._missed_deadline
    
    @property
    def overrun(self) -> float:
        return self._overrun
```

```python
async def fixed_rate_loop(
    frequency: float,
    clock: Clock,
    warn_on_overrun: bool = True,
    max_overrun: Optional[float] = None
) -> AsyncIterator[Tick]:
    if frequency <= 0:
        raise ValueError(f"Frequency must be > 0, got {frequency}")
    
    logger = logging.getLogger(__name__)
    period = 1.0 / frequency
    iteration = 0
    expected_time = clock.now()
    
    while True:
        iteration += 1
        expected_time += period
        actual_time = clock.now()
        overrun = max(0.0, actual_time - expected_time)
        missed = overrun > 0
        
        # Create tick
        tick = Tick(
            iteration=iteration,
            expected_time=expected_time,
            actual_time=actual_time,
            overrun=overrun,
            missed_deadline=missed
        )
        
        # Yield to user
        yield tick
        
        # Handle overrun
        if missed:
            if warn_on_overrun:
                logger.warning(
                    f"Rate {frequency} Hz: "
                    f"Iteration {iteration} missed deadline by {overrun * 1000:.1f}ms"
                )
            
            if max_overrun is not None and overrun > max_overrun:
                raise DeadlineExceededError(
                    f"Overrun {overrun:.3f}s exceeds max {max_overrun:.3f}s"
                )
        
        # Sleep until next tick
        now = clock.now()
        remaining = expected_time - now
        
        if remaining > 0:
            await clock.sleep(remaining)
        else:
            await clock.process_pending()
```

---

## Usage Examples

### Basic Control Loop

```python
from ros2_zenoh_python import fixed_rate_loop
from ros2_zenoh_python.testing import Clock, TimeMode

async def control_loop():
    clock = Clock(TimeMode.WALL_TIME)
    
    async for tick in fixed_rate_loop(100.0, clock):  # 100 Hz
        # Read sensors
        state = read_sensors()
        
        # Compute control
        command = controller(state)
        
        # Actuate
        send_command(command)
        
        # Check timing
        if tick.overrun > 0.001:
            print(f"Control loop slow: {tick.overrun * 1000:.1f}ms behind")
        
        if should_stop():
            break
```

### Explicit Rate Object

```python
from ros2_zenoh_python import Rate

async def heartbeat():
    clock = Clock(TimeMode.WALL_TIME)
    rate = Rate(1.0, clock)  # 1 Hz
    
    while running:
        send_heartbeat()
        
        if rate.missed_deadline:
            print(f"Heartbeat delayed by {rate.overrun:.3f}s")
        
        await rate.sleep()
```

### Hard Deadline Enforcement

```python
from ros2_zenoh_python import fixed_rate_loop, DeadlineExceededError

async def real_time_task():
    clock = Clock(TimeMode.WALL_TIME)
    
    try:
        async for tick in fixed_rate_loop(1000.0, clock, max_overrun=0.005):
            critical_processing()
    except DeadlineExceededError as e:
        print(f"Real-time constraint violated: {e}")
        emergency_stop()
```

### Testing with Fast-Forward

```python
from ros2_zenoh_python.testing import Clock, TimeMode
import asyncio

async def test_control_loop():
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    iterations = []
    
    async def loop():
        async for tick in fixed_rate_loop(10.0, clock):
            iterations.append(tick.iteration)
            if tick.iteration >= 100:
                break
    
    # Run loop
    task = asyncio.create_task(loop())
    
    # Fast-forward (instant!)
    await clock.advance_by(10.0)
    
    # Verify
    assert len(iterations) == 100
    assert clock.now() == 10.0
```

### Combining with Pausable Pattern

```python
from ros2_zenoh_python import fixed_rate_loop

class PausableControlLoop:
    def __init__(self, clock: Clock):
        self.clock = clock
        self._paused = False
    
    async def run(self):
        async for tick in fixed_rate_loop(100.0, self.clock):
            if self._paused:
                continue  # Skip processing when paused
            
            process()
    
    def pause(self):
        self._paused = True
    
    def resume(self):
        self._paused = False
```

---

## Testing

### Unit Tests

```python
import pytest
from ros2_zenoh_python import Rate, fixed_rate_loop, DeadlineExceededError
from ros2_zenoh_python.testing import Clock, TimeMode
import asyncio

@pytest.mark.asyncio
async def test_rate_on_time():
    """Rate maintains schedule when processing is fast."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    rate = Rate(10.0, clock)  # 10 Hz = 0.1s period
    
    async def loop():
        for _ in range(10):
            # Fast processing (0.01s < 0.1s period)
            await clock.sleep(0.01)
            await rate.sleep()
    
    task = asyncio.create_task(loop())
    await clock.advance_by(1.0)
    
    assert clock.now() == 1.0
    assert not rate.missed_deadline
    assert rate.overrun == 0.0

@pytest.mark.asyncio
async def test_rate_deadline_miss():
    """Rate detects deadline misses."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    rate = Rate(10.0, clock, warn_on_overrun=False)  # Disable warnings
    
    async def loop():
        # Slow processing (0.15s > 0.1s period)
        await clock.sleep(0.15)
        await rate.sleep()
    
    task = asyncio.create_task(loop())
    await clock.advance_by(0.2)
    
    assert rate.missed_deadline
    assert rate.overrun > 0.04  # Missed by ~0.05s

@pytest.mark.asyncio
async def test_fixed_rate_loop_no_drift():
    """fixed_rate_loop prevents cumulative drift."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    
    async def loop():
        async for tick in fixed_rate_loop(10.0, clock):
            # Small processing delay each iteration
            await clock.sleep(0.001)
            if tick.iteration >= 100:
                break
    
    task = asyncio.create_task(loop())
    await clock.advance_by(10.0)
    
    # No drift: exactly 10.0s for 100 iterations
    assert clock.now() == 10.0

@pytest.mark.asyncio
async def test_max_overrun_enforcement():
    """fixed_rate_loop raises on excessive overrun."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    
    async def loop():
        async for tick in fixed_rate_loop(10.0, clock, max_overrun=0.01):
            # Very slow processing
            await clock.sleep(0.5)
    
    with pytest.raises(DeadlineExceededError):
        task = asyncio.create_task(loop())
        await clock.advance_by(1.0)

@pytest.mark.asyncio
async def test_tick_information():
    """Tick provides accurate timing information."""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    ticks = []
    
    async def loop():
        async for tick in fixed_rate_loop(10.0, clock):
            ticks.append(tick)
            if tick.iteration >= 5:
                break
    
    task = asyncio.create_task(loop())
    await clock.advance_by(0.5)
    
    assert len(ticks) == 5
    assert ticks[0].iteration == 1
    assert ticks[0].expected_time == 0.1
    assert ticks[4].iteration == 5
    assert ticks[4].expected_time == 0.5
```

### Performance Tests

```python
@pytest.mark.benchmark
async def test_rate_overhead():
    """Rate has minimal overhead."""
    clock = Clock(TimeMode.WALL_TIME)
    rate = Rate(1000.0, clock)  # 1000 Hz
    
    start = clock.now()
    for _ in range(1000):
        await rate.sleep()
    duration = clock.now() - start
    
    # Should take ~1 second with minimal overhead
    assert 1.0 <= duration <= 1.1
```

---

## Best Practices

### ✅ Do

```python
# Inject Clock for testability
class ControlLoop:
    def __init__(self, clock: Clock):
        self.clock = clock
    
    async def run(self):
        async for tick in fixed_rate_loop(100.0, self.clock):
            self.control_step()

# Check for deadline misses in critical loops
async for tick in fixed_rate_loop(1000.0, clock):
    critical_task()
    if tick.overrun > 0.001:
        log.error("Real-time constraint violated!")

# Use SIM_TIME_TEST for fast tests
async def test_loop():
    clock = Clock(TimeMode.SIM_TIME_TEST)
    # Test runs instantly
```

### ❌ Don't

```python
# Don't use asyncio.sleep directly
while True:
    process()
    await asyncio.sleep(0.1)  # Not testable, drifts over time!

# Don't ignore deadline misses in critical loops
async for tick in fixed_rate_loop(1000.0, clock):
    critical_task()
    # No check for tick.overrun - could be dangerously slow!

# Don't queue missed iterations
if behind_schedule:
    for i in range(missed_count):
        process_old_data()  # Wrong! Skip old data instead
```

---

## Type Hints

```python
from typing import Protocol

class ClockProtocol(Protocol):
    """Type hint for clock dependency."""
    def now(self) -> float: ...
    async def sleep(self, duration: float) -> None: ...
    async def process_pending(self) -> None: ...

# Use in type hints
async def control_loop(clock: ClockProtocol) -> None:
    async for tick in fixed_rate_loop(100.0, clock):
        process()
```

---

## Related Documentation

- **Core Specification**: [[Rate-Core]]
- **Clock**: [[Clock-Core]], [[Clock-Python]]
- **Tutorial**: `02-Tutorials/Patterns/Fixed-Rate-Loops.md`
- **Pattern**: `04-How-To/Write-Control-Loop.md`

---

## Implementation Status

- [ ] Rate class
- [ ] fixed_rate_loop function
- [ ] Tick dataclass
- [ ] DeadlineExceededError
- [ ] Unit tests (target: 100% coverage)
- [ ] Performance benchmarks
- [ ] Documentation and tutorials



