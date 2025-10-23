---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: adr
id: "adr-002-three-time-modes"
title: "ADR-002: Three Mutually Exclusive Time Modes"
summary: "Implement WALL_TIME, SIM_TIME_LIVE, and SIM_TIME_TEST as distinct, non-overlapping time management modes"

# ADR-specific fields
status: "accepted"
date: "2025-10-23"
deciders: ["ros2-zenoh-team"]
consulted: []
informed: []

# Optional metadata
tags: ["testing", "time", "simulation", "architecture"]
related_patterns: ["clock-injection", "testable-components"]
related_specs: ["Testing-Clock"]
supersedes: null
superseded_by: null

# Future-proofing
ros2_zenoh:
  languages: ["python"]
  components: ["testing", "core"]
  phase: "1-python-core"
---

# ADR-002: Three Mutually Exclusive Time Modes

## Status

**Accepted** (2025-10-23)

---

## Context

ros2-zenoh needs to support three different time scenarios:

1. **Production with real time**: Wall clock time (default)
2. **Production with simulator**: Time driven by `/clock` topic (Gazebo, Isaac Sim, etc.)
3. **Testing**: Manually controlled time for fast, deterministic tests

These scenarios have fundamentally different semantics:
- Real time cannot be controlled
- Simulator time jumps unpredictably (pause/resume, variable speed)
- Test time should advance instantly with zero wall-clock delay

### Problem

Many frameworks mix these concerns:
- "Fast mode" flags that change behavior
- Single `Clock` class trying to handle all cases
- Confusing APIs where some methods work in some modes but not others

**Example of problematic design:**
```python
clock = Clock()
clock.fast_mode = True  # What does this do exactly?
clock.use_sim_time = True  # Can both be true?
clock.advance_by(5.0)  # Does this work in all modes?
```

### Requirements

- **Clear semantics**: Each mode has well-defined behavior
- **No mode confusion**: Can't accidentally use test APIs in production
- **Type safety**: IDE should catch invalid usage
- **No flags**: Mode is explicit, not a runtime configuration
- **Mutually exclusive**: Only one mode active at a time

---

## Decision

**Implement three distinct, mutually exclusive time modes: `WALL_TIME`, `SIM_TIME_LIVE`, and `SIM_TIME_TEST`.**

Each mode:
- Is set at `Clock` initialization
- Cannot be changed after construction
- Has its own valid method set
- Raises errors if invalid methods are called

```python
class TimeMode(Enum):
    WALL_TIME = 1       # Real system time (production default)
    SIM_TIME_LIVE = 2   # External /clock updates (production with simulator)
    SIM_TIME_TEST = 3   # Manual control (testing only)
```

---

## Consequences

### Positive

- ✅ **No ambiguity**: Clear what each mode does
- ✅ **Type-safe**: Invalid method calls raise errors immediately
- ✅ **Self-documenting**: Code shows intent via mode selection
- ✅ **No runtime flags**: Simpler logic, no `if fast_mode:` branches
- ✅ **Testability**: Test mode is completely separate from production
- ✅ **Safety**: Can't accidentally call `advance_by()` in production

### Negative

- ⚠️ **Cannot switch modes**: Must create new `Clock` instance
  - This is actually a feature - prevents mode confusion
- ⚠️ **More complex initially**: Three modes vs one class
  - But simpler in practice - each mode is smaller

### Neutral

- 🔸 Different from ROS2's use_sim_time parameter (which is a boolean)
- 🔸 Requires explicit mode selection

---

## Alternatives Considered

### Option 1: Single Clock with Flags

```python
clock = Clock(use_sim_time=True, fast_mode=True)
```

**Pros:**
- Familiar pattern
- One class to understand

**Cons:**
- Unclear semantics: What does `use_sim_time=True, fast_mode=True` mean?
- No type safety: Can call `advance_by()` even in wrong mode
- Confusing combinations: Which flags can be combined?
- Runtime branching: Every method needs `if self.fast_mode:`

**Why rejected:**
- Flags hide complexity rather than eliminating it
- Easy to misconfigure
- No compile-time guarantees

### Option 2: Inheritance Hierarchy

```python
class Clock: ...
class WallClock(Clock): ...
class SimClock(Clock): ...
class TestClock(Clock): ...
```

**Pros:**
- Type-safe: Different classes for different modes
- Clear separation

**Cons:**
- Harder to use: Must import correct class
- Code duplication for shared functionality
- Users must know which class to instantiate

**Why rejected:**
- Over-engineered for this use case
- Enum + mode checking is simpler
- Shared `now()` / `sleep()` interface would require abstract base class anyway

### Option 3: Two Modes (Production vs Test)

```python
class TimeMode(Enum):
    PRODUCTION = 1  # Handles both wall time and /clock
    TEST = 2
```

**Pros:**
- Simpler: Only two modes
- Production code doesn't care about wall vs sim time distinction

**Cons:**
- Hides important distinction between wall time and sim time
- Production code might need different logic for each
- Less clear what "PRODUCTION" means

**Why rejected:**
- Wall time and sim time have different behaviors (blocking on `/clock`)
- Being explicit prevents bugs

---

## References

- ROS2 Time and Clock design: https://design.ros2.org/articles/clock_and_time.html
- Original discussion: `TESTING_DESIGN_DECISIONS.md`
- Related spec: [[Testing-Clock]]

---

## Implementation Notes

### API Design

| Method | WALL_TIME | SIM_TIME_LIVE | SIM_TIME_TEST |
|--------|-----------|---------------|---------------|
| `now()` | ✅ `time.time()` | ✅ Latest `/clock` | ✅ Test time |
| `sleep(duration)` | ✅ Real sleep | ✅ Wait for `/clock` | ✅ Wait for advance |
| `set_time(t)` | ❌ Error | ❌ Error | ✅ Set test time |
| `advance_by(d)` | ❌ Error | ❌ Error | ✅ Advance time |
| `advance_to_next_event()` | ❌ Error | ❌ Error | ✅ Step through events |

### Usage Examples

**Production - Wall Time:**
```python
from ros2_zenoh_python.time import Clock, TimeMode

clock = Clock(TimeMode.WALL_TIME)

# Works
t = clock.now()  # Returns time.time()
await clock.sleep(1.0)  # Real 1-second delay

# Raises error
clock.advance_by(5.0)  # TimeModleError: advance_by() not available in WALL_TIME
```

**Production - Simulator:**
```python
# Clock subscribes to /clock topic
clock = Clock(TimeMode.SIM_TIME_LIVE, node=my_node)

# Works
t = clock.now()  # Returns latest /clock time
await clock.sleep(1.0)  # Blocks until /clock advances by 1.0s

# Raises error
clock.advance_by(5.0)  # TimeModeError
```

**Testing:**
```python
clock = Clock(TimeMode.SIM_TIME_TEST)
clock.set_time(100.0)

# Works
t = clock.now()  # Returns 100.0
await clock.advance_by(5.0)  # Processes events, advances to 105.0
await clock.set_time(200.0)  # Jump to 200.0

# sleep() waits for manual advancement
async def task():
    await clock.sleep(10.0)  # Blocks until time reaches 110.0
    print("Done!")

asyncio.create_task(task())
await clock.advance_by(10.0)  # Wakes up task, prints "Done!"
```

### Error Handling

```python
class TimeModeError(Exception):
    """Raised when method is called in wrong TimeMode."""
    pass

class Clock:
    def advance_by(self, duration: float) -> None:
        if self.mode != TimeMode.SIM_TIME_TEST:
            raise TimeModeError(
                f"advance_by() only available in SIM_TIME_TEST, "
                f"current mode: {self.mode}"
            )
        # ... implementation ...
```

### Migration Path

Existing code:
```python
# Old (if exists)
clock = Clock()
```

New code:
```python
# Explicit mode
clock = Clock(TimeMode.WALL_TIME)  # or TimeMode.SIM_TIME_TEST for tests
```

Default remains `WALL_TIME` for backward compatibility.

---

## Review History

| Date | Reviewer | Decision |
|------|----------|----------|
| 2025-10-23 | ros2-zenoh-team | Accepted |


