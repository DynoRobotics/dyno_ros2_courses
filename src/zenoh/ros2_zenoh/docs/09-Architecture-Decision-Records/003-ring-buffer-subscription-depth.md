---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: adr
id: "adr-003-ring-buffer-subscription-depth"
title: "ADR-003: Use Ring Buffer Semantics for Subscription Depth"
summary: "Drop oldest messages when subscription queue is full, matching ROS2 DDS behavior"

# ADR-specific fields
status: "accepted"
date: "2025-10-23"
deciders: ["ros2-zenoh-team"]
consulted: []
informed: []

# Optional metadata
tags: ["subscription", "messaging", "architecture", "ros2-compatibility"]
related_patterns: []
related_specs: ["Subscription"]
supersedes: null
superseded_by: null

# Future-proofing
ros2_zenoh:
  languages: ["python"]
  components: ["subscription"]
  phase: "1-python-core"
---

# ADR-003: Use Ring Buffer Semantics for Subscription Depth

## Status

**Accepted** (2025-10-23)

---

## Context

ROS2 subscriptions have a "depth" parameter that controls how many messages to buffer before processing. When a subscription receives messages faster than the callback can process them, we must decide what to do when the buffer is full.

### Two Overflow Strategies

1. **Drop newest** (reject incoming): Keep old messages, drop new ones
2. **Drop oldest** (ring buffer): Keep new messages, evict old ones

### ROS2 DDS Behavior

ROS2 with DDS uses **ring buffer semantics**: When the queue is full and a new message arrives, the oldest message is dropped and the newest is added.

**Rationale in ROS2:**
- Sensor data should be fresh (latest IMU reading > stale reading)
- Queue depth is about backpressure, not guaranteed delivery
- If callback is slow, better to skip old data than fall further behind

### Problem

Initial implementation used `asyncio.Queue` with drop-newest:
```python
try:
    queue.put_nowait(message)
except asyncio.QueueFull:
    logger.warning("Dropped NEW message")  # Wrong!
```

This doesn't match ROS2 behavior and can cause confusing bugs when porting from `rclpy`.

---

## Decision

**Use ring buffer semantics (drop oldest) for subscription depth, matching ROS2 DDS behavior.**

Implementation uses `collections.deque(maxlen=depth)`:
- Automatically drops oldest when full
- O(1) append and popleft operations  
- Simple, standard library solution

---

## Consequences

### Positive

- ✅ **ROS2 compatibility**: Matches DDS behavior exactly
- ✅ **Fresh data**: Applications always get recent messages
- ✅ **Simple**: `deque(maxlen)` handles eviction automatically
- ✅ **Predictable**: No surprising behavior differences from rclpy
- ✅ **Sensor-friendly**: Latest IMU/camera data > stale data

### Negative

- ⚠️ **Message loss**: Old messages are silently dropped
  - This is the same trade-off ROS2 makes
  - Solution: Increase depth or fix slow callback
- ⚠️ **No backpressure**: Publisher doesn't know messages were dropped
  - Again, matches ROS2 behavior
  - This is a limitation of pub/sub pattern generally

### Neutral

- 🔸 Different from some message queues (RabbitMQ, Kafka) that drop newest or block
- 🔸 Requires logging/monitoring to detect overflow

---

## Alternatives Considered

### Option 1: Drop Newest (Keep Oldest)

```python
try:
    queue.put_nowait(message)
except QueueFull:
    logger.warning("Queue full, dropping new message")
```

**Pros:**
- Simpler with `asyncio.Queue`
- No data loss for messages already in queue

**Cons:**
- **Doesn't match ROS2**: Confusing for users porting from rclpy
- **Stale data**: Callback processes old messages while new ones are dropped
- **Backlog problem**: Slow callback gets further behind

**Why rejected:**
- Incompatible with ROS2 semantics
- Leads to processing stale data

### Option 2: Block Publisher

```python
await queue.put(message)  # Block until space available
```

**Pros:**
- No message loss
- Natural backpressure

**Cons:**
- **Not pub/sub**: Publishers shouldn't block on slow subscribers
- **Deadlock risk**: Can freeze publisher thread
- **Not ROS2-compatible**: DDS never blocks publishers

**Why rejected:**
- Violates pub/sub pattern
- Doesn't match ROS2

### Option 3: Unlimited Queue

```python
queue = deque()  # No maxlen
```

**Pros:**
- Never drops messages
- Simple

**Cons:**
- **Memory leak**: Slow callback → unbounded growth
- **OOM crashes**: Eventually exhausts memory
- **Not ROS2-compatible**: Depth parameter exists for a reason

**Why rejected:**
- Dangerous in production
- Defeats purpose of depth parameter

### Option 4: Configurable Strategy

```python
subscription = node.create_subscription(
    msg_type, topic, callback,
    overflow_strategy="drop_oldest"  # or "drop_newest", "block"
)
```

**Pros:**
- Flexible
- Supports different use cases

**Cons:**
- **Complexity**: More code to maintain
- **Fragmentation**: Different apps behave differently
- **Not ROS2-compatible**: ROS2 has one behavior

**Why rejected:**
- YAGNI: We haven't identified use cases for drop-newest
- Simplicity beats flexibility here
- Match ROS2, don't invent new semantics

---

## References

- ROS2 QoS documentation: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
- Python `collections.deque`: https://docs.python.org/3/library/collections.html#collections.deque
- Original discussion: `TESTING_DESIGN_DECISIONS.md`

---

## Implementation Notes

### Production Code

```python
from collections import deque

class Subscription:
    def __init__(
        self,
        msg_type: type[T],
        topic: str,
        callback: Callable[[T], None],
        depth: int = 10,
    ):
        self._message_buffer = deque(maxlen=depth)  # Ring buffer!
        self._message_event = asyncio.Event()
        # ...
    
    def _zenoh_callback(self, sample: zenoh.Sample):
        """Called by Zenoh when message arrives."""
        msg = self._deserialize(sample)
        
        # deque automatically drops oldest if full
        self._message_buffer.append(msg)
        
        # Log if we're at capacity (optional)
        if len(self._message_buffer) == self._message_buffer.maxlen:
            self._logger.debug(
                f"Subscription {self.topic} at full capacity ({self._message_buffer.maxlen})"
            )
        
        self._message_event.set()
    
    async def _process_messages(self):
        """Background task that drains buffer and calls callback."""
        while True:
            await self._message_event.wait()
            
            while self._message_buffer:
                msg = self._message_buffer.popleft()
                
                # Call user callback
                if asyncio.iscoroutinefunction(self.callback):
                    await self.callback(msg)
                else:
                    self.callback(msg)
            
            self._message_event.clear()
```

### Testing

```python
async def test_subscription_depth_overflow():
    """Verify ring buffer semantics."""
    received = []
    
    def callback(msg):
        received.append(msg.data)
    
    # Small depth for testing
    sub = Subscription(String, "/test", callback, depth=3)
    
    # Inject 5 messages rapidly (more than depth)
    for i in range(5):
        sub._zenoh_callback(create_sample(f"msg_{i}"))
    
    # Process messages
    await sub._process_messages()
    
    # Should only receive last 3 (oldest 2 dropped)
    assert received == ["msg_2", "msg_3", "msg_4"]
```

### Migration Notes

No migration needed - this has always been the intended behavior, just not correctly implemented initially.

### Depth Selection Guidelines

For users:

- **High-frequency sensors** (IMU, cameras): depth=5-10
  - If callback can't keep up, want latest data
  
- **Low-frequency commands**: depth=1-2
  - Usually want latest command, not backlog
  
- **Event streams**: depth=100+
  - If order and completeness matter
  - But consider using services/actions instead

- **Default**: depth=10
  - Good balance for most cases

### Monitoring Overflow

```python
class Subscription:
    def __init__(self, ...):
        self._overflow_count = 0
    
    def _zenoh_callback(self, sample):
        was_full = len(self._message_buffer) == self._message_buffer.maxlen
        self._message_buffer.append(msg)
        
        if was_full:
            self._overflow_count += 1
            if self._overflow_count % 100 == 0:
                self._logger.warning(
                    f"{self.topic}: Dropped {self._overflow_count} messages. "
                    f"Consider increasing depth or optimizing callback."
                )
```

---

## Review History

| Date | Reviewer | Decision |
|------|----------|----------|
| 2025-10-23 | ros2-zenoh-team | Accepted |


