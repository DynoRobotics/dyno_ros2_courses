---
id: index:ubiquitous-language
title: Ubiquitous Language
type: reference
category: index
summary: Domain terminology for ros2-zenoh ecosystem
updated: 2025-10-23
---

# Ubiquitous Language

**Purpose:** Single source of truth for domain terminology across all implementations (Python, Rust, C, TypeScript)

---

## Core Abstractions

### Node
**Definition:** Container for publishers, subscribers, services, and actions  
**Bounded Context:** ros2-zenoh runtime  
**Lifecycle:** Created → Setup → Running → Teardown → Destroyed  
**Responsibilities:**
- Manage Zenoh session
- Create and destroy publishers/subscribers
- Coordinate resource cleanup

### Component
**Definition:** Business logic unit with inputs, outputs, and state  
**Bounded Context:** Application layer  
**Pattern:** Follows Component pattern with `setup()`/`teardown()`  
**Responsibilities:**
- Encapsulate domain logic
- Manage component-specific state
- React to inputs, produce outputs

### Clock
**Definition:** Time source for scheduling and delays  
**Bounded Context:** Time management  
**Types:**
- **WALL_TIME:** Real system time (`time.monotonic()`)
- **SIM_TIME_LIVE:** Subscribe to `/clock` topic
- **SIM_TIME_TEST:** Controlled time for testing (no wall time dependency)

**Invariant:** Modes are mutually exclusive

---

## Domain Model

### Entities
*(Objects with identity)*

#### Publisher
**Identity:** (Node, Topic, MessageType)  
**Responsibilities:**
- Serialize messages to CDR
- Publish to Zenoh key expression
- Track subscriber count via liveliness

#### Subscriber
**Identity:** (Node, Topic, MessageType, Callback)  
**Responsibilities:**
- Deserialize CDR messages
- Buffer messages (ring buffer semantics)
- Invoke callback on message receipt

#### ServiceClient
**Identity:** (Node, ServiceName, ServiceType)  
**Responsibilities:**
- Send requests (async)
- Receive responses
- Track service server availability via liveliness

#### ServiceServer
**Identity:** (Node, ServiceName, ServiceType, Handler)  
**Responsibilities:**
- Receive requests
- Invoke handler (async)
- Send responses

#### ActionClient
**Identity:** (Node, ActionName, ActionType)  
**Responsibilities:**
- Send goals
- Receive feedback
- Cancel goals
- Receive results

#### ActionServer
**Identity:** (Node, ActionName, ActionType, ExecuteCallback)  
**Responsibilities:**
- Accept/reject goals
- Execute goals (async)
- Publish feedback
- Handle cancellation
- Return results

---

### Value Objects
*(Immutable descriptors)*

#### TopicName
**Type:** String  
**Format:** Fully-qualified ROS2 topic name (`/namespace/topic_name`)  
**Validation:** Must start with `/`, no trailing `/`

#### MessageType
**Type:** String  
**Format:** ROS2 interface type (`package_name/MessageName`)  
**Examples:** `std_msgs/String`, `geometry_msgs/Twist`

#### TypeHash
**Type:** Bytes (32 bytes)  
**Format:** RIHS01 type hash for ROS2 interoperability  
**Purpose:** Enable dynamic type checking

#### TimeStamp
**Type:** Immutable (seconds + nanoseconds)  
**Range:** ROS2 time (can represent both wall and sim time)

#### Duration
**Type:** Immutable (seconds)  
**Operations:** Add, subtract, compare

#### QoS
**Type:** Immutable settings  
**Fields:**
- `reliability`: RELIABLE | BEST_EFFORT
- `durability`: VOLATILE | TRANSIENT_LOCAL
- `history`: KEEP_LAST(depth) | KEEP_ALL
- `deadline`: Duration
- `lifespan`: Duration
- `liveliness`: AUTOMATIC | MANUAL_BY_TOPIC

---

### Aggregates
*(Consistency boundaries)*

#### NodeState
**Root:** Node  
**Members:** All publishers, subscribers, clients, servers, actions  
**Invariant:** All members destroyed when node destroyed  
**Operations:** Atomic creation/destruction

#### ComponentState
**Root:** Component  
**Members:** Business logic state (frozen dataclass)  
**Invariant:** State transitions only through defined logic  
**Immutability:** Use `@dataclass(frozen=True)` for state snapshots

#### MessageBuffer
**Root:** Subscription  
**Members:** deque of messages  
**Invariant:** Length ≤ depth (ring buffer - drop oldest)  
**Operations:** Append (auto-drops oldest), pop

---

## Events
*(Domain events - things that happened)*

### MessageReceived
**When:** Message arrived on subscription  
**Data:** Topic, message content, timestamp  
**Triggers:** Callback invocation

### ServiceRequested
**When:** Service call received  
**Data:** Service name, request content, request ID  
**Triggers:** Handler invocation

### GoalReceived
**When:** Action goal received  
**Data:** Action name, goal content, goal ID  
**Triggers:** Accept/reject decision, execution

### GoalCancelled
**When:** Cancel request received  
**Data:** Goal ID  
**Triggers:** Cancellation handling in execute callback

### TimeAdvanced
**When:** Simulation time changed (TEST mode only)  
**Data:** Old time, new time, delta  
**Triggers:** Process pending sleeps/timers

### PublisherMatched / SubscriberMatched
**When:** Liveliness token detected  
**Data:** Topic, remote node info  
**Triggers:** Update matched counts

---

## Business Rules
*(Invariants and constraints)*

### Message Ordering
**Rule:** FIFO within single topic from single publisher  
**Scope:** Best-effort (Zenoh doesn't guarantee global ordering)  
**Exception:** Network re-ordering possible

### Depth Enforcement
**Rule:** When buffer full, drop **oldest** message (ring buffer)  
**Rationale:** Subscriber always sees most recent data  
**ROS2 Compatibility:** Matches ROS2 DDS behavior

### Type Safety
**Rule:** CDR encoding/decoding enforces type consistency  
**Validation:** TypeHash comparison for rmw_zenoh interop  
**Error:** Deserialization failure if types mismatch

### Simulation Time Isolation
**Rule:** TEST mode never affects WALL_TIME or SIM_TIME_LIVE  
**Purpose:** Enable parallel testing without interference  
**Enforcement:** TimeMode is immutable per Clock instance

### Callback Execution
**Rule:** Callbacks execute in asyncio tasks (concurrent if async)  
**Exception:** User responsible for thread safety in callback  
**Best Practice:** Keep callbacks fast, offload heavy work

### Resource Cleanup
**Rule:** Node context manager ensures cleanup (`async with`)  
**Order:** Destroy publishers/subscribers before closing session  
**Failure:** Log error but continue cleanup

### Discovery
**Rule:** Publishers/subscribers use liveliness tokens for discovery  
**Format:** `@ros2_lv/<namespace>/MP/<topic>/<type>/<hash>` (publication)  
**Format:** `@ros2_lv/<namespace>/MS/<topic>/<type>/<hash>` (subscription)

---

## Bounded Contexts

### Application Layer
**Concerns:** Business logic, domain models, components  
**Language:** Component, State, Input, Output, Logic  
**Dependencies:** Uses runtime layer (Node, Publisher, etc.)

### Runtime Layer (ros2-zenoh)
**Concerns:** ROS2 abstractions over Zenoh  
**Language:** Node, Publisher, Subscriber, Service, Action  
**Dependencies:** Transport layer (Zenoh)

### Transport Layer (Zenoh)
**Concerns:** Low-level pub/sub, queries, liveliness  
**Language:** Session, Publisher, Subscriber, Liveliness Token  
**Dependencies:** Network (UDP/TCP)

### Time Management
**Concerns:** Wall time, simulation time, testing time  
**Language:** Clock, TimeMode, Duration, Timer  
**Cross-cutting:** Used by all layers

### Testing
**Concerns:** Mocks, fixtures, deterministic testing  
**Language:** MockNode, Clock (TEST mode), advance_by()  
**Cross-cutting:** Replaces runtime layer for tests

---

## Terminology Map

### ROS2 → ros2-zenoh

| ROS2 Term | ros2-zenoh Term | Notes |
|-----------|-----------------|-------|
| `Node` | `Node` | Same concept |
| `Publisher` | `Publisher` | Same API surface |
| `Subscription` | `Subscription` | More explicit async API |
| `Client` | `ServiceClient` | More explicit name |
| `Service` | `ServiceServer` | More explicit name |
| `ActionClient` | `ActionClient` | Same |
| `ActionServer` | `ActionServer` | Different callback model |
| `Timer` | `Clock.sleep()` + `asyncio.create_task()` | More Pythonic |
| `Executor` | `asyncio` event loop | Native async/await |
| `QoS` | `QoS` | Subset of ROS2 QoS |

### Zenoh → ros2-zenoh

| Zenoh Term | ros2-zenoh Term | Notes |
|------------|-----------------|-------|
| `Session` | Hidden in `Node` | Managed internally |
| `Publisher` | Wrapped in `Publisher` | CDR encoding added |
| `Subscriber` | Wrapped in `Subscription` | CDR decoding + buffering |
| `Queryable` | `ServiceServer` | Request/response abstraction |
| `Get` | `ServiceClient.call_async()` | Request/response abstraction |
| `Liveliness Token` | Discovery mechanism | Hidden from user |

---

## Anti-Patterns

### ❌ Mixing Time Modes
**Don't:** Use multiple Clock instances with different modes in same test  
**Why:** Breaks isolation, unpredictable behavior  
**Instead:** Use one Clock per test, pass via dependency injection

### ❌ Static Sleeps in Library Code
**Don't:** `await asyncio.sleep(5.0)` for waiting  
**Why:** Not deterministic, slow tests  
**Instead:** Use `Clock.sleep()` or active waiting (`wait_for_*()`)

### ❌ Mutable State in Messages
**Don't:** Modify message after publishing  
**Why:** Shared references can cause race conditions  
**Instead:** Treat messages as immutable after creation

### ❌ Blocking Callbacks
**Don't:** Run long computations in subscriber callback  
**Why:** Blocks asyncio loop, delays other callbacks  
**Instead:** Offload work to `asyncio.create_task()` or executor

### ❌ Manual Resource Management
**Don't:** Manually call `node.destroy()` without context manager  
**Why:** Easy to forget, leaks resources  
**Instead:** Always use `async with Node(...) as node:`

---

## Usage Examples

### Creating a Node
```python
from ros2_zenoh_python import Node

async with Node("my_node", namespace="/my_ns") as node:
    # Use node
    pass  # Automatic cleanup
```

### Publishing
```python
pub = node.create_publisher(String, "/topic", depth=10)
await pub.wait_for_subscribers()  # Active waiting
pub.publish(String(data="hello"))
```

### Subscribing
```python
async def callback(msg: String):
    print(f"Received: {msg.data}")

sub = node.create_subscription(String, "/topic", callback, depth=10)
# Callback invoked automatically
```

### Testing with Mocks
```python
from ros2_zenoh_python.testing import MockNode, Clock, TimeMode

async def test_my_component():
    clock = Clock(TimeMode.TEST)
    async with MockNode("test_node", clock=clock) as node:
        pub = node.create_publisher(String, "/topic")
        sub = node.create_subscription(String, "/topic", callback)
        
        pub.publish(String(data="test"))
        clock.advance_by(Duration(seconds=0.1))  # Process messages
        
        # Verify callback invoked
```

---

**Document Status:** Living document - evolves with implementation  
**Last Updated:** 2025-10-23  
**Next Review:** After Phase 1 completion

