---
id: index:domain-model
title: Domain Model
type: reference
category: index
summary: High-level domain model for ros2-zenoh
updated: 2025-10-23
---

# Domain Model

**Purpose:** Visual representation of the ros2-zenoh architecture, showing bounded contexts, aggregates, and relationships.

See [[Ubiquitous-Language]] for terminology definitions.

---

## Bounded Contexts

```mermaid
graph TB
    subgraph "Application Layer"
        Component[Component<br/>Business Logic]
        State[Component State<br/>frozen dataclass]
    end
    
    subgraph "ros2-zenoh Runtime"
        Node[Node<br/>Resource Manager]
        Pub[Publisher]
        Sub[Subscription]
        Client[Service Client]
        Server[Service Server]
        AClient[Action Client]
        AServer[Action Server]
    end
    
    subgraph "Time Management"
        Clock[Clock<br/>TimeMode]
        Timer[Scheduled Events]
    end
    
    subgraph "Testing"
        MockNode[MockNode]
        MockPub[MockPublisher]
        MockSub[MockSubscriber]
    end
    
    subgraph "Transport Layer"
        Zenoh[Zenoh Session]
        Liveliness[Liveliness Tokens]
    end
    
    Component -->|uses| Node
    Component -->|injected with| Clock
    Component -->|has| State
    
    Node -->|creates| Pub
    Node -->|creates| Sub
    Node -->|creates| Client
    Node -->|creates| Server
    Node -->|creates| AClient
    Node -->|creates| AServer
    Node -->|uses| Clock
    
    Pub -->|publishes via| Zenoh
    Sub -->|subscribes via| Zenoh
    Client -->|queries via| Zenoh
    Server -->|queryable via| Zenoh
    
    Pub -->|advertises via| Liveliness
    Sub -->|discovers via| Liveliness
    
    MockNode -.->|replaces| Node
    MockPub -.->|replaces| Pub
    MockSub -.->|replaces| Sub
    
    Clock -->|schedules| Timer
```

---

## Core Aggregates

### 1. Node Aggregate

**Root Entity:** Node  
**Value Objects:** NodeName, Namespace  
**Collections:**
- publishers: Dict[TopicName, Publisher]
- subscriptions: Dict[TopicName, Subscription]
- service_clients: Dict[ServiceName, ServiceClient]
- service_servers: Dict[ServiceName, ServiceServer]
- action_clients: Dict[ActionName, ActionClient]
- action_servers: Dict[ActionName, ActionServer]

**Invariant:** All members destroyed when Node destroyed

```mermaid
classDiagram
    class Node {
        +name: str
        +namespace: str
        +zenoh_session: Session
        +create_publisher()
        +create_subscription()
        +create_service_client()
        +create_service_server()
        +create_action_client()
        +create_action_server()
        +destroy()
    }
    
    class Publisher {
        +topic: str
        +message_type: Type
        +publish(msg)
        +wait_for_subscribers()
    }
    
    class Subscription {
        +topic: str
        +message_type: Type
        +callback: Callable
        +depth: int
        +_buffer: deque
    }
    
    Node "1" --o "0..*" Publisher : creates
    Node "1" --o "0..*" Subscription : creates
```

### 2. Component Aggregate

**Root Entity:** Component  
**Value Objects:** ComponentState  
**Associations:**
- Inputs: List[Subscription]
- Outputs: List[Publisher]
- Clock: Clock (injected)

**Invariant:** State transitions only through defined business logic

```python
@dataclass(frozen=True)
class ComponentState:
    """Immutable state snapshot"""
    field1: int
    field2: str
    # ... more fields

class MyComponent:
    def __init__(self, node: Node, clock: Clock):
        self._node = node
        self._clock = clock
        self._state = ComponentState(field1=0, field2="")
    
    async def setup(self):
        """Setup I/O"""
        self._input_sub = self._node.create_subscription(...)
        self._output_pub = self._node.create_publisher(...)
    
    async def teardown(self):
        """Cleanup"""
        pass
    
    def _on_input(self, msg):
        """Update state (create new immutable state)"""
        new_state = ComponentState(
            field1=self._state.field1 + 1,
            field2=msg.data
        )
        self._state = new_state
        self._output_pub.publish(...)
```

### 3. MessageBuffer Aggregate

**Root Entity:** Subscription  
**Value Objects:** None (primitives)  
**Collection:** deque[Message] (maxlen=depth)

**Invariant:** Buffer length ≤ depth (ring buffer semantics)

```python
from collections import deque

class Subscription:
    def __init__(self, ..., depth: int = 10):
        self._buffer = deque(maxlen=depth)  # Auto-drops oldest
        self._message_event = asyncio.Event()
    
    def _on_zenoh_message(self, sample):
        msg = deserialize(sample.payload)
        self._buffer.append(msg)  # Drops oldest if full
        self._message_event.set()
    
    async def _process_messages(self):
        while True:
            await self._message_event.wait()
            while self._buffer:
                msg = self._buffer.popleft()
                await self._callback(msg)
            self._message_event.clear()
```

---

## Entity Relationships

```mermaid
erDiagram
    Node ||--o{ Publisher : creates
    Node ||--o{ Subscription : creates
    Node ||--o{ ServiceClient : creates
    Node ||--o{ ServiceServer : creates
    Node ||--o{ ActionClient : creates
    Node ||--o{ ActionServer : creates
    Node ||--|| Clock : "uses"
    
    Component ||--|| Node : "uses"
    Component ||--|| Clock : "injected with"
    Component ||--o| ComponentState : "has"
    
    Subscription ||--o{ Message : "buffers"
    
    Publisher }o--|| Zenoh : "publishes via"
    Subscription }o--|| Zenoh : "subscribes via"
```

---

## Layer Architecture

### Vertical Layers

```
┌─────────────────────────────────────────┐
│      Application Layer                  │
│  - Business Logic                       │
│  - Domain Models                        │
│  - Components                           │
└─────────────────┬───────────────────────┘
                  │ uses
┌─────────────────▼───────────────────────┐
│      ros2-zenoh Runtime                 │
│  - Node, Publisher, Subscription        │
│  - Service, Action                      │
│  - CDR Encoding/Decoding                │
│  - Type Safety                          │
└─────────────────┬───────────────────────┘
                  │ uses
┌─────────────────▼───────────────────────┐
│      Transport Layer (Zenoh)            │
│  - Session, Publisher, Subscriber       │
│  - Liveliness, Discovery                │
│  - Network (UDP/TCP)                    │
└─────────────────────────────────────────┘
```

### Horizontal Concerns (Cross-Cutting)

```
Time Management:
├─ Clock (WALL_TIME | SIM_TIME_LIVE | SIM_TIME_TEST)
├─ Duration, TimeStamp
└─ Scheduled events

Testing:
├─ MockNode, MockPublisher, MockSubscriber
├─ Clock (TEST mode)
└─ Fixtures

Logging:
├─ RosoutHandler
└─ Structured logs
```

---

## Data Flow

### Publishing

```mermaid
sequenceDiagram
    participant App as Application
    participant Pub as Publisher
    participant Zenoh as Zenoh Session
    participant Network
    
    App->>Pub: publish(msg)
    Pub->>Pub: serialize to CDR
    Pub->>Zenoh: put(key, payload)
    Zenoh->>Network: UDP/TCP packet
```

### Subscribing

```mermaid
sequenceDiagram
    participant Network
    participant Zenoh as Zenoh Session
    participant Sub as Subscription
    participant Buffer as MessageBuffer
    participant Callback
    
    Network->>Zenoh: Receive packet
    Zenoh->>Sub: on_sample(sample)
    Sub->>Sub: deserialize CDR
    Sub->>Buffer: append(msg)
    Note over Buffer: Drops oldest if full
    Sub->>Sub: set event
    Sub->>Buffer: popleft()
    Sub->>Callback: await callback(msg)
```

### Service Call

```mermaid
sequenceDiagram
    participant Client as ServiceClient
    participant Zenoh as Zenoh Session
    participant Server as ServiceServer
    participant Handler
    
    Client->>Client: serialize request
    Client->>Zenoh: get(key, payload)
    Zenoh->>Server: on_query(query)
    Server->>Server: deserialize request
    Server->>Handler: await handler(request)
    Handler-->>Server: response
    Server->>Server: serialize response
    Server->>Zenoh: reply(response)
    Zenoh-->>Client: response payload
    Client->>Client: deserialize response
```

---

## Time Management Architecture

```mermaid
stateDiagram-v2
    [*] --> WALL_TIME: Default
    [*] --> SIM_TIME_LIVE: Subscribe to /clock
    [*] --> SIM_TIME_TEST: Testing mode
    
    WALL_TIME: Uses time.monotonic()
    SIM_TIME_LIVE: Uses /clock messages
    SIM_TIME_TEST: Controlled via advance_by()
    
    note right of WALL_TIME
        Real system time
        asyncio.sleep() used directly
    end note
    
    note right of SIM_TIME_LIVE
        Simulation time from ROS2
        /clock topic subscription
        Approximate timing
    end note
    
    note right of SIM_TIME_TEST
        Deterministic testing
        No wall time dependency
        advance_by() processes events
    end note
```

---

## Testing Architecture

```mermaid
graph LR
    subgraph "Production"
        ProdNode[Node]
        ProdPub[Publisher]
        ProdSub[Subscription]
        Zenoh[Zenoh Session]
        
        ProdNode --> ProdPub
        ProdNode --> ProdSub
        ProdPub --> Zenoh
        ProdSub --> Zenoh
    end
    
    subgraph "Testing"
        TestNode[MockNode]
        TestPub[MockPublisher]
        TestSub[MockSubscriber]
        Bus[In-Memory Bus]
        
        TestNode --> TestPub
        TestNode --> TestSub
        TestPub --> Bus
        TestSub --> Bus
    end
    
    Component -->|Production| ProdNode
    Component -->|Testing| TestNode
    
    style Component fill:#f9f,stroke:#333
    style ProdNode fill:#9f9,stroke:#333
    style TestNode fill:#99f,stroke:#333
```

**Key Difference:** MockNode uses in-memory message bus instead of Zenoh, enabling:
- Synchronous message delivery (no network delay)
- Deterministic testing
- Fast execution
- No Zenoh daemon required

---

## Action Architecture

Actions are composed of underlying services and topics:

```mermaid
graph TB
    subgraph "Action Client"
        AClient[ActionClient]
        GoalPub[Goal Publisher]
        CancelPub[Cancel Publisher]
        StatusSub[Status Subscriber]
        FeedbackSub[Feedback Subscriber]
        ResultClient[Result Service Client]
    end
    
    subgraph "Action Server"
        AServer[ActionServer]
        GoalSub[Goal Subscriber]
        CancelSub[Cancel Subscriber]
        StatusPub[Status Publisher]
        FeedbackPub[Feedback Publisher]
        ResultServer[Result Service Server]
        Executor[Execute Callback]
    end
    
    AClient --> GoalPub
    AClient --> CancelPub
    AClient --> StatusSub
    AClient --> FeedbackSub
    AClient --> ResultClient
    
    AServer --> GoalSub
    AServer --> CancelSub
    AServer --> StatusPub
    AServer --> FeedbackPub
    AServer --> ResultServer
    AServer --> Executor
    
    GoalPub -.->|/<action>/goal| GoalSub
    CancelPub -.->|/<action>/cancel| CancelSub
    StatusPub -.->|/<action>/status| StatusSub
    FeedbackPub -.->|/<action>/feedback| FeedbackSub
    ResultClient -.->|/<action>/get_result| ResultServer
```

**Topics:**
- `/<action>/goal` - Goal messages
- `/<action>/cancel` - Cancel requests
- `/<action>/status` - Goal status updates
- `/<action>/feedback` - Execution feedback

**Service:**
- `/<action>/get_result` - Retrieve final result

---

## Discovery via Liveliness

```mermaid
sequenceDiagram
    participant Pub as Publisher
    participant Zenoh as Zenoh Session
    participant Sub as Subscriber
    
    Note over Pub: Create publisher
    Pub->>Zenoh: declare liveliness token<br/>@ros2_lv/*/MP/<topic>/<type>/<hash>
    
    Note over Sub: Create subscriber
    Sub->>Zenoh: liveliness.get(@ros2_lv/**/MS/<topic>/*)
    Zenoh-->>Sub: Matching publishers
    Sub->>Sub: Update matched_publishers count
    
    Sub->>Zenoh: declare liveliness token<br/>@ros2_lv/*/MS/<topic>/<type>/<hash>
    Zenoh-->>Pub: Liveliness change
    Pub->>Pub: Update matched_subscriptions count
```

**Key Expression Format:**
- `MP` = Matched Publication (publisher advertising)
- `MS` = Matched Subscription (subscriber advertising)
- `<hash>` = RIHS01 type hash (ensures type compatibility)

---

## State Machines

### Service Client State

```mermaid
stateDiagram-v2
    [*] --> Created
    Created --> WaitingForServer: wait_for_server()
    WaitingForServer --> Ready: Server discovered
    Ready --> Calling: call_async()
    Calling --> Ready: Response received
    Calling --> Error: Timeout/Error
    Error --> Ready: Retry
    Ready --> [*]: destroy()
```

### Action Goal State

```mermaid
stateDiagram-v2
    [*] --> PENDING: Goal sent
    PENDING --> ACCEPTED: Server accepts
    PENDING --> REJECTED: Server rejects
    ACCEPTED --> EXECUTING: Execution starts
    EXECUTING --> CANCELING: Cancel requested
    EXECUTING --> SUCCEEDED: Execution completes
    EXECUTING --> ABORTED: Execution fails
    CANCELING --> CANCELED: Cancel confirmed
    SUCCEEDED --> [*]
    REJECTED --> [*]
    ABORTED --> [*]
    CANCELED --> [*]
```

---

## Key Design Patterns

### 1. Dependency Injection

```python
class MyComponent:
    def __init__(self, node: Node, clock: Clock):
        self._node = node  # Injected
        self._clock = clock  # Injected
```

**Benefits:**
- Testability (inject mocks)
- Flexibility (swap implementations)
- Clear dependencies

### 2. Async Context Manager

```python
async with Node("my_node") as node:
    # Use node
    pass  # Automatic cleanup
```

**Benefits:**
- Resource safety
- Exception handling
- Clear lifecycle

### 3. Ring Buffer

```python
self._buffer = deque(maxlen=depth)
```

**Benefits:**
- Bounded memory
- Drop oldest semantics
- O(1) append/pop

### 4. Immutable State

```python
@dataclass(frozen=True)
class State:
    field: int
```

**Benefits:**
- Thread safety
- Predictable behavior
- Clear state transitions

---

## Performance Characteristics

| Operation | Complexity | Notes |
|-----------|------------|-------|
| publish() | O(1) | Serialize + Zenoh put |
| subscribe callback | O(1) | Deserialize + callback |
| buffer append | O(1) | deque append (drop oldest) |
| wait_for_subscribers() | O(n) | Poll liveliness tokens |
| create_publisher() | O(1) | Setup Zenoh publisher |
| create_subscription() | O(1) | Setup Zenoh subscriber |
| Node.destroy() | O(n) | n = number of entities |

---

## Memory Model

```
Node
├── Zenoh Session (1)
│   └── Network buffers (~10MB default)
├── Publishers (n)
│   └── Serialization buffer per publish (~1KB typical)
├── Subscriptions (m)
│   └── Message buffer: depth × message_size
│       (e.g., depth=10, ~1KB msg = ~10KB)
└── Service/Action entities
    └── Per-request buffers (~1KB typical)

Total Memory: ~10MB base + (n+m) × 1KB + Σ(buffer_size)
```

**Typical:** Node with 5 pubs, 5 subs, depth=10 → ~15MB

---

**Document Status:** Living document  
**Last Updated:** 2025-10-23  
**Next Review:** After Phase 1 completion

