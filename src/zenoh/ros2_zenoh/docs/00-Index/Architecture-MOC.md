---
id: index:architecture-moc
title: Architecture Map of Content
type: index
category: moc
summary: Navigation hub for architecture documentation
updated: 2025-10-23
---

# Architecture Map of Content

**Purpose:** Central navigation for all ros2-zenoh architecture documentation

---

## Core References

- [[Ubiquitous-Language]] - Domain terminology and concepts
- [[Domain-Model]] - Visual architecture and relationships
- [[ACTION_PLAN]] - Current priorities and timeline

---

## Concepts (Explanations)

### Core Abstractions
- [[01-Concepts/Component-Lifecycle]] - Setup/teardown pattern
- [[01-Concepts/Immutable-State]] - Frozen dataclasses for state
- [[01-Concepts/Dependency-Injection]] - Injecting Node and Clock

###Time Management
- [[01-Concepts/Simulation-Time]] - WALL_TIME vs SIM_TIME_LIVE vs SIM_TIME_TEST
- [[01-Concepts/Clock-Modes]] - When to use each mode
- [[01-Concepts/Timer-Scheduling]] - Async delays and scheduled events

### Testing
- [[01-Concepts/Testing-Philosophy]] - Deterministic, fast, isolated
- [[01-Concepts/Mocking-Strategy]] - MockNode vs real Node
- [[01-Concepts/Fast-Mode-Testing]] - advance_by() for instant tests

---

## Patterns (Proven Solutions)

### Basic Patterns
- [[05-Patterns/Basic-Publisher]] - Simple publishing pattern
- [[05-Patterns/Basic-Subscriber]] - Simple subscription pattern
- [[05-Patterns/Basic-Component]] - Stateless component

### Advanced Patterns
- [[05-Patterns/Stateful-Component]] - Component with immutable state
- [[05-Patterns/Image-Processing]] - Large data handling
- [[05-Patterns/Service-Handler]] - Request/response pattern
- [[05-Patterns/Action-Server]] - Long-running goal execution
- [[05-Patterns/Testing-With-Mocks]] - Testing with MockNode

---

## Layers

### Application Layer
**What:** Business logic, domain models, components  
**Concerns:** What the system does  
**Key Concepts:**
- [[01-Concepts/Component-Lifecycle]]
- [[01-Concepts/Immutable-State]]
- [[01-Concepts/Dependency-Injection]]

**Patterns:**
- [[05-Patterns/Basic-Component]]
- [[05-Patterns/Stateful-Component]]

### Runtime Layer (ros2-zenoh)
**What:** ROS2 abstractions over Zenoh  
**Concerns:** How messages flow  
**Key Entities:**
- Node - Resource manager
- Publisher - Send messages
- Subscription - Receive messages
- ServiceClient/ServiceServer - Request/response
- ActionClient/ActionServer - Goals with feedback

**Specifications:**
- [[06-Specs/ZenohNode]]
- [[06-Specs/Publisher]]
- [[06-Specs/Subscription]]

### Transport Layer (Zenoh)
**What:** Low-level pub/sub  
**Concerns:** Network communication  
**Key Concepts:**
- Session - Connection to Zenoh router
- Liveliness - Discovery mechanism
- Key expressions - Topic routing

**External:** Zenoh documentation

---

## Cross-Cutting Concerns

### Time Management

```
TimeMode
├─ WALL_TIME          → Real system time
├─ SIM_TIME_LIVE      → /clock topic subscription
└─ SIM_TIME_TEST      → Deterministic testing
```

**Concepts:**
- [[01-Concepts/Simulation-Time]]
- [[01-Concepts/Clock-Modes]]

**Specifications:**
- [[06-Specs/Testing-Clock]]

### Testing

```
Testing Strategy
├─ Unit Tests         → MockNode + Clock (TEST mode)
├─ Integration Tests  → Real Node + Clock (WALL_TIME)
└─ Interop Tests      → ros2-zenoh ↔ rclpy
```

**Concepts:**
- [[01-Concepts/Testing-Philosophy]]
- [[01-Concepts/Mocking-Strategy]]

**Patterns:**
- [[05-Patterns/Testing-With-Mocks]]

**Specifications:**
- [[06-Specs/Testing-Mocks]]

### Code Generation

```
Generation Pipeline
1. Specification      → Obsidian markdown (YAML + TypeScript)
2. Pattern Selection  → Choose appropriate pattern
3. Code Generation    → AST-based or LLM-based
4. Multi-Language     → Python, Rust, C, TypeScript
```

**Future:**
- Pattern Library (Phase 3)
- MCP Server (Phase 4)

---

## Component Specifications

### Current (Phase 1 - In Progress)
- [[06-Specs/Testing-Clock]] - Simulation time for testing
- [[06-Specs/Testing-Mocks]] - In-memory test doubles
- [[06-Specs/Subscription-Depth]] - Ring buffer semantics

### Existing (To Document)
- [[06-Specs/ZenohNode]] - Main node interface
- [[06-Specs/Publisher]] - Message publishing
- [[06-Specs/Subscription]] - Message receiving
- [[06-Specs/ServiceClient]] - Service requests
- [[06-Specs/ServiceServer]] - Service handling
- [[06-Specs/ActionClient]] - Action goals
- [[06-Specs/ActionServer]] - Goal execution

---

## Tutorials (Learning-Oriented)

### Getting Started
- [[02-Tutorials/Getting-Started]] - Installation and first node
- [[02-Tutorials/First-Component]] - Build a simple component

### Testing
- [[02-Tutorials/Testing-With-Mocks]] - Write deterministic tests
- [[02-Tutorials/Simulation-Time-Testing]] - Use Clock for testing

---

## How-To Guides (Problem-Solving)

### Data Handling
- [[03-HowTo/Handle-Large-Data]] - Images, point clouds, etc.
- [[03-HowTo/Immutable-State-Pattern]] - Manage component state

### Testing
- [[03-HowTo/Test-With-Sim-Time]] - Use SIM_TIME_TEST mode
- [[03-HowTo/Debug-Timing-Issues]] - Fix flaky tests

### Implementation
- [[03-HowTo/Implement-Lifecycle]] - Setup/teardown pattern
- [[03-HowTo/Generate-From-Spec]] - Use documentation-driven workflow

---

## Reference (Information)

### API Documentation
- [[04-Reference/API/Python]] - Python API reference
- [[04-Reference/API/Rust]] - Rust API reference (future)
- [[04-Reference/API/C]] - C API reference (future)
- [[04-Reference/API/TypeScript]] - TypeScript API reference (future)

### Message Types
- [[04-Reference/ROS2-Messages]] - Standard ROS2 messages

---

## Development Workflow

```mermaid
graph LR
    Draft[Write Description] --> Spec[Generate Spec]
    Spec --> Review[Review & Refine]
    Review --> Generate[Generate Code]
    Generate --> Implement[Implement Logic]
    Implement --> Test[Write Tests]
    Test --> Extract[Extract Pattern]
    Extract --> Draft2[Next Component]
```

**Steps:**
1. **Draft:** Write description in `06-Specs/DRAFT-ComponentName.md`
2. **Analyze:** MCP tool extracts domain model
3. **Specify:** Generate formal spec with YAML + TypeScript
4. **Review:** Human reviews and refines
5. **Generate:** MCP generates code from spec
6. **Implement:** Developer adds business logic
7. **Test:** Write tests using MockNode
8. **Extract:** Document as reusable pattern

**Current Status:** Validating workflow in Phase 1

---

## Architecture Decisions

### ADR-001: AsyncIO Over Threading
**Status:** Accepted  
**Context:** Need concurrent I/O for multiple subscriptions  
**Decision:** Use Python asyncio (native async/await)  
**Consequences:** All callbacks must be async-aware

### ADR-002: Ring Buffer for Depth
**Status:** Accepted  
**Context:** Need bounded memory for message buffers  
**Decision:** Use `deque(maxlen=depth)` - drop oldest  
**Consequences:** Matches ROS2 DDS behavior

### ADR-003: Immutable State Pattern
**Status:** Recommended  
**Context:** Need predictable state transitions  
**Decision:** Use `@dataclass(frozen=True)` for state  
**Consequences:** Create new state for each transition

### ADR-004: Clock Injection
**Status:** Accepted  
**Context:** Need testable time-dependent code  
**Decision:** Inject Clock via dependency injection  
**Consequences:** All time operations go through Clock

### ADR-005: Three Time Modes
**Status:** Accepted  
**Context:** Need wall time, sim time, and test time  
**Decision:** TimeMode enum with mutually exclusive modes  
**Consequences:** No accidental mixing of modes

### ADR-006: Documentation-Driven Development
**Status:** Proposed (Validating in Phase 1)  
**Context:** Need consistent multi-language generation  
**Decision:** Specifications in Obsidian as source of truth  
**Consequences:** Write specs before implementation

---

## Quality Attributes

### Performance
- **Goal:** < 1ms publish latency (typical)
- **Measurement:** Profiling tools
- **Strategy:** Zero-copy where possible

### Reliability
- **Goal:** 99.99% message delivery (best-effort Zenoh)
- **Measurement:** Long-running stress tests
- **Strategy:** Graceful degradation, error logging

### Testability
- **Goal:** All code testable without Zenoh daemon
- **Measurement:** Test coverage > 90%
- **Strategy:** MockNode, Clock injection, pure functions

### Maintainability
- **Goal:** New contributor productive in < 1 week
- **Measurement:** Onboarding time
- **Strategy:** Documentation-driven, clear patterns

---

## External Dependencies

```
ros2-zenoh
├── zenoh-python      → Transport layer
├── pycdr2            → CDR encoding/decoding
├── builtin_interfaces → ROS2 time messages
└── asyncio           → Concurrency (stdlib)
```

---

## Related Projects

- **rmw_zenoh_cpp** - Official ROS2 Zenoh middleware (C++)
- **Zenoh** - Zero-overhead pub/sub protocol
- **ROS2** - Robot Operating System 2
- **DDS** - Data Distribution Service (alternative transport)

---

## Glossary

Quick links to [[Ubiquitous-Language]] definitions:

- **Node** - Container for publishers/subscribers
- **Component** - Business logic unit
- **Clock** - Time source (WALL/SIM/TEST)
- **TimeMode** - Clock operating mode
- **MockNode** - In-memory test double
- **Aggregate** - Consistency boundary
- **Value Object** - Immutable descriptor
- **CDR** - Common Data Representation (serialization)
- **Liveliness** - Zenoh discovery mechanism

---

## Navigation Tips

### By Role

**Application Developer:**
1. Start: [[02-Tutorials/Getting-Started]]
2. Learn: [[01-Concepts/Component-Lifecycle]]
3. Build: [[05-Patterns/Basic-Component]]

**Library Contributor:**
1. Read: [[Ubiquitous-Language]]
2. Understand: [[Domain-Model]]
3. Follow: [[06-Specs/Testing-Clock]] (example spec)

**Pattern Designer:**
1. Extract: [[05-Patterns/Basic-Publisher]] (template)
2. Document: [[06-Specs/ZenohNode]] (spec format)
3. Validate: Write tests

### By Task

**I want to publish a message:**
→ [[05-Patterns/Basic-Publisher]]

**I want to test my component:**
→ [[05-Patterns/Testing-With-Mocks]]

**I want to understand time management:**
→ [[01-Concepts/Simulation-Time]]

**I want to write a specification:**
→ [[06-Specs/Testing-Clock]] (example)

**I want to understand the architecture:**
→ [[Domain-Model]]

---

**Last Updated:** 2025-10-23  
**Maintainer:** Development Team  
**Next Review:** After Phase 1 completion

