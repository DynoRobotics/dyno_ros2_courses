---
id: index:action-plan
title: ROS2 Zenoh Action Plan
type: index
status: active
updated: 2025-10-23
---

# ROS2 Zenoh Action Plan

**Current Phase:** Phase 1 - Polish Python Implementation  
**Timeline:** 2-3 weeks  
**Full Plan:** See [[../ROS2_ZENOH_ECOSYSTEM_PLAN.md]] for complete multi-language roadmap

---

## Current Status

### ✅ Completed
- Python core library (Node, Publisher, Subscriber, Service, Action)
- Python message generation (pycdr2-based)
- Interop tests with rclpy (passing)
- Basic examples and tools

### 🚧 In Progress
- Testing infrastructure ([[TESTING_DESIGN_DECISIONS.md]])
- Ring buffer depth semantics
- Remove static sleeps

### ⏳ Next
- Documentation-driven workflow validation
- Pattern extraction from existing code
- Multi-language planning

---

## Phase 1: Polish Python (Current - Weeks 1-3)

### Week 1: Testing Infrastructure Setup

**Goal:** Foundation for deterministic, fast testing

#### 1.1 Vault & Documentation Foundation
- [x] Create docs folder structure
- [ ] Write core documentation
  - [ ] [[Ubiquitous-Language]]
  - [ ] [[Domain-Model]]
  - [ ] [[Architecture-MOC]]
- [ ] Create templates
  - [ ] Pattern template
  - [ ] Component spec template
  - [ ] Tutorial template

#### 1.2 Specify Testing Components (Doc-Driven!)

Write specifications BEFORE implementing:

- [ ] **[[06-Specs/Testing-Clock]]** - Simulation time management
  - TimeMode enum (WALL_TIME, SIM_TIME_LIVE, SIM_TIME_TEST)
  - Clock class with `sleep()`, `now()`, `advance_by()`
  - Event queue for scheduled wakeups
- [ ] **[[06-Specs/Testing-Mocks]]** - In-memory testing mocks
  - MockNode, MockPublisher, MockSubscriber
  - Synchronous message delivery
  - Optional depth enforcement
- [ ] **[[06-Specs/Subscription-Depth]]** - Ring buffer semantics
  - Update existing Subscription to use `deque(maxlen=depth)`
  - Document drop-oldest behavior

#### 1.3 Implementation (From Specs)

```bash
# Create from specifications
ros2_zenoh_python/testing/
├── __init__.py
├── clock.py          # Implements Testing-Clock.md
├── mocks.py          # Implements Testing-Mocks.md
└── fixtures.py       # Pytest fixtures
```

**Acceptance:** All specs implemented, tests pass

### Week 2: Refactor & Test

#### 2.1 Update Production Code
- [ ] Inject Clock into components that use time
- [ ] Update Subscription to use ring buffer (deque)
- [ ] Remove all static `asyncio.sleep()` calls
- [ ] Add `wait_for_*` methods where needed

#### 2.2 Update Tests
- [ ] Refactor tests to use MockNode
- [ ] Use Clock.advance_by() instead of sleeps
- [ ] Verify all tests pass with SIM_TIME_TEST
- [ ] Measure test execution time (target: <10s total)

#### 2.3 Documentation
- [ ] Document architecture decisions
- [ ] Add API documentation
- [ ] Update examples to follow best practices

**Acceptance:** All tests pass, execution time improved, code documented

### Week 3: Pattern Extraction

#### 3.1 Extract Patterns from Existing Code

Retroactively document what works:

- [ ] **[[05-Patterns/Basic-Publisher]]** - Extract from examples
- [ ] **[[05-Patterns/Basic-Subscriber]]** - Extract from examples
- [ ] **[[05-Patterns/Service-Handler]]** - Extract from service examples
- [ ] **[[05-Patterns/Action-Server]]** - Extract from action server
- [ ] **[[05-Patterns/Testing-With-Mocks]]** - Extract from tests

#### 3.2 Validate Specs for Existing Components

Write retroactive specs for what we built:

- [ ] **[[06-Specs/ZenohNode]]** - Document existing Node API
- [ ] **[[06-Specs/Publisher]]** - Document Publisher API
- [ ] **[[06-Specs/Subscription]]** - Document Subscription API
- [ ] **[[06-Specs/ServiceClient]]** - Document Service client
- [ ] **[[06-Specs/ActionClient]]** - Document Action client

#### 3.3 Workflow Validation
- [ ] Verify spec format works for our domain
- [ ] Identify gaps in documentation structure
- [ ] Refine templates based on learnings

**Acceptance:** 5+ patterns documented, key components have specs, workflow validated

---

## Phase 2: TypeScript Interface Generator (Weeks 4-7)

**Goal:** Enable multi-language message generation

### Scope
- Port Python generator to TypeScript
- Add ROS2 workspace overlay support (AMENT_PREFIX_PATH)
- Generate Python, Rust, C, TypeScript messages
- Browser bundle for web-based generation

**Reference:** [[../ROS2_ZENOH_ECOSYSTEM_PLAN.md#phase-2]]

---

## Phase 3: Pattern Library (Weeks 8-10)

**Goal:** Formalize patterns for multi-language generation

### Scope
- Pattern catalog (10+ proven patterns)
- TypeScript + YAML IR format
- Simple generator (AST-based, deterministic)
- LLM generator (few-shot, adaptive)

**Reference:** [[../ROS2_ZENOH_ECOSYSTEM_PLAN.md#phase-3]]

---

## Phase 4: MCP Server (Weeks 11-14)

**Goal:** Documentation-driven workflow tooling

### Scope
- Documentation tools (analyze, create spec, review)
- Code generation tools (from specs)
- Inspection tools (runtime introspection)
- Debugging tools (timing, tracing)

**Reference:** [[../ROS2_ZENOH_ECOSYSTEM_PLAN.md#phase-4]]

---

## Phase 5: Multi-Language (Weeks 15+)

**Goal:** Rust, C, TypeScript implementations

### Scope
- Generate from proven patterns
- Cross-language tests
- Performance benchmarks
- Complete documentation

**Reference:** [[../ROS2_ZENOH_ECOSYSTEM_PLAN.md#phase-5]]

---

## Success Criteria

### Phase 1 Complete ✓
- [ ] Testing infrastructure implemented (Clock, mocks)
- [ ] All tests pass with simulation time
- [ ] Test execution time < 10s
- [ ] 5+ patterns documented
- [ ] Key components have specs
- [ ] Documentation-driven workflow validated

### Future Phases
See [[../ROS2_ZENOH_ECOSYSTEM_PLAN.md#success-criteria]]

---

## Repository Structure

```
/home/ubuntu/ws/src/zenoh/               # Monorepo root
├── docs/                                # THIS VAULT (source of truth)
│   ├── 00-Index/                        # Navigation (you are here)
│   ├── 01-Concepts/                     # Explanations
│   ├── 02-Tutorials/                    # Learning guides
│   ├── 03-HowTo/                        # Problem-solving
│   ├── 04-Reference/                    # API docs
│   ├── 05-Patterns/                     # Code patterns
│   └── 06-Specs/                        # Component specs
│
├── ros2_zenoh_python/                   # Python implementation (CURRENT)
│   ├── ros2_zenoh_python/
│   │   ├── node.py, publisher.py, ...   # Core library
│   │   ├── testing/                     # 🚧 Phase 1
│   │   │   ├── clock.py
│   │   │   └── mocks.py
│   │   └── _bundled_msgs/
│   ├── tests/
│   ├── examples/
│   ├── TESTING_DESIGN_DECISIONS.md
│   └── TESTING_GUIDE.md
│
├── ros2_interface_generator/            # Python message generator
│   └── ros2_interface_generator/        # Will port to TypeScript (Phase 2)
│
├── patterns/                            # Pattern library (Phase 3)
│   ├── specs/                           # TypeScript + YAML
│   └── validation/
│
├── mcp-server/                          # MCP server (Phase 4)
│   └── src/
│
├── ros2_zenoh_rs/                       # Rust (future)
├── ros2_zenoh_node/                     # TypeScript/Node (future)
└── ROS2_ZENOH_ECOSYSTEM_PLAN.md         # Full roadmap
```

---

## Next Actions (This Week)

**Priority: Complete Week 1 of Phase 1**

1. Write core documentation (Ubiquitous Language, Domain Model)
2. Create templates (pattern, spec)
3. Write Testing-Clock specification
4. Write Testing-Mocks specification
5. Implement Clock from spec
6. Implement MockNode from spec

**Time estimate:** 5-7 days

---

## Links

- **Full Roadmap:** [[../ROS2_ZENOH_ECOSYSTEM_PLAN.md]]
- **Testing Decisions:** [[../ros2_zenoh_python/TESTING_DESIGN_DECISIONS.md]]
- **Testing Guide:** [[../ros2_zenoh_python/TESTING_GUIDE.md]]

---

**Status:** Active  
**Owner:** Development Team  
**Next Review:** End of Week 1 (Phase 1)

