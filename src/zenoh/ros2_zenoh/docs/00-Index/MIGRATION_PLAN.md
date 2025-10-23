---
id: index:migration-plan
title: Documentation Migration & Analysis Plan
type: index
updated: 2025-10-23
classification: internal
llm_processing: cloud-ok
schema_version: "1.0"
---

# Documentation Migration & Analysis Plan

**Purpose:** Migrate existing documentation and code into the new `ros2_zenoh/` structure

---

## Current State Analysis

### Existing Documentation (Excellent!)

✅ **TESTING_DESIGN_DECISIONS.md** (599 lines)
- Comprehensive design rationale
- 12 key decisions documented
- Architecture patterns defined
- State management strategies

✅ **TESTING_GUIDE.md** (438 lines)
- User-facing guide
- Code examples
- Best practices
- Common patterns

### Existing Implementation

✅ **ros2_zenoh_python/** - Working Python library
- node.py, publisher.py, subscription.py
- service_client.py, service_server.py
- action_client.py, action_server.py
- Comprehensive tests
- Working examples

### What's Missing (To Implement)

❌ **testing/** module - Not yet implemented
- clock.py - TimeMode, Clock class
- mocks.py - MockNode, MockPublisher, etc.
- fixtures.py - pytest fixtures

❌ **Ring buffer depth** - Needs update in subscription.py

---

## Migration Strategy

### Phase 0.1: Create Clean Ecosystem (5 min)

```bash
cd /home/ubuntu/ws/src/zenoh
mkdir -p ros2_zenoh/{python,rust,typescript,c}

# Move new docs
mv docs ros2_zenoh/
mv .cursorrules ros2_zenoh/
mv patterns ros2_zenoh/
mv mcp-server ros2_zenoh/
mv ROS2_ZENOH_ECOSYSTEM_PLAN.md ros2_zenoh/

# Copy working Python (not move!)
cp -r ros2_zenoh_python ros2_zenoh/python/
```

### Phase 0.2: Migrate Existing Docs → New Structure (1 day)

**Convert TESTING_DESIGN_DECISIONS.md into:**

1. **Architecture Decision Records** (docs/00-Index/ADR-*.md)
   - ADR-001: Custom Mock Classes vs unittest.mock
   - ADR-002: Pure Asyncio vs ROS2 Timers
   - ADR-003: Three Distinct Time Modes
   - ADR-004: advance_by() for Multi-Event Processing
   - ADR-005: Ring Buffer Semantics (Drop Oldest)
   - ADR-006: No Custom Executor
   - ADR-007: Dependency Injection Pattern
   - ADR-008: Separation of Business Logic from I/O
   - ADR-009: Immutable State for Small Data
   - ADR-010: Separate Metadata for Large Data
   - ADR-011: Simple Lifecycle Pattern
   - ADR-012: Opt-In Testing Module

2. **Concepts** (docs/01-Concepts/)
   - Simulation-Time.md (from "Time Management" section)
   - Clock-Modes.md (WALL_TIME, SIM_TIME_LIVE, SIM_TIME_TEST)
   - Mocking-Strategy.md (custom classes rationale)
   - Dependency-Injection.md (patterns)
   - Immutable-State.md (small + large data patterns)
   - Component-Lifecycle.md (simple/context/full patterns)

3. **Patterns** (docs/05-Patterns/)
   - Testing-With-Mocks.md (from existing guide)
   - Dependency-Injection-Pattern.md
   - Separation-Of-Concerns.md (3-layer architecture)
   - Immutable-State-Pattern.md
   - Context-Manager-Pattern.md

**Convert TESTING_GUIDE.md into:**

4. **Tutorials** (docs/02-Tutorials/)
   - Testing-With-Simulation-Time.md
   - Testing-With-MockNode.md
   - Testing-Periodic-Publishers.md
   - Testing-State-Machines.md

5. **How-To Guides** (docs/03-HowTo/)
   - Test-With-Sim-Time.md
   - Mock-Services.md
   - Handle-Large-Data.md
   - Test-Multi-Node-Scenarios.md

### Phase 0.3: Document Existing Code (Retroactive Specs) (2 days)

**Create specs for what exists:**

```
docs/06-Specs/
├── ZenohNode.md           # Document node.py
├── Publisher.md           # Document publisher.py
├── Subscription.md        # Document subscription.py + plan depth update
├── ServiceClient.md       # Document service_client.py
├── ServiceServer.md       # Document service_server.py
├── ActionClient.md        # Document action_client.py
└── ActionServer.md        # Document action_server.py
```

**Purpose:**
- Validate our spec format works
- Document what we have
- Identify improvements needed
- Create baseline for changes

### Phase 0.4: Extract Patterns from Code (1 day)

**Analyze existing code and extract patterns:**

```
docs/05-Patterns/
├── Basic-Publisher.md       # From examples/
├── Basic-Subscriber.md      # From examples/
├── Service-Client-Pattern.md
├── Action-Server-Pattern.md
└── Testing-With-Mocks.md    # From test patterns
```

---

## Migration Mapping

| Source | Destination | Action |
|--------|-------------|--------|
| **TESTING_DESIGN_DECISIONS.md** | | |
| → "Q1: Mocking Strategy" | `docs/00-Index/ADR-001-Custom-Mocks.md` | Extract ADR |
| → "Q2: Timer Implementation" | `docs/00-Index/ADR-002-Pure-Asyncio.md` | Extract ADR |
| → "Q3: Fast Mode Scope" | `docs/01-Concepts/Simulation-Time.md` | Concept doc |
| → "Q4: Integration Strategy" | `docs/00-Index/ADR-012-Opt-In-Testing.md` | Extract ADR |
| → "Q5: Custom Executor" | `docs/00-Index/ADR-006-No-Executor.md` | Extract ADR |
| → "Mocking Strategy" section | `docs/01-Concepts/Mocking-Strategy.md` | Concept doc |
| → "Time Management" section | `docs/01-Concepts/Clock-Modes.md` | Concept doc |
| → "Subscription Depth" section | `docs/00-Index/ADR-005-Ring-Buffer.md` | Extract ADR |
| → "Architecture Patterns" section | `docs/05-Patterns/` | Extract patterns |
| → "State Management" section | `docs/01-Concepts/Immutable-State.md` | Concept doc |
| → "Lifecycle Patterns" section | `docs/01-Concepts/Component-Lifecycle.md` | Concept doc |
| **TESTING_GUIDE.md** | | |
| → "Simulation Time" section | `docs/02-Tutorials/Testing-With-Simulation-Time.md` | Tutorial |
| → "Mocking & DI" section | `docs/02-Tutorials/Testing-With-MockNode.md` | Tutorial |
| → "Best Practices" section | `docs/03-HowTo/Testing-Best-Practices.md` | How-to |
| → "Common Patterns" section | `docs/05-Patterns/Testing-Patterns.md` | Patterns |
| **Code** | | |
| → `ros2_zenoh_python/node.py` | `docs/06-Specs/ZenohNode.md` | Retroactive spec |
| → `ros2_zenoh_python/publisher.py` | `docs/06-Specs/Publisher.md` | Retroactive spec |
| → `ros2_zenoh_python/subscription.py` | `docs/06-Specs/Subscription.md` | Retroactive spec |
| → `tests/test_*.py` | `docs/05-Patterns/Testing-Patterns.md` | Extract patterns |
| → `examples/*.py` | `docs/05-Patterns/Basic-*.md` | Extract patterns |

---

## New Work (After Migration)

### Phase 1A: Write New Specs (Doc-Driven)

```
docs/06-Specs/
├── Testing-Clock.md         # NEW - TimeMode, Clock class
├── Testing-Mocks.md         # NEW - MockNode, MockPublisher
└── Testing-Fixtures.md      # NEW - pytest fixtures
```

### Phase 1B: Implement from Specs

```
ros2_zenoh/python/ros2_zenoh_python/testing/
├── __init__.py
├── clock.py          # Implement from Testing-Clock.md
├── mocks.py          # Implement from Testing-Mocks.md
└── fixtures.py       # Implement from Testing-Fixtures.md
```

### Phase 1C: Update Existing Code

```
ros2_zenoh/python/ros2_zenoh_python/
└── subscription.py   # Update: Add ring buffer (deque)
```

---

## Deliverables Checklist

### Phase 0: Migration & Analysis

- [ ] Create `ros2_zenoh/` structure
- [ ] Copy Python implementation
- [ ] Extract 12 ADRs from TESTING_DESIGN_DECISIONS.md
- [ ] Create 6 Concept docs
- [ ] Create 5 Pattern docs from existing decisions
- [ ] Create 4 Tutorial docs from TESTING_GUIDE.md
- [ ] Create 4 How-To docs
- [ ] Write 7 retroactive specs (existing code)
- [ ] Extract patterns from code/tests
- [ ] Validate all links work
- [ ] Review for completeness

### Phase 1: New Implementation

- [ ] Write Testing-Clock spec
- [ ] Write Testing-Mocks spec
- [ ] Write Testing-Fixtures spec
- [ ] Implement clock.py from spec
- [ ] Implement mocks.py from spec
- [ ] Implement fixtures.py from spec
- [ ] Update subscription.py for ring buffer
- [ ] Write comprehensive tests
- [ ] Update examples to use new patterns

---

## Success Criteria

✅ **All existing knowledge preserved**
- Design decisions → ADRs
- User guides → Tutorials/How-Tos
- Concepts → Concept docs

✅ **Spec format validated**
- Can write specs for existing code
- Specs are clear and actionable
- Format works for our domain

✅ **Foundation for doc-driven workflow**
- Write spec → implement → test
- Proven on real features
- Templates validated

✅ **Clean ecosystem**
- `ros2_zenoh/` has only quality-controlled code
- Documentation complete
- CI/CD validates everything

---

## Timeline

| Phase | Duration | Description |
|-------|----------|-------------|
| 0.1 | 5 min | Create structure |
| 0.2 | 1 day | Migrate docs (ADRs, Concepts, Patterns) |
| 0.3 | 2 days | Write retroactive specs |
| 0.4 | 1 day | Extract patterns from code |
| **Total** | **4 days** | **Complete migration** |
| 1A | 1 day | Write new specs (Clock, Mocks) |
| 1B | 3 days | Implement testing infrastructure |
| 1C | 1 day | Update existing code |
| **Total** | **5 days** | **Phase 1 complete** |

---

## Next Actions

**Immediate (Step by step):**

1. ✅ Read TESTING_DESIGN_DECISIONS.md (DONE)
2. ✅ Read TESTING_GUIDE.md (DONE)
3. ✅ Create migration plan (THIS FILE)
4. ➡️ Create `ros2_zenoh/` structure
5. ➡️ Extract first ADR (validate format)
6. ➡️ Extract first Concept doc (validate format)
7. ➡️ Write first retroactive spec (validate format)
8. ➡️ Continue with full migration

**Want to proceed?** Should I:
- Create the `ros2_zenoh/` structure now?
- Start extracting ADRs?
- Write the first retroactive spec?


