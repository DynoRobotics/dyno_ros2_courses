---
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: index
id: "open-issues"
title: "Open Issues and Future Considerations"
summary: "Known limitations and future work for ros2-zenoh ecosystem"
tags: ["open-issues", "future-work", "limitations"]
---

# Open Issues and Future Considerations

This document tracks known limitations, design decisions that may need revisiting, and potential future work.

---

## 1. DDS Interoperability

**Status**: Not currently possible  
**Impact**: High (limits adoption with existing ROS2 deployments)  
**Complexity**: Medium (if bridge exists), High (if we build it)  
**Related**: ADR-004 (when created)

### Problem

ros2-zenoh-python targets `rmw_zenoh` compatibility, but most ROS2 deployments use DDS-based RMWs (FastDDS, CycloneDDS). No bridge currently exists between rmw_zenoh and DDS.

### Potential Solutions

1. **Wait for official bridge**: ROS2/Zenoh teams may provide one
2. **Implement our own bridge**: Translate between key formats (feasible - both use Zenoh)
3. **Support both formats**: Make key generation configurable

### Architectural Preparation

We've isolated key format generation to make future adaptation easier:
- `liveliness_manager.py`: Discovery format (encoding-aware namespaces)
- Data key generation: Separate function, encoding-aware
- Both can be made configurable without major refactoring

### Timeline

- **v1.0**: rmw_zenoh only (current plan)
- **v1.x**: Add bridge support if available
- **v2.0**: Potentially support multiple key formats

---

## 2. Type Name Format (Python vs C++)

**Status**: Using Python-style, may need C++ mangled  
**Impact**: Medium (affects rmw_zenoh C++ interop)  
**Complexity**: Low (just string format)  
**Priority**: HIGH - must verify before v1.0

### Problem

C++ rmw_zenoh uses mangled type names: `geometry_msgs::msg::dds_::Twist_`  
Our Python uses slash-separated: `geometry_msgs/msg/Twist`

### Question

Does this cause interop issues? Need to verify with actual rmw_zenoh testing.

### Action Items

- [ ] Test Python ↔ C++ rmw_zenoh interop
- [ ] Determine if mangling is required  
- [ ] Add configuration if needed
- [ ] Update specs with correct format

### Investigation

Check the experiments file - it shows C++ format with `::dds_::` in the type name.
This may be required for full compatibility.

---

## 3. Non-CDR Encoding Compatibility

**Status**: Supported via namespace isolation  
**Impact**: Medium (limits encoding flexibility)  
**Complexity**: Low (already implemented via namespaces)  
**Related**: Core/Publisher spec

### Current Solution

Different encodings use isolated namespaces:
- CDR: `0/` and `@ros2_lv/` (rmw_zenoh compatible)
- JSON: `zenoh_json/` and `@zenoh_app/json/` (isolated)
- MessagePack: `zenoh_msgpack/` and `@zenoh_app/msgpack/` (isolated)

This prevents rmw_zenoh from discovering and attempting to deserialize non-CDR messages.

### Limitations

- Encoding not in discovery metadata (can't query "give me JSON publishers")
- rmw_zenoh cannot co-exist with non-CDR on same topics (but safe via namespaces)
- Must know encoding at subscription time

### Future Improvements

If needed:
1. Add encoding to liveliness token
2. Support multi-encoding subscriptions (advanced use case)
3. Encoding negotiation protocol

### Current Recommendation

Document: "Use CDR for rmw_zenoh interop, other encodings for Zenoh-native apps only"

---

## 4. Lifecycle Management

**Status**: No full lifecycle for v1.0; Pausable pattern for v1.0  
**Impact**: Low (async patterns + pause/resume cover most needs)  
**Complexity**: Medium (if full lifecycle added)  
**Related**: [[ADR-005]]

### Current Approach

No built-in lifecycle system. Use proper async initialization patterns:

```python
class MyComponent:
    def __init__(self, node):
        self.node = node
    
    async def setup(self):
        # 1. Initialize state first
        self.state = ComplexState()
        
        # 2. Create subscriptions after
        self.sub = self.node.create_subscription(...)
    
    def callback(self, msg):
        self.state.process(msg)  # Safe
```

### Pausable Component Convention (Approved for Phase 1.5)

**WILL DOCUMENT**: Standard API convention for pause/resume:

**API Convention**:
- `~/pause` service (`std_srvs/srv/Trigger`)
- `~/resume` service (`std_srvs/srv/Trigger`)

**Implementation**: User's choice (drop, queue, buffer, etc.)

**Optional Helper**: `ros2_zenoh_python.conventions.PausableServiceMixin`

```bash
# Usage from ROS2
ros2 service call /my_component/pause std_srvs/srv/Trigger
ros2 service call /my_component/resume std_srvs/srv/Trigger
```

**Use cases**: Debugging, testing, reconfiguration, resource management  
**Timeline**: After spec refactoring (Phase 1.5), before new features  
**Doc**: `03-Conventions/Pausable-Component-API.md`

---

### Fixed-Rate Execution (Approved for Phase 1.5)

**WILL IMPLEMENT**: Patterns for maintaining fixed execution rates:

**Components**:
- **`Rate`** class - Low-level building block for rate control
- **`fixed_rate_loop()`** async iterator - Pythonic, recommended pattern
- ❌ **NO `Timer`** - Skip callback-based timers (not Pythonic)

**Usage**:
```python
# Recommended: async iterator
async for tick in fixed_rate_loop(10.0, clock=clock):  # 10 Hz
    process()
    if tick.overrun > 0:
        print(f"Missed deadline by {tick.overrun}s")

# Alternative: explicit Rate
rate = Rate(10.0, clock=clock)
while True:
    process()
    await rate.sleep()
```

**Features**:
- Clock-aware (works with SIM_TIME_TEST for fast tests)
- Deadline detection and warnings
- Configurable overrun behavior (warn, raise, silent)
- Skip-if-behind semantics (don't queue missed iterations)

**Use cases**: Control loops, sensor publishing, periodic tasks  
**Timeline**: Phase 1.5 (with Clock implementation)  
**Specs**: `06-Specs/Core/Rate.md`, `06-Specs/Python/Rate-Python.md`

### Rationale

- Priority 1 is rmw_zenoh interop (no lifecycle requirement)
- Async/await provides sufficient ordering control
- Pausable pattern covers debugging/testing needs
- Most components are simple
- Avoid premature abstraction

### If Full Lifecycle Needed in Future

Optional `LifecycleComponent` base class:

```python
class MyComponent(LifecycleComponent):  # Optional!
    async def on_configure(self): ...
    async def on_activate(self): ...
    async def on_deactivate(self): ...
```

But NOT required - keep simple path simple.

### Indicators We'd Need Full Lifecycle

- Multiple user reports of initialization races
- Complex state machines become common (beyond pause/resume)
- Users writing lifecycle themselves repeatedly

**Current**: Document best practices, provide Pausable pattern, trust developers.

---

## 5. Multi-Language Code Generation

**Status**: Planned for Phase 3  
**Impact**: High (enables Rust, C, TypeScript)  
**Complexity**: High  
**Related**: [[ROS2_ZENOH_ECOSYSTEM_PLAN]]

### Approach

1. **Phase 1**: Build Python library, extract patterns
2. **Phase 2**: Document patterns in specs (Core/ vs Python/)
3. **Phase 3**: Build MCP server + generation tools
4. **Phase 4+**: Generate other languages

### Key Design

- TypeScript + YAML for IR (Intermediate Representation)
- Pattern library (executable Python examples)
- MCP server (TypeScript, universal)
- Per-language generators

### Blocked On

- Python patterns must be proven first
- Need sufficient examples to extract patterns
- Specs must be complete and validated

---

## 6. Performance Optimization

**Status**: Not yet profiled  
**Impact**: Unknown  
**Complexity**: Medium  
**Priority**: After v1.0 (premature optimization)

### Areas to Profile

1. **Serialization**: Is pre-bound function actually faster?
2. **Discovery**: Is liveliness query efficient?
3. **Message throughput**: Can we handle high-frequency publishers?
4. **Latency**: What's the overhead vs rmw_zenoh C++?

### When to Optimize

- After v1.0 release
- After user feedback
- Only if profiling shows actual bottlenecks

### Likely Optimizations

- Cython for hot paths
- Zero-copy where possible (Zenoh SHM)
- Async batching for high-frequency messages

---

## 7. Testing Infrastructure

**Status**: Planned (Phase 1)  
**Impact**: High (enables reliable testing)  
**Complexity**: Medium  
**Related**: [[Testing-Clock]], [[Testing-Mocks]]

### To Implement

- [ ] `testing.Clock` (SIM_TIME_TEST mode)
- [ ] `testing.MockNode` and mock entities
- [ ] Fixtures for common test patterns
- [ ] Integration test harness

### Blockers

- Need to finish Core specs first
- Then implement from specs (doc-driven)

---

## 8. Documentation Completeness

**Status**: In progress  
**Impact**: High (usability)  
**Complexity**: Medium  

### Needed

- [ ] Complete API reference (auto-generated)
- [ ] Tutorial series
- [ ] Best practices guide
- [ ] Migration guide (from rclpy)
- [ ] Troubleshooting guide

### Tooling

- Auto-generate API docs from code
- Test code examples in documentation
- Validate specs against code

---

## 9. Error Handling Patterns

**Status**: Not formalized  
**Impact**: Medium  
**Complexity**: Low  

### Questions

1. When to raise exceptions vs return None?
2. How to handle Zenoh session errors?
3. Retry policies for flaky networks?
4. Error recovery patterns?

### Current

- Ad-hoc error handling
- Some timeouts, some exceptions

### Future

- Document error handling patterns
- Consistent exception hierarchy
- Recovery strategies guide

---

## 10. Observability

**Status**: Basic logging only  
**Impact**: Medium (debugging)  
**Complexity**: Medium  

### Current

Python `logging` module

### Future Possibilities

- Structured logging
- Metrics (message rates, latencies)
- Tracing (OpenTelemetry?)
- Health checks
- Diagnostic tools

---

## Summary

**v1.0 Focus:**
1. ✅ rmw_zenoh interop (Priority 1)
2. ✅ Core pub/sub/service/action functionality
3. ✅ Proper async patterns (no lifecycle)
4. ✅ Namespace isolation for encodings
5. ⏳ Testing infrastructure (Clock, Mocks)
6. ⏳ Documentation

**Future Work:**
1. DDS bridge (if/when available)
2. Multi-language generation (Phase 3)
3. Performance optimization (post-v1.0)
4. Optional lifecycle (if demand emerges)
5. Advanced features (metrics, tracing, etc.)

**Philosophy**: 
- Ship v1.0 with core features done well
- Don't over-engineer before learning from users
- Add complexity only when proven necessary

