---
# Core Identity
id: pattern:{{PATTERN_NAME}}
title: {{PATTERN_TITLE}}
type: reference
updated: {{DATE}}

# Classification & Governance
classification: public
llm_processing: cloud-ok
license: CC-BY-4.0
author: Development Team

# Discoverability
summary: {{ONE_SENTENCE_SUMMARY}}
tags: [pattern, {{TAG1}}, {{TAG2}}]

# Pattern-Specific
category: pattern
version: "1.0"
status: draft
derived_from: {{SOURCE_CODE_PATH}}
languages: [python, rust, c, typescript]

# Pattern Metadata
complexity: simple | moderate | complex
use_cases:
  - When you need X
  - When you have constraint Y
anti_patterns:
  - Don't do X (use Y instead)
related_patterns: []

# Quality Attributes
performance: O(1) | O(n) | O(log n)
memory: low | medium | high
testability: high | medium | low

# Future-Proofing
schema_version: "1.0"
---

# {{PATTERN_TITLE}}

## Purpose

Why this pattern exists and when to use it.

## Problem

What problem does this pattern solve? What's the context?

## Solution

High-level description of the solution approach.

## Structure

```typescript
// TypeScript + YAML IR showing the structure
interface ComponentStructure {
  state: {
    field: Type;
  };
  inputs: {
    topic: MessageType;
  };
  outputs: {
    topic: MessageType;
  };
  logic: {
    on_input(msg: MessageType): void;
  };
}
```

## Implementation

### Python

```python
from dataclasses import dataclass
from ros2_zenoh_python import Node
from ros2_zenoh_python.testing import Clock

@dataclass(frozen=True)
class State:
    """Immutable state"""
    field: int = 0

class ExampleComponent:
    """
    Example implementation following this pattern.
    
    This is a COMPLETE, RUNNABLE example.
    """
    def __init__(self, node: Node, clock: Clock):
        self._node = node
        self._clock = clock
        self._state = State()
        
    async def setup(self):
        """Setup I/O"""
        self._input_sub = self._node.create_subscription(
            MessageType, 
            "/input", 
            self._on_input
        )
        self._output_pub = self._node.create_publisher(
            MessageType, 
            "/output"
        )
        
    async def teardown(self):
        """Cleanup"""
        pass
        
    async def _on_input(self, msg: MessageType):
        """Handle input"""
        # Update state (create new immutable state)
        new_state = State(field=self._state.field + 1)
        self._state = new_state
        
        # Produce output
        self._output_pub.publish(MessageType(data="result"))

# Usage
async def main():
    async with Node("example") as node:
        clock = Clock(TimeMode.WALL_TIME)
        component = ExampleComponent(node, clock)
        await component.setup()
        
        # Run...
        
        await component.teardown()
```

### Rust

```rust
// Rust implementation
// TODO: Add when Rust library is ready
```

### C

```c
// C implementation
// TODO: Add when C library is ready
```

### TypeScript

```typescript
// TypeScript implementation
// TODO: Add when TS library is ready
```

## Testing Strategy

### How to Test Components Following This Pattern

```python
import pytest
from ros2_zenoh_python.testing import MockNode, Clock, TimeMode

async def test_example_component():
    """Test the component with mocks"""
    clock = Clock(TimeMode.TEST)
    
    async with MockNode("test_node", clock=clock) as node:
        component = ExampleComponent(node, clock)
        await component.setup()
        
        # Create mock I/O
        input_pub = node.create_publisher(MessageType, "/input")
        output_sub = node.create_subscription(
            MessageType, 
            "/output", 
            output_callback
        )
        
        # Send input
        input_pub.publish(MessageType(data="test"))
        
        # Advance time to process
        clock.advance_by(Duration(seconds=0.1))
        
        # Verify output received
        assert output_received
        
        await component.teardown()
```

## Variations

### Variation 1: Stateful Version

Changes needed for components with complex state...

### Variation 2: Multi-Input Version

Changes needed for components with multiple input topics...

## Trade-offs

### Pros
- ✅ Advantage 1
- ✅ Advantage 2

### Cons
- ❌ Limitation 1
- ❌ Limitation 2

## When to Use

Use this pattern when:
- ✅ Condition 1
- ✅ Condition 2

Don't use this pattern when:
- ❌ Condition 3 (use [[AlternativePattern]] instead)
- ❌ Condition 4

## Anti-Patterns

### ❌ Anti-Pattern 1: Blocking Callbacks

**Don't:**
```python
def callback(msg):
    time.sleep(1.0)  # Blocks event loop!
```

**Do:**
```python
async def callback(msg):
    await clock.sleep(Duration(seconds=1.0))
```

### ❌ Anti-Pattern 2: Mutable State

**Don't:**
```python
self.state.field = new_value  # Mutation!
```

**Do:**
```python
self.state = State(field=new_value)  # New object
```

## Related Patterns

- [[BasicComponent]] - Simpler version
- [[StatefulComponent]] - More complex version
- [[TestingWithMocks]] - How to test this

## References

- Derived from: `{{SOURCE_CODE_PATH}}`
- Used in: [[ExampleApplication]]
- Concept: [[RelevantConcept]]

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | {{DATE}} | Initial version |


