---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "examples/action_server_fibonacci.py"
last_generated: "2025-10-23T16:51:15.430206"
tags: ["api", "python", "auto-generated"]
---

# action_server_fibonacci

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `examples/action_server_fibonacci.py`

Fibonacci Action Server Example

This example demonstrates how to create an action server using ros2_zenoh_python.
The server accepts Fibonacci sequence requests and provides feedback during execution.

## Functions

### `goal_callback`

```python
goal_callback(goal)
```

Decide whether to accept or reject a goal request.

Args:
    goal: The goal request
    
Returns:
    GoalResponse.ACCEPT or GoalResponse.REJECT

