---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/generate_unified_types.py"
last_generated: "2025-10-23T16:51:15.353233"
tags: ["api", "python", "auto-generated"]
---

# generate_unified_types

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/generate_unified_types.py`

Unified ROS 2 CDR Types Generator

Generates a single package per language containing all ROS 2 interface types:
- Python: ros2-interfaces-py (pip package: ros2-interfaces-py)
- Rust: ros2_interfaces_rs (crate: ros2_interfaces_rs)
- TypeScript: @ros2-cdr/interfaces-ts (npm package)
- C: ros2_interfaces_c (CMake project)

Uses Jinja2 templates for all code generation.

## Functions

### `main`

```python
main()
```

