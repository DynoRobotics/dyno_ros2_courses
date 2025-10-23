---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/generate_simplified_types.py"
last_generated: "2025-10-23T16:51:15.386993"
tags: ["api", "python", "auto-generated"]
---

# generate_simplified_types

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/generate_simplified_types.py`

ROS 2 Interface Parser and Simplified Type Generator

This script parses ROS 2 interface packages and generates simplified Python types
based on the message definitions found in .msg files.

## Functions

### `find_ros2_msg_files`

```python
find_ros2_msg_files() -> List[str]
```

Find ROS 2 message files from the environment.

### `main`

```python
main()
```

