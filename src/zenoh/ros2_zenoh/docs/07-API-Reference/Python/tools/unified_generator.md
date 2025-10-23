---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/unified_generator.py"
last_generated: "2025-10-23T16:51:15.364953"
tags: ["api", "python", "auto-generated"]
---

# unified_generator

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/unified_generator.py`

Unified ROS 2 Message Generator and Integrator

This script generates simplified ROS 2 message types and integrates them directly
into the ros2_zenoh_python package in one step.

## Functions

### `integrate_into_package`

```python
integrate_into_package(package_name: str, generated_content: str, target_package_dir: str)
```

Integrate generated content directly into the ros2_zenoh_python package.

### `update_package_init`

```python
update_package_init(package_name: str, message_classes: list, target_package_dir: str)
```

Update the main package __init__.py to export the new message types.

### `main`

```python
main()
```

