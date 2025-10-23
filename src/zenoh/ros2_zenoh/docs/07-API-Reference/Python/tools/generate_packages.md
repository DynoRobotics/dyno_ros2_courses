---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/generate_packages.py"
last_generated: "2025-10-23T16:51:15.304591"
tags: ["api", "python", "auto-generated"]
---

# generate_packages

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/generate_packages.py`

Template-Based ROS 2 Package Generator

Generates proper packages for multiple languages using Jinja2 templates.
Output structure:
  output/
    python/
      geometry_msgs/
      std_msgs/
      sensor_msgs/
    rust/
      geometry_msgs/
      std_msgs/
      sensor_msgs/
    typescript/
      geometry_msgs/
      std_msgs/
      sensor_msgs/

## Functions

### `main`

```python
main()
```

