---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/package_generator.py"
last_generated: "2025-10-23T16:51:15.266942"
tags: ["api", "python", "auto-generated"]
---

# package_generator

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/package_generator.py`

Template-Based ROS 2 Package Generator

This script generates proper packages for multiple languages using Jinja2 templates:
- Python package with setup.py, __init__.py, proper namespace structure
- Rust crate with Cargo.toml, lib.rs, proper module structure  
- NPM package with package.json, proper TypeScript structure
- C library with CMakeLists.txt, proper header organization

Uses templates for clean separation of code generation logic and templates.

## Functions

### `main`

```python
main()
```

