---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/multi_lang_generator.py"
last_generated: "2025-10-23T16:51:15.407394"
tags: ["api", "python", "auto-generated"]
---

# multi_lang_generator

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/multi_lang_generator.py`

Multi-Language ROS 2 Message Generator

This script generates ROS 2 message types for multiple languages with proper dependency handling:
- Python (dataclasses with CDR serialization)
- Rust (structs with serde and CDR serialization)  
- C (structs with Micro-CDR serialization)
- TypeScript (interfaces with JSON serialization)

It also generates conversion functions between languages, especially for Tauri applications.

## Functions

### `main`

```python
main()
```

