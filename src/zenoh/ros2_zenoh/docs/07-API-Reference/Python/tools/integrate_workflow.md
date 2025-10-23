---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/integrate_workflow.py"
last_generated: "2025-10-23T16:51:15.372918"
tags: ["api", "python", "auto-generated"]
---

# integrate_workflow

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/integrate_workflow.py`

Complete Integration Workflow for ros2_zenoh_python

This script provides a complete workflow for generating and integrating
simplified types from ROS 2 packages into the ros2_zenoh_python module.

## Functions

### `run_command`

```python
run_command(cmd, description)
```

Run a command and handle errors.

### `generate_and_integrate_packages`

```python
generate_and_integrate_packages(package_list_file: str, output_dir: str, messages_module: str, dry_run: bool)
```

Generate and integrate types for multiple packages.

### `create_package_list_file`

```python
create_package_list_file(packages: list, filename: str)
```

Create a package list file.

### `main`

```python
main()
```

