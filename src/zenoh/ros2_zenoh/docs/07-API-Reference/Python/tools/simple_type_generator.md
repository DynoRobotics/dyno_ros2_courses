---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "tools/simple_type_generator.py"
last_generated: "2025-10-23T16:51:15.288760"
tags: ["api", "python", "auto-generated"]
---

# simple_type_generator

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `tools/simple_type_generator.py`

Simple ROS 2 Message Parser and Generator

This script parses specific ROS 2 message files and generates clean simplified Python types.

## Functions

### `parse_msg_file`

```python
parse_msg_file(file_path: str) -> MessageInfo
```

Parse a .msg file and extract message information.

### `parse_field_line`

```python
parse_field_line(line: str) -> Optional[FieldInfo]
```

Parse a single field line from a .msg file.

### `get_python_type`

```python
get_python_type(ros_type: str, is_array: bool) -> str
```

Convert ROS type to Python type.

### `get_default_value`

```python
get_default_value(ros_type: str, is_array: bool) -> str
```

Get default value for a field.

### `generate_message_class`

```python
generate_message_class(message_info: MessageInfo) -> str
```

Generate a simplified Python class for a message.

### `generate_simple_types`

```python
generate_simple_types(messages: List[MessageInfo]) -> str
```

Generate simplified types for common messages.

### `generate_module_integration`

```python
generate_module_integration(messages: List[MessageInfo], package_name: str) -> str
```

Generate code that can be integrated into ros2_zenoh_python.messages module.

### `find_ros2_package_path`

```python
find_ros2_package_path(package_name: str) -> Optional[str]
```

Find the path to a ROS 2 package's msg directory.

### `find_msg_files_from_package`

```python
find_msg_files_from_package(package_name: str) -> List[str]
```

Find all .msg files for a ROS 2 package.

### `process_package_list`

```python
process_package_list(package_list_file: str) -> List[str]
```

Process a file containing a list of package names.

### `main`

```python
main()
```

