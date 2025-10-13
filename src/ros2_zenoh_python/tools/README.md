# ROS 2 Interface Parser and Simplified Type Generator

This directory contains tools for parsing ROS 2 interface packages and generating simplified Python types based on message definitions.

## Tools

### 1. `generate_simplified_types.py`

A comprehensive parser that processes entire ROS 2 interface packages and generates Python modules with simplified dataclasses.

**Features:**
- Parses all `.msg` files in a directory tree
- Handles cross-package dependencies
- Generates proper imports and type annotations
- Supports arrays and bounded arrays
- Handles builtin types and custom message types

**Usage:**
```bash
# Generate simplified types for geometry_msgs package
python3 generate_simplified_types.py --input /opt/ros/jazzy/share/geometry_msgs/msg --output generated_types --package geometry_msgs --verbose

# Generate types for all packages in a directory
python3 generate_simplified_types.py --input /opt/ros/jazzy/share --output generated_types --verbose
```

### 2. `simple_type_generator.py`

A simpler, more focused generator that creates clean simplified types for specific message files or packages.

### 3. `integrate_types.py`

A tool for integrating generated types into the existing `ros2_zenoh_python.messages` module.

**Features:**
- Integrates generated classes into existing messages.py
- Updates `__all__` lists automatically
- Supports dry-run mode for preview
- Handles multiple packages

**Usage:**
```bash
# Integrate generated types into messages module
python3 integrate_types.py --package geometry_msgs --generated-file generated_types/geometry_msgs_messages.py

# Preview integration without making changes
python3 integrate_types.py --package geometry_msgs --generated-file generated_types/geometry_msgs_messages.py --dry-run
```

### 4. `integrate_workflow.py`

A complete workflow script that generates and integrates types for multiple packages in one command.

**Features:**
- End-to-end workflow from package list to integrated types
- Supports both individual packages and package lists
- Dry-run mode for testing
- Automatic cleanup of temporary files

**Usage:**
```bash
# Complete workflow for multiple packages
python3 integrate_workflow.py --packages geometry_msgs std_msgs nav_msgs

# Use existing package list file
python3 integrate_workflow.py --package-list package_list.txt

# Preview workflow without making changes
python3 integrate_workflow.py --packages geometry_msgs std_msgs --dry-run

# Create a package list file
python3 integrate_workflow.py --packages geometry_msgs std_msgs nav_msgs --create-list --list-file my_packages.txt
```

## Integration Workflow

The recommended approach for adding new ROS 2 message types to `ros2_zenoh_python`:

1. **Generate Integration Code**: Use `simple_type_generator.py` with `--integrate` flag
2. **Integrate into Module**: Use `integrate_types.py` to merge into `messages.py`
3. **Update Exports**: Update `__init__.py` to export new types
4. **Test**: Verify the integrated types work correctly

**Quick Start:**
```bash
# Complete workflow for common packages
python3 integrate_workflow.py --packages geometry_msgs std_msgs nav_msgs --dry-run

# If satisfied with preview, run without --dry-run
python3 integrate_workflow.py --packages geometry_msgs std_msgs nav_msgs
```

**Usage:**

#### **Process ROS 2 Package by Name (Recommended)**
```bash
# Generate simplified types for a single ROS 2 package
python3 simple_type_generator.py --package geometry_msgs --output-dir generated_types --verbose

# Generate types for std_msgs package
python3 simple_type_generator.py --package std_msgs --output-dir generated_types
```

#### **Batch Process Multiple Packages**
```bash
# Create a package list file
echo -e "geometry_msgs\nstd_msgs\nnav_msgs" > package_list.txt

# Process all packages in the list
python3 simple_type_generator.py --package-list package_list.txt --output-dir generated_types --verbose
```

#### **Generate Integration Code for Module Integration**
```bash
# Generate code ready for integration into ros2_zenoh_python.messages
python3 simple_type_generator.py --package geometry_msgs --integrate --output-dir generated_types

# Batch generate integration code for multiple packages
python3 simple_type_generator.py --package-list package_list.txt --integrate --output-dir generated_types
```

#### **Process Individual Files (Legacy)**
```bash
# Generate simplified types for a single message file
python3 simple_type_generator.py --input /opt/ros/jazzy/share/geometry_msgs/msg/Twist.msg --output twist_simplified.py --verbose

# Generate simplified types for all messages in a directory
python3 simple_type_generator.py --input /opt/ros/jazzy/share/geometry_msgs/msg --output geometry_msgs_simplified.py --verbose
```

## Package List File Format

When using `--package-list`, create a text file with one package name per line:

```
# Common ROS 2 message packages
# This file contains a list of ROS 2 packages to process for simplified types

# Core geometry messages
geometry_msgs

# Standard messages
std_msgs

# Built-in interfaces
builtin_interfaces

# Navigation messages
nav_msgs

# Sensor messages
sensor_msgs
```

**Rules:**
- One package name per line
- Lines starting with `#` are comments and ignored
- Empty lines are ignored
- Package names should match exactly what's installed in your ROS 2 system

## Generated Code Examples

### Input: `Twist.msg`
```
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```

### Output: `twist_simplified.py`
```python
from dataclasses import dataclass, field
from typing import List

# Common ROS 2 message types converted to simplified Python dataclasses

@dataclass
class Twist:
    """Simplified geometry_msgs.msg.Twist message."""
    linear: Vector3 = Vector3()
    angular: Vector3 = Vector3()
```

## Integration with ros2_zenoh_python

The generated simplified types can be easily integrated with the `ros2_zenoh_python` package:

1. **Copy generated files** to your project directory
2. **Import the simplified types** in your code
3. **Use with ros2_zenoh_python** publishers and subscribers

**Example:**
```python
from ros2_zenoh_python import Node, Publisher
from geometry_msgs_simplified import Twist, Vector3

# Create a node
with Node("my_node") as node:
    # Create publisher using simplified types
    pub = node.create_publisher(Twist, "/turtle1/cmd_vel")
    
    # Create and publish a message
    msg = Twist(
        linear=Vector3(x=1.0, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=0.5)
    )
    pub.publish(msg)
```

## Benefits

1. **No ROS 2 Dependencies**: Generated types don't require ROS 2 installation
2. **Clean API**: Simple dataclasses with proper type hints
3. **Easy Integration**: Works seamlessly with `ros2_zenoh_python`
4. **Automatic Generation**: Parse any ROS 2 package and generate types
5. **Type Safety**: Full Python type annotations for better IDE support

## Limitations

- Cross-package dependencies require manual handling
- Complex message types may need additional customization
- Generated types are simplified versions, not full ROS 2 message implementations
- Some advanced ROS 2 features (like constants) are not supported

## Future Enhancements

- Support for service definitions (`.srv` files)
- Support for action definitions (`.action` files)
- Automatic dependency resolution
- Integration with ROS 2 message generation tools
- Support for message constants and default values
