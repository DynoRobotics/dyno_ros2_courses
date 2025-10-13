#!/usr/bin/env python3
"""
ROS 2 Interface Parser and Simplified Type Generator

This script parses ROS 2 interface packages and generates simplified Python types
based on the message definitions found in .msg files.
"""

import os
import sys
import argparse
import re
from pathlib import Path
from typing import Dict, List, Set, Optional, Any
from dataclasses import dataclass, field


@dataclass
class FieldInfo:
    """Information about a message field."""
    name: str
    type: str
    array_size: Optional[int] = None
    is_array: bool = False
    is_bounded_array: bool = False
    is_string: bool = False
    is_builtin: bool = False
    default_value: Optional[str] = None


@dataclass
class MessageInfo:
    """Information about a ROS 2 message."""
    name: str
    package: str
    fields: List[FieldInfo] = field(default_factory=list)
    dependencies: Set[str] = field(default_factory=set)
    file_path: str = ""


class ROS2InterfaceParser:
    """Parser for ROS 2 interface files."""
    
    def __init__(self):
        self.builtin_types = {
            'bool', 'byte', 'char', 'wchar', 'uint8', 'uint16', 'uint32', 'uint64',
            'int8', 'int16', 'int32', 'int64', 'float32', 'float64', 'string',
            'wstring', 'time', 'duration'
        }
        
        self.type_mappings = {
            'bool': 'bool',
            'byte': 'int',
            'char': 'str',
            'wchar': 'str',
            'uint8': 'int',
            'uint16': 'int',
            'uint32': 'int',
            'uint64': 'int',
            'int8': 'int',
            'int16': 'int',
            'int32': 'int',
            'int64': 'int',
            'float32': 'float',
            'float64': 'float',
            'string': 'str',
            'wstring': 'str',
            'time': 'Time',
            'duration': 'Duration'
        }
    
    def parse_msg_file(self, file_path: str) -> MessageInfo:
        """Parse a .msg file and extract message information."""
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Extract package name from path
        path_parts = Path(file_path).parts
        package_name = None
        for i, part in enumerate(path_parts):
            if part == 'msg' and i > 0:
                package_name = path_parts[i-1]
                break
        
        if not package_name:
            package_name = "unknown"
        
        # Extract message name from filename
        message_name = Path(file_path).stem
        
        message_info = MessageInfo(
            name=message_name,
            package=package_name,
            file_path=file_path
        )
        
        # Parse fields
        lines = content.strip().split('\n')
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            field_info = self._parse_field_line(line)
            if field_info:
                message_info.fields.append(field_info)
                
                # Track dependencies
                if not field_info.is_builtin and field_info.type not in self.builtin_types:
                    # Handle cross-package dependencies
                    if '/' in field_info.type:
                        message_info.dependencies.add(field_info.type)
                    else:
                        message_info.dependencies.add(field_info.type)
        
        return message_info
    
    def _parse_field_line(self, line: str) -> Optional[FieldInfo]:
        """Parse a single field line from a .msg file."""
        # Remove comments
        if '#' in line:
            line = line[:line.index('#')]
        
        line = line.strip()
        if not line:
            return None
        
        parts = line.split()
        if len(parts) < 2:
            return None
        
        field_type = parts[0]
        field_name = parts[1]
        
        # Handle arrays
        is_array = False
        is_bounded_array = False
        array_size = None
        
        if '[' in field_type and ']' in field_type:
            is_array = True
            # Extract array size
            start = field_type.find('[')
            end = field_type.find(']')
            size_str = field_type[start+1:end]
            
            if size_str:
                try:
                    array_size = int(size_str)
                    is_bounded_array = True
                except ValueError:
                    pass
            
            # Remove array brackets from type
            field_type = field_type[:start]
        
        # Check if it's a builtin type
        is_builtin = field_type in self.builtin_types
        is_string = field_type in ['string', 'wstring']
        
        return FieldInfo(
            name=field_name,
            type=field_type,
            array_size=array_size,
            is_array=is_array,
            is_bounded_array=is_bounded_array,
            is_string=is_string,
            is_builtin=is_builtin
        )
    
    def find_msg_files(self, directory: str) -> List[str]:
        """Find all .msg files in a directory tree."""
        msg_files = []
        for root, dirs, files in os.walk(directory):
            for file in files:
                if file.endswith('.msg'):
                    msg_files.append(os.path.join(root, file))
        return msg_files


class SimplifiedTypeGenerator:
    """Generator for simplified Python types."""
    
    def __init__(self):
        self.generated_types = set()
        self.type_dependencies = {}
        self.builtin_types = {
            'bool', 'byte', 'char', 'wchar', 'uint8', 'uint16', 'uint32', 'uint64',
            'int8', 'int16', 'int32', 'int64', 'float32', 'float64', 'string',
            'wstring', 'time', 'duration'
        }
    
    def generate_message_class(self, message_info: MessageInfo) -> str:
        """Generate a simplified Python class for a message."""
        class_name = message_info.name
        
        # Generate field definitions
        field_definitions = []
        for field in message_info.fields:
            python_type = self._get_python_type(field)
            default_value = self._get_default_value(field)
            field_definitions.append(f"    {field.name}: {python_type} = {default_value}")
        
        # Generate the class
        class_code = f'''@dataclass
class {class_name}:
    """Simplified {message_info.package}.msg.{class_name} message."""
'''
        
        if field_definitions:
            class_code += '\n'.join(field_definitions)
        else:
            class_code += "    pass"
        
        return class_code
    
    def _get_python_type(self, field: FieldInfo) -> str:
        """Get the Python type for a field."""
        if field.is_array:
            if field.is_bounded_array:
                return f"List[{self._get_base_python_type(field.type)}]"
            else:
                return f"List[{self._get_base_python_type(field.type)}]"
        else:
            return self._get_base_python_type(field.type)
    
    def _get_base_python_type(self, ros_type: str) -> str:
        """Get the base Python type for a ROS type."""
        type_mappings = {
            'bool': 'bool',
            'byte': 'int',
            'char': 'str',
            'wchar': 'str',
            'uint8': 'int',
            'uint16': 'int',
            'uint32': 'int',
            'uint64': 'int',
            'int8': 'int',
            'int16': 'int',
            'int32': 'int',
            'int64': 'int',
            'float32': 'float',
            'float64': 'float',
            'string': 'str',
            'wstring': 'str',
            'time': 'Time',
            'duration': 'Duration'
        }
        
        return type_mappings.get(ros_type, ros_type)
    
    def _get_default_value(self, field: FieldInfo) -> str:
        """Get the default value for a field."""
        if field.is_array:
            return "field(default_factory=list)"
        elif field.type in ['bool']:
            return "False"
        elif field.type in ['string', 'wstring']:
            return '""'
        elif field.type in ['time', 'duration']:
            return f"{self._get_base_python_type(field.type)}()"
        elif field.type in ['float32', 'float64']:
            return "0.0"
        elif field.type in self.builtin_types:
            return "0"
        else:
            # For custom types, create an instance
            if '/' in field.type:
                # Cross-package dependency
                pkg, msg_type = field.type.split('/', 1)
                return f"{msg_type}()"
            else:
                return f"{field.type}()"
    
    def generate_package_module(self, messages: List[MessageInfo], package_name: str) -> str:
        """Generate a complete Python module for a package."""
        imports = [
            "from dataclasses import dataclass, field",
            "from typing import List, Optional",
            "",
            "# Import dependencies",
        ]
        
        # Add dependency imports
        dependencies = set()
        for msg in messages:
            dependencies.update(msg.dependencies)
        
        # Add builtin type imports
        if any('time' in dep for dep in dependencies):
            imports.append("from builtin_interfaces.msg import Time")
        if any('duration' in dep for dep in dependencies):
            imports.append("from builtin_interfaces.msg import Duration")
        
        # Add std_msgs imports
        if any('std_msgs' in dep for dep in dependencies):
            imports.append("from std_msgs.msg import Header")
        
        # Add other package dependencies
        for dep in sorted(dependencies):
            if dep not in ['time', 'duration', 'std_msgs/Header']:
                if '/' in dep:
                    # Handle cross-package dependencies
                    pkg, msg_type = dep.split('/', 1)
                    imports.append(f"from {pkg}.msg import {msg_type}")
                else:
                    imports.append(f"from .{dep.lower()} import {dep}")
        
        imports.append("")
        
        # Generate classes
        classes = []
        for msg in messages:
            classes.append(self.generate_message_class(msg))
            classes.append("")  # Empty line between classes
        
        # Combine everything
        module_content = '\n'.join(imports + classes)
        
        return module_content


def main():
    parser = argparse.ArgumentParser(description='Parse ROS 2 interface packages and generate simplified types')
    parser.add_argument('--input', '-i', required=True,
                       help='Input directory containing ROS 2 interface packages')
    parser.add_argument('--output', '-o', default='generated_types',
                       help='Output directory for generated Python files')
    parser.add_argument('--package', '-p',
                       help='Specific package to process (if not specified, processes all)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"Error: Input directory '{args.input}' does not exist")
        sys.exit(1)
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    parser_obj = ROS2InterfaceParser()
    generator = SimplifiedTypeGenerator()
    
    # Find all .msg files
    msg_files = parser_obj.find_msg_files(args.input)
    
    if args.package:
        # Filter by specific package
        msg_files = [f for f in msg_files if f'/{args.package}/' in f]
    
    if not msg_files:
        print("No .msg files found")
        sys.exit(1)
    
    print(f"Found {len(msg_files)} .msg files")
    
    # Group messages by package
    packages = {}
    for msg_file in msg_files:
        path_parts = Path(msg_file).parts
        package_name = None
        for i, part in enumerate(path_parts):
            if part == 'msg' and i > 0:
                package_name = path_parts[i-1]
                break
        
        if package_name:
            if package_name not in packages:
                packages[package_name] = []
            packages[package_name].append(msg_file)
    
    # Process each package
    for package_name, package_msg_files in packages.items():
        print(f"\nProcessing package: {package_name}")
        
        messages = []
        for msg_file in package_msg_files:
            if args.verbose:
                print(f"  Parsing: {msg_file}")
            
            try:
                message_info = parser_obj.parse_msg_file(msg_file)
                messages.append(message_info)
                print(f"  ✓ {message_info.name} ({len(message_info.fields)} fields)")
            except Exception as e:
                print(f"  ✗ Error parsing {msg_file}: {e}")
        
        if messages:
            # Generate Python module
            module_content = generator.generate_package_module(messages, package_name)
            
            # Write to file
            output_file = os.path.join(args.output, f"{package_name}.py")
            with open(output_file, 'w') as f:
                f.write(module_content)
            
            print(f"  Generated: {output_file}")
    
    print(f"\nGenerated simplified types in: {args.output}")
    print("To use these types, copy them to your ros2_zenoh_python package or import them directly.")


if __name__ == '__main__':
    main()
