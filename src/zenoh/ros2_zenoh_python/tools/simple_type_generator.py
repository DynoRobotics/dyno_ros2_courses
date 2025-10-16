#!/usr/bin/env python3
"""
Simple ROS 2 Message Parser and Generator

This script parses specific ROS 2 message files and generates clean simplified Python types.
"""

import os
import sys
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class FieldInfo:
    """Information about a message field."""
    name: str
    type: str
    is_array: bool = False
    array_size: Optional[int] = None


@dataclass
class MessageInfo:
    """Information about a ROS 2 message."""
    name: str
    package: str
    fields: List[FieldInfo] = None
    file_path: str = ""


def parse_msg_file(file_path: str) -> MessageInfo:
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
        file_path=file_path,
        fields=[]
    )
    
    # Parse fields
    lines = content.strip().split('\n')
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        
        field_info = parse_field_line(line)
        if field_info:
            message_info.fields.append(field_info)
    
    return message_info


def parse_field_line(line: str) -> Optional[FieldInfo]:
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
            except ValueError:
                pass
        
        # Remove array brackets from type
        field_type = field_type[:start]
    
    return FieldInfo(
        name=field_name,
        type=field_type,
        is_array=is_array,
        array_size=array_size
    )


def get_python_type(ros_type: str, is_array: bool = False) -> str:
    """Convert ROS type to Python type."""
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
    
    python_type = type_mappings.get(ros_type, ros_type)
    
    if is_array:
        return f"List[{python_type}]"
    else:
        return python_type


def get_default_value(ros_type: str, is_array: bool = False) -> str:
    """Get default value for a field."""
    if is_array:
        return "field(default_factory=list)"
    elif ros_type in ['bool']:
        return "False"
    elif ros_type in ['string', 'wstring']:
        return '""'
    elif ros_type in ['float32', 'float64']:
        return "0.0"
    elif ros_type in ['time', 'duration']:
        return f"{ros_type}()"
    elif ros_type in ['Vector3', 'Point', 'Quaternion', 'Pose', 'Transform', 'Accel', 'Wrench']:
        return f"{ros_type}()"
    else:
        return "0"


def generate_message_class(message_info: MessageInfo) -> str:
    """Generate a simplified Python class for a message."""
    class_name = message_info.name
    
    # Generate field definitions
    field_definitions = []
    for field in message_info.fields:
        python_type = get_python_type(field.type, field.is_array)
        default_value = get_default_value(field.type, field.is_array)
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


def generate_simple_types(messages: List[MessageInfo]) -> str:
    """Generate simplified types for common messages."""
    
    # Generate imports
    imports = [
        "from dataclasses import dataclass, field",
        "from typing import List",
        "",
        "# Common ROS 2 message types converted to simplified Python dataclasses",
        ""
    ]
    
    # Generate classes
    classes = []
    for msg in messages:
        classes.append(generate_message_class(msg))
        classes.append("")  # Empty line between classes
    
    # Combine everything
    module_content = '\n'.join(imports + classes)
    
    return module_content


def generate_module_integration(messages: List[MessageInfo], package_name: str) -> str:
    """Generate code that can be integrated into ros2_zenoh_python.messages module."""
    
    # Generate imports
    imports = [
        "from dataclasses import dataclass, field",
        "from typing import List",
        "",
        f"# Generated simplified types for {package_name} package",
        ""
    ]
    
    # Generate classes
    classes = []
    for msg in messages:
        classes.append(generate_message_class(msg))
        classes.append("")  # Empty line between classes
    
    # Add export list
    export_list = [msg.name for msg in messages]
    exports = [
        "",
        f"# Export all {package_name} message types",
        f"__all__ = {export_list}",
        ""
    ]
    
    # Combine everything
    module_content = '\n'.join(imports + classes + exports)
    
    return module_content


def find_ros2_package_path(package_name: str) -> Optional[str]:
    """Find the path to a ROS 2 package's msg directory."""
    # Common ROS 2 installation paths
    ros2_paths = [
        "/opt/ros/jazzy/share",
        "/opt/ros/humble/share", 
        "/opt/ros/galactic/share",
        "/opt/ros/foxy/share",
        "/usr/share",
        "/usr/local/share"
    ]
    
    for ros2_path in ros2_paths:
        if os.path.exists(ros2_path):
            package_path = os.path.join(ros2_path, package_name, "msg")
            if os.path.exists(package_path):
                return package_path
    
    return None


def find_msg_files_from_package(package_name: str) -> List[str]:
    """Find all .msg files for a ROS 2 package."""
    package_path = find_ros2_package_path(package_name)
    if not package_path:
        return []
    
    msg_files = []
    for file in os.listdir(package_path):
        if file.endswith('.msg'):
            msg_files.append(os.path.join(package_path, file))
    
    return msg_files


def process_package_list(package_list_file: str) -> List[str]:
    """Process a file containing a list of package names."""
    packages = []
    try:
        with open(package_list_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    packages.append(line)
    except Exception as e:
        print(f"Error reading package list file: {e}")
        sys.exit(1)
    
    return packages


def main():
    parser = argparse.ArgumentParser(description='Generate simplified types from ROS 2 message files')
    parser.add_argument('--package', '-p', 
                       help='ROS 2 package name (e.g., geometry_msgs)')
    parser.add_argument('--input', '-i',
                       help='Input .msg file or directory containing .msg files')
    parser.add_argument('--package-list', '-l',
                       help='File containing list of package names to process')
    parser.add_argument('--output', '-o', default='simplified_types.py',
                       help='Output Python file')
    parser.add_argument('--output-dir', '-d', default='src/ros2_interfaces_python/ros2_interfaces_python/msg',
                       help='Output directory for generated files')
    parser.add_argument('--integrate', action='store_true',
                       help='Generate code for integration into ros2_zenoh_python.messages module')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Determine input method
    if args.package_list:
        # Process multiple packages from a list file
        packages = process_package_list(args.package_list)
        print(f"Processing {len(packages)} packages from list file")
        
        for package_name in packages:
            print(f"\nProcessing package: {package_name}")
            msg_files = find_msg_files_from_package(package_name)
            
            if not msg_files:
                print(f"✗ No .msg files found for package '{package_name}'")
                continue
            
            print(f"Found {len(msg_files)} .msg files")
            
            # Parse messages
            messages = []
            for msg_file in msg_files:
                if args.verbose:
                    print(f"  Parsing: {msg_file}")
                
                try:
                    message_info = parse_msg_file(msg_file)
                    messages.append(message_info)
                    print(f"  ✓ {message_info.name} ({len(message_info.fields)} fields)")
                except Exception as e:
                    print(f"  ✗ Error parsing {msg_file}: {e}")
            
            if messages:
                # Generate Python module
                if args.integrate:
                    module_content = generate_module_integration(messages, package_name)
                    output_file = os.path.join(args.output_dir, f"{package_name}_messages.py")
                else:
                    module_content = generate_simple_types(messages)
                    output_file = os.path.join(args.output_dir, f"{package_name}_simplified.py")
                
                # Write to file
                with open(output_file, 'w') as f:
                    f.write(module_content)
                
                print(f"  Generated: {output_file}")
        
        print(f"\nAll packages processed. Generated files in: {args.output_dir}")
        
    elif args.package:
        # Process single package by name
        package_name = args.package
        print(f"Processing package: {package_name}")
        
        msg_files = find_msg_files_from_package(package_name)
        if not msg_files:
            print(f"Error: Package '{package_name}' not found or has no .msg files")
            print("Available packages in common locations:")
            for ros2_path in ["/opt/ros/jazzy/share", "/opt/ros/humble/share"]:
                if os.path.exists(ros2_path):
                    packages = [d for d in os.listdir(ros2_path) 
                              if os.path.isdir(os.path.join(ros2_path, d))]
                    print(f"  {ros2_path}: {', '.join(sorted(packages)[:10])}...")
            sys.exit(1)
        
        print(f"Found {len(msg_files)} .msg files")
        
        # Parse messages
        messages = []
        for msg_file in msg_files:
            if args.verbose:
                print(f"Parsing: {msg_file}")
            
            try:
                message_info = parse_msg_file(msg_file)
                messages.append(message_info)
                print(f"✓ {message_info.name} ({len(message_info.fields)} fields)")
            except Exception as e:
                print(f"✗ Error parsing {msg_file}: {e}")
        
        if messages:
            # Generate Python module
            if args.integrate:
                module_content = generate_module_integration(messages, package_name)
                output_file = os.path.join(args.output_dir, f"{package_name}_messages.py")
                print(f"\nGenerated module integration code in: {output_file}")
                print("To integrate into ros2_zenoh_python.messages:")
                print(f"1. Copy the generated classes to ros2_zenoh_python/messages.py")
                print(f"2. Add the types to the __all__ list in messages.py")
                print(f"3. Import them in ros2_zenoh_python/__init__.py")
            else:
                module_content = generate_simple_types(messages)
                output_file = os.path.join(args.output_dir, f"{package_name}_simplified.py")
                print(f"\nGenerated simplified types in: {output_file}")
                print("To use these types, import them in your ros2_zenoh_python code:")
                print(f"from {package_name}_simplified import {', '.join([msg.name for msg in messages])}")
            
            with open(output_file, 'w') as f:
                f.write(module_content)
    
    elif args.input:
        # Process file or directory (legacy method)
        msg_files = []
        if os.path.isfile(args.input) and args.input.endswith('.msg'):
            msg_files = [args.input]
        elif os.path.isdir(args.input):
            for root, dirs, files in os.walk(args.input):
                for file in files:
                    if file.endswith('.msg'):
                        msg_files.append(os.path.join(root, file))
        else:
            print(f"Error: Input '{args.input}' is not a .msg file or directory")
            sys.exit(1)
        
        if not msg_files:
            print("No .msg files found")
            sys.exit(1)
        
        print(f"Found {len(msg_files)} .msg files")
        
        # Parse messages
        messages = []
        for msg_file in msg_files:
            if args.verbose:
                print(f"Parsing: {msg_file}")
            
            try:
                message_info = parse_msg_file(msg_file)
                messages.append(message_info)
                print(f"✓ {message_info.name} ({len(message_info.fields)} fields)")
            except Exception as e:
                print(f"✗ Error parsing {msg_file}: {e}")
        
        if messages:
            # Generate Python module
            module_content = generate_simple_types(messages)
            
            # Write to file
            with open(args.output, 'w') as f:
                f.write(module_content)
            
            print(f"\nGenerated simplified types in: {args.output}")
            print("To use these types, import them in your ros2_zenoh_python code:")
            print(f"from {Path(args.output).stem} import {', '.join([msg.name for msg in messages])}")
    
    else:
        print("Error: Must specify either --package, --input, or --package-list")
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main()
