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
import hashlib
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
    type_hash: str = ""


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
        
        # Skip constant definitions (lines with = that are not field definitions)
        if '=' in line:
            # Check if this looks like a constant definition
            parts = line.split()
            if len(parts) >= 3:
                # Check if it's a constant like "uint8 DEBUG=10" or "uint8 PARAMETER_NOT_SET=0"
                if parts[1].endswith('=') or (len(parts) > 2 and parts[2] == '='):
                    return None
        
        parts = line.split()
        if len(parts) < 2:
            return None
        
        field_type = parts[0]
        field_name = parts[1]
        
        # Skip if field name contains = (this is a constant definition)
        if '=' in field_name:
            return None
        
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
    
def find_ros2_msg_files() -> List[str]:
    """Find ROS 2 message files from the environment."""
    msg_files = []
    
    # Method 1: Use ROS 2 environment variables
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        # Common ROS 2 installation paths
        possible_paths = [
            f'/opt/ros/{ros_distro}',
            f'/usr/local/ros2',
            f'/opt/ros2',
            os.path.expanduser(f'~/ros2_{ros_distro}'),
            os.path.expanduser('~/ros2_ws/install'),
            os.path.expanduser('~/ros2_ws/install'),
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                for root, dirs, files in os.walk(path):
                    for file in files:
                        if file.endswith('.msg'):
                            msg_files.append(os.path.join(root, file))
    
    # Method 2: Search Python site-packages for ROS 2 packages
    import site
    for site_dir in site.getsitepackages() + [site.getusersitepackages()]:
        if site_dir and os.path.exists(site_dir):
            for item in os.listdir(site_dir):
                if '_msgs' in item and os.path.isdir(os.path.join(site_dir, item)):
                    # Look for .msg files in the package
                    pkg_path = os.path.join(site_dir, item)
                    for root, dirs, files in os.walk(pkg_path):
                        for file in files:
                            if file.endswith('.msg'):
                                msg_files.append(os.path.join(root, file))
    
    # Method 3: Use ros2 pkg list if available
    try:
        import subprocess
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if result.returncode == 0:
            packages = result.stdout.strip().split('\n')
            for pkg in packages:
                if '_msgs' in pkg:
                    try:
                        # Get package path
                        result = subprocess.run(['ros2', 'pkg', 'prefix', pkg], capture_output=True, text=True)
                        if result.returncode == 0:
                            pkg_path = result.stdout.strip()
                            msg_dir = os.path.join(pkg_path, 'share', pkg, 'msg')
                            if os.path.exists(msg_dir):
                                for file in os.listdir(msg_dir):
                                    if file.endswith('.msg'):
                                        msg_files.append(os.path.join(msg_dir, file))
                    except:
                        continue
    except:
        pass
    
    return list(set(msg_files))  # Remove duplicates


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
        
        # Load actual ROS 2 hashes
        self.ros2_hashes = self._load_ros2_hashes()
    
    def _load_ros2_hashes(self) -> Dict[str, str]:
        """Load actual ROS 2 hashes from the extracted file."""
        try:
            # Try to import the generated hashes
            import sys
            import os
            hash_file_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'ros2_interfaces_python')
            sys.path.insert(0, hash_file_path)
            from ros2_message_hashes import ROS2_MESSAGE_HASHES
            print(f"✅ Loaded {len(ROS2_MESSAGE_HASHES)} ROS 2 message hashes from {hash_file_path}")
            return ROS2_MESSAGE_HASHES
        except ImportError:
            # Fallback to some common hashes
            return {
                'geometry_msgs.msg.Twist': 'RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a',
                'geometry_msgs.msg.Vector3': 'RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d',
                'std_msgs.msg.Header': 'RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01',
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
        message_type_key = f"{message_info.package}.msg.{class_name}"
        ros2_hash = self.ros2_hashes.get(message_type_key, "RIHS01_" + "0" * 64)
        
        # Try alternative naming convention with underscores if hash not found
        if ros2_hash == "RIHS01_" + "0" * 64:
            # Convert CamelCase to snake_case for ROS 2 naming convention
            import re
            snake_case_name = re.sub(r'(?<!^)(?=[A-Z])', '_', class_name)
            alt_message_type_key = f"{message_info.package}.msg.{snake_case_name}"
            ros2_hash = self.ros2_hashes.get(alt_message_type_key, "RIHS01_" + "0" * 64)
        
        class_code = f'''@dataclass
class {class_name}:
    """Simplified {message_info.package}.msg.{class_name} message."""
    
    # ROS message type hash (from actual ROS 2 generated code)
    TYPE_HASH: str = "{ros2_hash}"
'''
        
        if field_definitions:
            class_code += '\n'.join(field_definitions)
            
            # Add __post_init__ method for None fields
            none_fields = []
            for field in message_info.fields:
                if not field.is_builtin and field.type not in self.builtin_types:
                    none_fields.append(field)
            
            if none_fields:
                class_code += "\n    \n    def __post_init__(self):\n"
                for field in none_fields:
                    field_type = self._get_base_python_type(field.type)
                    class_code += f"        if self.{field.name} is None:\n"
                    class_code += f"            self.{field.name} = {field_type}()\n"
        else:
            class_code += "    pass"
        
        return class_code
    
    def _get_python_type(self, field: FieldInfo) -> str:
        """Get the Python type for a field."""
        if field.is_array:
            base_type = self._get_base_python_type(field.type)
            # Use string annotations for non-builtin types to avoid forward reference issues
            if not field.is_builtin and field.type not in self.builtin_types:
                return f"List['{base_type}']"
            else:
                return f"List[{base_type}]"
        else:
            base_type = self._get_base_python_type(field.type)
            # Use string annotations for non-builtin types to avoid forward reference issues
            if not field.is_builtin and field.type not in self.builtin_types:
                return f"Optional['{base_type}']"
            return base_type
    
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
        
        # Handle cross-package dependencies
        if '/' in ros_type:
            pkg, msg_type = ros_type.split('/', 1)
            return msg_type  # Just return the message type name
        
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
            # For custom types, use None and handle in __post_init__
            return "None"
    
    def generate_package_module(self, messages: List[MessageInfo], package_name: str) -> str:
        """Generate a complete Python module for a package."""
        imports = [
            f'"""Simplified {package_name} types"""',
            "",
            "from dataclasses import dataclass, field",
            "from typing import List, Optional, Dict, Any",
            "",
            "# Import dependencies",
        ]
        
        # Add dependency imports
        dependencies = set()
        for msg in messages:
            dependencies.update(msg.dependencies)
        
        # Add builtin type imports
        if any('time' in dep for dep in dependencies):
            imports.append("from ...builtin_interfaces.msg.builtin_interfaces import Time")
        if any('duration' in dep for dep in dependencies):
            imports.append("from ...builtin_interfaces.msg.builtin_interfaces import Duration")
        
        # Add std_msgs imports
        if any('std_msgs' in dep for dep in dependencies):
            imports.append("from ...std_msgs.msg.std_msgs import Header")
        
        # Add other package dependencies
        for dep in sorted(dependencies):
            if dep not in ['time', 'duration', 'std_msgs/Header']:
                if '/' in dep:
                    # Handle cross-package dependencies
                    pkg, msg_type = dep.split('/', 1)
                    if pkg != package_name:  # Don't import from same package
                        imports.append(f"from ...{pkg}.msg.{pkg} import {msg_type}")
                else:
                    # Don't import from same package
                    pass
        
        imports.append("")
        
        # Generate classes
        classes = []
        for msg in messages:
            classes.append(self.generate_message_class(msg))
            classes.append("")  # Empty line between classes
        
        # Generate __all__ list
        class_names = [msg.name for msg in messages]
        all_list = f"__all__ = {class_names}"
        
        # Combine everything
        module_content = '\n'.join(imports + classes + [all_list])
        
        return module_content


def main():
    parser = argparse.ArgumentParser(description='Parse ROS 2 interface packages and generate simplified types')
    parser.add_argument('--input', '-i',
                       help='Input directory containing ROS 2 interface packages (optional, will auto-detect if not provided)')
    parser.add_argument('--output', '-o', default='generated_types',
                       help='Output directory for generated Python files')
    parser.add_argument('--package', '-p',
                       help='Specific package to process (if not specified, processes all)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Auto-detect ROS 2 message files if no input directory specified
    if args.input:
        if not os.path.exists(args.input):
            print(f"Error: Input directory '{args.input}' does not exist")
            sys.exit(1)
        msg_files = []
        for root, dirs, files in os.walk(args.input):
            for file in files:
                if file.endswith('.msg'):
                    msg_files.append(os.path.join(root, file))
    else:
        print("Auto-detecting ROS 2 message files...")
        msg_files = find_ros2_msg_files()
        if not msg_files:
            print("Error: No ROS 2 message files found. Please specify --input directory or ensure ROS 2 is properly installed.")
            sys.exit(1)
        print(f"Found {len(msg_files)} message files from auto-detection")
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    parser_obj = ROS2InterfaceParser()
    generator = SimplifiedTypeGenerator()
    
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
            
            # Create ROS 2 standard directory structure
            package_dir = os.path.join(args.output, package_name)
            msg_dir = os.path.join(package_dir, 'msg')
            os.makedirs(msg_dir, exist_ok=True)
            
            # Write package __init__.py
            package_init = f'# {package_name} package\n'
            with open(os.path.join(package_dir, '__init__.py'), 'w') as f:
                f.write(package_init)
            
            # Write msg __init__.py
            msg_init = f'# {package_name}.msg package\n'
            with open(os.path.join(msg_dir, '__init__.py'), 'w') as f:
                f.write(msg_init)
            
            # Write the main message file
            output_file = os.path.join(msg_dir, f"{package_name}.py")
            with open(output_file, 'w') as f:
                f.write(module_content)
            
            print(f"  Generated: {output_file}")
    
    print(f"\nGenerated simplified types in: {args.output}")
    print("To use these types, copy them to your ros2_zenoh_python package or import them directly.")


if __name__ == '__main__':
    main()
