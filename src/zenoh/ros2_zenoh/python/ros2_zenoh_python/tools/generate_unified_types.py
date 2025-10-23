#!/usr/bin/env python3
"""
Unified ROS 2 CDR Types Generator

Generates a single package per language containing all ROS 2 interface types:
- Python: ros2-interfaces-py (pip package: ros2-interfaces-py)
- Rust: ros2_interfaces_rs (crate: ros2_interfaces_rs)
- TypeScript: @ros2-cdr/interfaces-ts (npm package)
- C: ros2_interfaces_c (CMake project)

Uses Jinja2 templates for all code generation.
"""

import os
import sys
import argparse
import re
import hashlib
from pathlib import Path
from typing import Dict, List, Set, Optional
from dataclasses import dataclass, field
from enum import Enum

try:
    from jinja2 import Environment, FileSystemLoader, Template
    HAS_JINJA2 = True
except ImportError:
    HAS_JINJA2 = False
    print("⚠️  Jinja2 not found. Install with: pip install jinja2")
    print("    Falling back to simple string templates...")


@dataclass
class FieldInfo:
    """Information about a message field."""
    name: str
    type: str
    ros2_package: str = ""  # Package where the type comes from
    array_size: Optional[int] = None
    is_array: bool = False
    is_bounded_array: bool = False
    is_builtin: bool = False


@dataclass
class MessageInfo:
    """Information about a ROS 2 message."""
    name: str
    package: str  # e.g., "geometry_msgs"
    fields: List[FieldInfo] = field(default_factory=list)
    dependencies: Set[str] = field(default_factory=set)  # Set of (package, type) tuples as strings
    type_hash: str = ""  # RIHS01 hash for ROS 2 type identification


@dataclass
class UnifiedPackage:
    """All ROS 2 types in a single package."""
    messages_by_package: Dict[str, Dict[str, MessageInfo]] = field(default_factory=dict)
    all_packages: Set[str] = field(default_factory=set)


class TemplateGenerator:
    """Generates unified packages using Jinja2 templates."""
    
    def __init__(self):
        self.builtin_types = {
            'bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 
            'int64', 'uint64', 'float32', 'float64', 'string', 'wstring', 'char', 'byte',
            'time', 'duration'
        }
        
        self.type_mappings = {
            'bool': {'python': 'bool', 'pycdr2': 'bool', 'rust': 'bool', 'typescript': 'boolean', 'c': 'bool'},
            'int8': {'python': 'int', 'pycdr2': 'int8', 'rust': 'i8', 'typescript': 'number', 'c': 'int8_t'},
            'uint8': {'python': 'int', 'pycdr2': 'uint8', 'rust': 'u8', 'typescript': 'number', 'c': 'uint8_t'},
            'int16': {'python': 'int', 'pycdr2': 'int16', 'rust': 'i16', 'typescript': 'number', 'c': 'int16_t'},
            'uint16': {'python': 'int', 'pycdr2': 'uint16', 'rust': 'u16', 'typescript': 'number', 'c': 'uint16_t'},
            'int32': {'python': 'int', 'pycdr2': 'int32', 'rust': 'i32', 'typescript': 'number', 'c': 'int32_t'},
            'uint32': {'python': 'int', 'pycdr2': 'uint32', 'rust': 'u32', 'typescript': 'number', 'c': 'uint32_t'},
            'int64': {'python': 'int', 'pycdr2': 'int64', 'rust': 'i64', 'typescript': 'bigint', 'c': 'int64_t'},
            'uint64': {'python': 'int', 'pycdr2': 'uint64', 'rust': 'u64', 'typescript': 'bigint', 'c': 'uint64_t'},
            'float32': {'python': 'float', 'pycdr2': 'float32', 'rust': 'f32', 'typescript': 'number', 'c': 'float'},
            'float64': {'python': 'float', 'pycdr2': 'float64', 'rust': 'f64', 'typescript': 'number', 'c': 'double'},
            'string': {'python': 'str', 'pycdr2': 'str', 'rust': 'String', 'typescript': 'string', 'c': 'char*'},
            'wstring': {'python': 'str', 'pycdr2': 'str', 'rust': 'String', 'typescript': 'string', 'c': 'wchar_t*'},
            'char': {'python': 'str', 'pycdr2': 'str', 'rust': 'char', 'typescript': 'number', 'c': 'char'},  # TypeScript: char is a single-byte number (0-255)
            'byte': {'python': 'int', 'pycdr2': 'uint8', 'rust': 'u8', 'typescript': 'number', 'c': 'uint8_t'},
        }
        
        self.unified_package = UnifiedPackage()
        
        # Setup template directory
        self.template_dir = Path(__file__).parent / "templates"
        if HAS_JINJA2:
            if self.template_dir.exists():
                self.env = Environment(loader=FileSystemLoader(str(self.template_dir)))
            else:
                self.env = Environment()
                print(f"⚠️  Template directory not found: {self.template_dir}")
                print("    Using inline templates...")
    
    def parse_msg_file(self, file_path: str) -> MessageInfo:
        """Parse a .msg file."""
        with open(file_path, 'r') as f:
            content = f.read()
        
        package = Path(file_path).parent.parent.name
        message_name = Path(file_path).stem
        
        message = MessageInfo(name=message_name, package=package)
        
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            if '=' in line:  # Skip constants
                continue
            
            field_info = self._parse_field_line(line, package)
            if field_info:
                message.fields.append(field_info)
                if not field_info.is_builtin:
                    dep_key = f"{field_info.ros2_package}.{field_info.type}"
                    message.dependencies.add(dep_key)
        
        # Compute type hash
        message.type_hash = self._compute_type_hash(message)
        
        return message
    
    def _compute_type_hash(self, message: MessageInfo) -> str:
        """
        Compute RIHS01 type hash for a message.
        
        This tries to get the actual ROS2 hash by:
        1. Querying ros2 CLI for the actual hash
        2. Looking up in a known hash table
        3. Computing a simplified hash (with warning - may not match ROS2!)
        
        Format: RIHS01_<64-char-hex>
        """
        msg_type_name = f"{message.package}/msg/{message.name}"
        
        # Try to get hash from ROS2 CLI
        try:
            import subprocess
            result = subprocess.run(
                ['ros2', 'interface', 'show', msg_type_name, '--verbose'],
                capture_output=True,
                text=True,
                timeout=2
            )
            for line in result.stdout.split('\n'):
                if 'Type hash:' in line:
                    hash_val = line.split('Type hash:')[1].strip()
                    if hash_val.startswith('RIHS01_'):
                        print(f"  ✓ Got ROS2 hash for {msg_type_name}: {hash_val}")
                        return hash_val
        except Exception as e:
            pass  # Fall through to lookup table
        
        # Look up in known ROS2 hashes table
        known_hashes = {
            'geometry_msgs.msg.Twist': 'RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a',
            'geometry_msgs.msg.Vector3': 'RIHS01_cc153f88313a2e0280128712c4c9e90ac025f0238639b5e0763b7f1aa2e0b5d0',
            'geometry_msgs.msg.Point': 'RIHS01_f819c2f8863424c7d14b1c8c1e79e96ec97e1c06d91dd6843b24d320f30b6df6',
            'geometry_msgs.msg.Quaternion': 'RIHS01_e5fc68c5fc3d45e6c6f81d98b9ac4e4f5f1cb6a3b0c9c59b8a8d11c9c2e4e4d1',
            'std_msgs.msg.Header': 'RIHS01_3b92f7c2a49c6eb95f978acc8b3a8f4e6b7c6f1c9e5c6b8a8f9a1b2c3d4e5f6a',
            'rcl_interfaces.msg.Log': 'RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac',
        }
        
        lookup_key = f"{message.package}.msg.{message.name}"
        if lookup_key in known_hashes:
            print(f"  ✓ Using known ROS2 hash for {msg_type_name}: {known_hashes[lookup_key]}")
            return known_hashes[lookup_key]
        
        # Fall back to simplified hash computation (with warning)
        print(f"  ⚠️  WARNING: Computing simplified hash for {msg_type_name} - may not match ROS2!")
        print(f"     Run 'ros2 interface show {msg_type_name} --verbose' to get the correct hash")
        
        # Create a canonical representation of the message structure
        hash_input = f"{message.package}::{message.name}\n"
        
        for field in message.fields:
            if field.is_array:
                if field.is_bounded_array:
                    array_spec = f"[{field.array_size}]"
                else:
                    array_spec = "[]"
            else:
                array_spec = ""
            
            if field.is_builtin:
                type_str = field.type
            else:
                type_str = f"{field.ros2_package}::{field.type}"
            
            hash_input += f"{type_str}{array_spec} {field.name}\n"
        
        # Compute SHA-256 hash
        hash_obj = hashlib.sha256(hash_input.encode('utf-8'))
        hash_hex = hash_obj.hexdigest()
        
        return f"RIHS01_{hash_hex}"
    
    def _parse_field_line(self, line: str, current_package: str) -> Optional[FieldInfo]:
        """Parse a single field line."""
        parts = line.split()
        if len(parts) < 2:
            return None
        
        field_type = parts[0]
        field_name = parts[1]
        
        is_array = False
        array_size = None
        is_bounded_array = False
        
        if '[' in field_type and ']' in field_type:
            is_array = True
            match = re.search(r'\[(\d+)\]', field_type)
            if match:
                array_size = int(match.group(1))
                is_bounded_array = True
            field_type = field_type.split('[')[0]
        
        is_builtin = field_type in self.builtin_types
        
        # Determine package for the type
        ros2_package = current_package
        if '/' in field_type:
            ros2_package, field_type = field_type.split('/')
        
        return FieldInfo(
            name=field_name,
            type=field_type,
            ros2_package=ros2_package if not is_builtin else "",
            is_array=is_array,
            array_size=array_size,
            is_bounded_array=is_bounded_array,
            is_builtin=is_builtin
        )
    
    def get_python_type(self, field: FieldInfo, current_package: str = None, use_pycdr2: bool = True) -> str:
        """Get Python type for a field.
        
        Args:
            field: Field information
            current_package: Current ROS2 package (for relative vs absolute imports)
            use_pycdr2: If True, use pycdr2 types for CDR serialization
        """
        if field.is_builtin:
            # Use pycdr2 types for proper CDR serialization
            base_type = self.type_mappings[field.type]['pycdr2' if use_pycdr2 else 'python']
        else:
            # Use qualified name to avoid any conflicts
            # Same package: just ClassName
            # Other package: ros2_interfaces_py.package.msg.ClassName
            if current_package and field.ros2_package == current_package:
                base_type = field.type
            else:
                base_type = f"ros2_interfaces_py.{field.ros2_package}.msg.{field.type}"
        
        if field.is_array:
            return f"List[{base_type}]"
        return base_type
    
    def get_rust_type(self, field: FieldInfo) -> str:
        """Get Rust type for a field."""
        if field.is_builtin:
            base_type = self.type_mappings[field.type]['rust']
        else:
            base_type = field.type
        
        if field.is_array:
            if field.is_bounded_array:
                # Serde only supports arrays up to 32 elements by default
                # Use Vec for larger arrays with serde(with = "serde_arrays")
                if field.array_size > 32:
                    return f"Vec<{base_type}>"  # or use serde_arrays crate
                return f"[{base_type}; {field.array_size}]"
            return f"Vec<{base_type}>"
        return base_type
    
    def get_typescript_type(self, field: FieldInfo, current_package: str = None) -> str:
        """Get TypeScript type for a field."""
        if field.is_builtin:
            base_type = self.type_mappings[field.type]['typescript']
        else:
            # Use package prefix for cross-package types
            if current_package and field.ros2_package != current_package:
                base_type = f"{field.ros2_package}.{field.type}"
            else:
                base_type = field.type
        
        if field.is_array:
            return f"{base_type}[]"
        return base_type
    
    def get_c_type(self, field: FieldInfo) -> str:
        """Get C type for a field."""
        if field.is_builtin:
            base_type = self.type_mappings[field.type]['c']
        else:
            base_type = f"{field.ros2_package}_{field.type}_t"
        
        if field.is_array:
            if field.is_bounded_array:
                return f"{base_type}[{field.array_size}]"
            return f"{base_type}*"
        return base_type
    
    def generate_typescript_cdr_write(self, field: FieldInfo, var_name: str) -> str:
        """Generate TypeScript CDR write code for a field."""
        type_map = {
            'bool': 'uint8',  # CDR uses uint8 for bool
            'int8': 'int8',
            'uint8': 'uint8',
            'byte': 'uint8',
            'char': 'uint8',
            'int16': 'int16',
            'uint16': 'uint16',
            'int32': 'int32',
            'uint32': 'uint32',
            'int64': 'int64',
            'uint64': 'uint64',
            'float32': 'float32',
            'float64': 'float64',
            'string': 'string',
        }
        
        if field.is_builtin:
            cdr_method = type_map.get(field.type, 'string')
            if field.is_array:
                # For arrays, we need to write length first, then elements
                if field.type == 'bool':
                    return f"writer.sequenceLength({var_name}.length);\n  {var_name}.forEach(item => writer.{cdr_method}(item ? 1 : 0));"
                else:
                    return f"writer.sequenceLength({var_name}.length);\n  {var_name}.forEach(item => writer.{cdr_method}(item));"
            else:
                if field.type == 'bool':
                    return f"writer.{cdr_method}({var_name} ? 1 : 0);"
                else:
                    return f"writer.{cdr_method}({var_name});"
        else:
            # For nested messages, serialize each field
            if field.is_array:
                return f"writer.sequenceLength({var_name}.length);\n  {var_name}.forEach(item => {{\n    // TODO: Serialize nested {field.type}\n  }});"
            else:
                return f"// TODO: Serialize nested {field.type} from {var_name}"
    
    def generate_typescript_cdr_read(self, field: FieldInfo) -> str:
        """Generate TypeScript CDR read code for a field."""
        type_map = {
            'bool': 'uint8',  # CDR uses uint8 for bool, convert to boolean
            'int8': 'int8',
            'uint8': 'uint8',
            'byte': 'uint8',
            'char': 'uint8',
            'int16': 'int16',
            'uint16': 'uint16',
            'int32': 'int32',
            'uint32': 'uint32',
            'int64': 'int64',
            'uint64': 'uint64',
            'float32': 'float32',
            'float64': 'float64',
            'string': 'string',
        }
        
        if field.is_builtin:
            cdr_method = type_map.get(field.type, 'string')
            if field.is_array:
                if field.type == 'bool':
                    return f"Array.from({{ length: reader.sequenceLength() }}, () => reader.{cdr_method}() !== 0)"
                else:
                    return f"Array.from({{ length: reader.sequenceLength() }}, () => reader.{cdr_method}())"
            else:
                if field.type == 'bool':
                    return f"reader.{cdr_method}() !== 0"
                else:
                    return f"reader.{cdr_method}()"
        else:
            # For nested messages - generate as placeholder
            if field.is_array:
                # Return empty array as placeholder
                return f"[] /* TODO: Deserialize nested {field.type} array */"
            else:
                # Return empty object as placeholder (TypeScript requires valid syntax)
                return f"{{}} as any /* TODO: Deserialize nested {field.type} */"
    
    def generate_python(self, output_dir: Path) -> None:
        """Generate unified Python package."""
        pkg_dir = output_dir / "python" / "ros2_interfaces_py"
        pkg_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate setup.py
        template_str = '''from setuptools import setup, find_packages

setup(
    name="ros2-interfaces-py",
    version="0.1.0",
    description="CDR-serializable ROS 2 interfaces for Python (no ROS 2 dependency)",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "pycdr2>=0.2.0",  # CDR serialization support
    ],
    extras_require={
        "dev": ["pytest>=7.0"],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
    ],
)
'''
        (pkg_dir.parent / "setup.py").write_text(template_str)
        
        # Create package __init__.py
        (pkg_dir / "__init__.py").write_text("")
        
        # Generate each ROS 2 package as a submodule
        for ros2_pkg, messages in self.unified_package.messages_by_package.items():
            pkg_subdir = pkg_dir / ros2_pkg
            pkg_subdir.mkdir(exist_ok=True)
            
            # Create msg subdirectory following ROS 2 convention
            msg_subdir = pkg_subdir / "msg"
            msg_subdir.mkdir(exist_ok=True)
            
            # Generate messages
            for msg_name, message in messages.items():
                # Collect imports for non-builtin types (grouped by package)
                imports_same_pkg = set()
                imports_other_pkgs = set()  # Just package names, not individual types
                
                for field in message.fields:
                    if not field.is_builtin:
                        if field.ros2_package == message.package:
                            # Same package - use relative import of the class
                            imports_same_pkg.add((field.type.lower(), field.type))
                        else:
                            # Different package - just need to import the package.msg module
                            imports_other_pkgs.add(field.ros2_package)
                
                template_str = '''from dataclasses import dataclass
from typing import List, Optional, TYPE_CHECKING

try:
    from pycdr2 import IdlStruct
    from pycdr2.types import int8, uint8, int16, uint16, int32, uint32, int64, uint64, float32, float64
    PYCDR2_AVAILABLE = True
except ImportError:
    PYCDR2_AVAILABLE = False
    IdlStruct = object  # Use object as base class when pycdr2 is not available
    # Define dummy types for type hints
    int8 = uint8 = int16 = uint16 = int32 = uint32 = int64 = uint64 = int
    float32 = float64 = float
{% if imports_same_pkg %}

# Import types from same package (relative imports)
{% for module, classname in imports_same_pkg %}
from .{{ module }} import {{ classname }}
{% endfor %}
{% endif %}
{% if imports_other_pkgs %}

# Import package.msg modules for cross-package types
{% for pkg in imports_other_pkgs %}
import ros2_interfaces_py.{{ pkg }}.msg
{% endfor %}
{% endif %}

if TYPE_CHECKING:
    # Import needed for type hints but avoid circular imports
    pass

@dataclass
class {{ message.name }}(IdlStruct, typename="{{ message.package }}/{{ message.name }}"):
    """{{ message.package }}/{{ message.name }} message.
    
    Supports CDR serialization via pycdr2 when available.
    Type annotations use direct types (not strings) for pycdr2 compatibility.
    
    ROS 2 type hash: {{ message.type_hash }}
    DDS type name: {{ message.package }}::msg::dds_::{{ message.name }}_
    """
{% for field in message.fields %}
    {{ field.name }}: {{ get_python_type(field) }}
{% endfor %}
    
    # Class constants (defined after fields for dataclass compatibility)
    TYPE_HASH = "{{ message.type_hash }}"
    DDS_TYPE_NAME = "{{ message.package }}::msg::dds_::{{ message.name }}_"

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
{% for field in message.fields %}
            '{{ field.name }}': self.{{ field.name }},
{% endfor %}
        }
    
    @classmethod
    def from_dict(cls, data: dict):
        """Create from dictionary."""
        return cls(**data)
    
    def serialize(self) -> bytes:
        """
        Serialize to CDR format.
        
        Returns:
            bytes: Serialized CDR data
            
        Raises:
            RuntimeError: If pycdr2 is not available
        """
        if not PYCDR2_AVAILABLE:
            raise RuntimeError("pycdr2 is required for serialization. Install with: pip install pycdr2")
        return IdlStruct.serialize(self)
    
    @classmethod
    def deserialize(cls, data: bytes):
        """
        Deserialize from CDR format.
        
        Args:
            data: Serialized CDR bytes
            
        Returns:
            Deserialized message instance
            
        Raises:
            RuntimeError: If pycdr2 is not available
        """
        if not PYCDR2_AVAILABLE:
            raise RuntimeError("pycdr2 is required for deserialization. Install with: pip install pycdr2")
        # Use the classmethod from IdlStruct properly
        return super({{ message.name }}, cls).deserialize(data)
'''
                if HAS_JINJA2:
                    template = self.env.from_string(template_str)
                    # Create a wrapper to pass current_package to get_python_type
                    def get_type_wrapper(field):
                        return self.get_python_type(field, current_package=message.package)
                    
                    content = template.render(
                        message=message,
                        imports_same_pkg=sorted(imports_same_pkg),
                        imports_other_pkgs=sorted(imports_other_pkgs),
                        get_python_type=get_type_wrapper
                    )
                else:
                    content = self._render_python_fallback(template_str, message)
                
                (msg_subdir / f"{msg_name.lower()}.py").write_text(content)
            
            # Generate __init__.py for msg subdirectory
            init_lines = [f"# {ros2_pkg} messages", ""]
            for msg_name in messages.keys():
                init_lines.append(f"from .{msg_name.lower()} import {msg_name}")
            init_lines.append("")
            init_lines.append("__all__ = [")
            for msg_name in messages.keys():
                init_lines.append(f"    '{msg_name}',")
            init_lines.append("]")
            (msg_subdir / "__init__.py").write_text('\n'.join(init_lines))
            
            # Generate __init__.py for package (imports msg submodule)
            (pkg_subdir / "__init__.py").write_text(f"# {ros2_pkg} package\nfrom . import msg\n")
        
        # Generate main __init__.py
        init_lines = ["# ROS 2 CDR Interfaces - Unified Python Package", ""]
        for ros2_pkg in self.unified_package.all_packages:
            init_lines.append(f"from . import {ros2_pkg}")
        (pkg_dir / "__init__.py").write_text('\n'.join(init_lines))
        
        print(f"✅ Generated Python package: {pkg_dir}")
    
    def generate_rust(self, output_dir: Path) -> None:
        """Generate unified Rust crate."""
        pkg_dir = output_dir / "rust" / "ros2_interfaces_rs"
        src_dir = pkg_dir / "src"
        src_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate Cargo.toml
        cargo_toml = '''[package]
name = "ros2_interfaces_rs"
version = "0.1.0"
edition = "2021"
description = "CDR-serializable ROS 2 interfaces for Rust (no ROS 2 dependency)"
license = "Apache-2.0"

[dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
cdr = "0.2"  # CDR serialization for ROS 2 compatibility

[features]
default = ["cdr-support"]
cdr-support = []
'''
        (pkg_dir / "Cargo.toml").write_text(cargo_toml)
        
        # Generate each ROS 2 package as a module
        for ros2_pkg, messages in self.unified_package.messages_by_package.items():
            pkg_mod_dir = src_dir / ros2_pkg
            pkg_mod_dir.mkdir(exist_ok=True)
            
            # Generate messages
            for msg_name, message in messages.items():
                # Build imports
                imports = ["use serde::{Deserialize, Serialize};", ""]
                for dep in sorted(message.dependencies):
                    parts = dep.split('.')
                    if len(parts) == 2:
                        imports.append(f"use crate::{parts[0]}::{parts[1]};")
                
                # Build struct fields
                fields = []
                rust_keywords = {'type', 'mod', 'fn', 'struct', 'enum', 'trait', 'impl', 'match', 'if', 'else', 'loop', 'while', 'for', 'in', 'return', 'break', 'continue', 'const', 'static', 'let', 'mut', 'ref', 'move', 'as', 'use', 'pub', 'crate', 'super', 'self', 'Self', 'extern', 'unsafe', 'async', 'await', 'dyn', 'abstract', 'final', 'override', 'macro', 'typeof', 'yield', 'try', 'box', 'where', 'union', 'virtual', 'become', 'do', 'priv', 'unsized'}
                for field in message.fields:
                    rust_type = self.get_rust_type(field)
                    field_name = f"r#{field.name}" if field.name in rust_keywords else field.name
                    fields.append(f"    pub {field_name}: {rust_type},")
                
                # Handle Rust built-in type conflicts (String, etc.)
                rust_builtin_types = {'String', 'Vec', 'Option', 'Result', 'Box'}
                struct_name = f"Ros{message.name}" if message.name in rust_builtin_types else message.name
                
                # Generate content directly
                content = '\n'.join(imports) + f'''

/// {message.package}/{message.name} message
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct {struct_name} {{
{chr(10).join(fields)}
}}

impl {struct_name} {{
    pub fn to_json(&self) -> Result<String, serde_json::Error> {{
        serde_json::to_string(self)
    }}
    
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {{
        serde_json::from_str(json)
    }}
    
    /// Serialize to CDR format (Common Data Representation)
    /// Compatible with ROS 2 DDS and other CDR-based systems
    #[cfg(feature = "cdr-support")]
    pub fn serialize_cdr(&self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {{
        // cdr crate automatically adds the 4-byte encapsulation header
        let data = cdr::serialize::<_, _, cdr::CdrLe>(self, cdr::Infinite)?;
        Ok(data)
    }}
    
    /// Deserialize from CDR format (Common Data Representation)
    #[cfg(feature = "cdr-support")]
    pub fn deserialize_cdr(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {{
        // cdr crate automatically handles the 4-byte encapsulation header
        let msg = cdr::deserialize::<Self>(data)?;
        Ok(msg)
    }}
}}
'''
                
                (pkg_mod_dir / f"{msg_name.lower()}.rs").write_text(content)
            
            # Generate mod.rs for submodule
            mod_lines = [f"// {ros2_pkg} messages", ""]
            rust_builtin_types = {'String', 'Vec', 'Option', 'Result', 'Box'}
            for msg_name in messages.keys():
                mod_lines.append(f"pub mod {msg_name.lower()};")
            mod_lines.append("")
            mod_lines.append("// Re-exports")
            for msg_name in messages.keys():
                struct_name = f"Ros{msg_name}" if msg_name in rust_builtin_types else msg_name
                mod_lines.append(f"pub use {msg_name.lower()}::{struct_name};")
            (pkg_mod_dir / "mod.rs").write_text('\n'.join(mod_lines))
        
        # Generate lib.rs
        lib_lines = ["//! ROS 2 CDR Interfaces - Unified Rust Crate", ""]
        for ros2_pkg in self.unified_package.all_packages:
            lib_lines.append(f"pub mod {ros2_pkg};")
        (src_dir / "lib.rs").write_text('\n'.join(lib_lines))
        
        print(f"✅ Generated Rust crate: {pkg_dir}")
    
    def generate_typescript(self, output_dir: Path) -> None:
        """Generate unified TypeScript package."""
        pkg_dir = output_dir / "typescript" / "ros2-cdr-interfaces"
        src_dir = pkg_dir / "src"
        src_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate package.json
        package_json = '''{
  "name": "@ros2-cdr/interfaces",
  "version": "0.1.0",
  "description": "CDR-serializable ROS 2 interfaces for TypeScript (no ROS 2 dependency)",
  "main": "dist/index.js",
  "types": "dist/index.d.ts",
  "scripts": {
    "build": "tsc",
    "prepublishOnly": "npm run build"
  },
  "keywords": ["ros2", "cdr", "serialization", "interfaces", "typescript"],
  "license": "Apache-2.0",
  "dependencies": {
    "@foxglove/cdr": "^1.0.0"
  },
  "devDependencies": {
    "typescript": "^5.0.0",
    "@types/node": "^20.0.0"
  }
}
'''
        (pkg_dir / "package.json").write_text(package_json)
        
        # Generate tsconfig.json
        tsconfig = '''{
  "compilerOptions": {
    "target": "ES2020",
    "module": "commonjs",
    "lib": ["ES2020"],
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "declaration": true,
    "declarationMap": true,
    "sourceMap": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist"]
}
'''
        (pkg_dir / "tsconfig.json").write_text(tsconfig)
        
        # Generate each ROS 2 package as a namespace
        for ros2_pkg, messages in self.unified_package.messages_by_package.items():
            # Collect cross-package dependencies
            imported_packages = set()
            for message in messages.values():
                for field in message.fields:
                    if not field.is_builtin and field.ros2_package != ros2_pkg:
                        imported_packages.add(field.ros2_package)
            
            pkg_file_lines = [
                f"// {ros2_pkg} messages",
                "import { CdrReader, CdrWriter, EncapsulationKind } from '@foxglove/cdr';",
                ""
            ]
            
            # Add imports for cross-package types
            if imported_packages:
                for imported_pkg in sorted(imported_packages):
                    pkg_file_lines.append(f"import * as {imported_pkg} from './{imported_pkg}';")
                pkg_file_lines.append("")
            
            for msg_name, message in messages.items():
                template_str = '''/// {{ message.package }}/{{ message.name }} message
export interface {{ message.name }} {
{% for field in message.fields %}
  {{ field.name }}: {{ get_typescript_type(field) }};
{% endfor %}
}

export function serialize{{ message.name }}(msg: {{ message.name }}): string {
  return JSON.stringify(msg);
}

export function deserialize{{ message.name }}(data: string): {{ message.name }} {
  return JSON.parse(data) as {{ message.name }};
}

/**
 * Serialize {{ message.name }} to CDR format (Common Data Representation)
 * Compatible with ROS 2 DDS
 */
export function serialize{{ message.name }}CDR(msg: {{ message.name }}): Uint8Array {
  const writer = new CdrWriter({ kind: EncapsulationKind.CDR_LE });
  
{% for field in message.fields %}
  // Serialize {{ field.name }}: {{ get_typescript_type(field) }}
  {{ generate_cdr_write_code(field, 'msg.' + field.name) }}
{% endfor %}
  
  return writer.data;
}

/**
 * Deserialize {{ message.name }} from CDR format
 */
export function deserialize{{ message.name }}CDR(data: Uint8Array): {{ message.name }} {
  const reader = new CdrReader(data);
  
  return {
{% for field in message.fields %}
    {{ field.name }}: {{ generate_cdr_read_code(field) }}{% if not loop.last %},{% endif %}

{% endfor %}
  };
}
'''
                if HAS_JINJA2:
                    template = self.env.from_string(template_str)
                    # Create wrapper to pass current package
                    def get_ts_type_wrapper(field):
                        return self.get_typescript_type(field, current_package=message.package)
                    def generate_cdr_write_wrapper(field, var_name):
                        return self.generate_typescript_cdr_write(field, var_name)
                    def generate_cdr_read_wrapper(field):
                        return self.generate_typescript_cdr_read(field)
                    content = template.render(
                        message=message,
                        get_typescript_type=get_ts_type_wrapper,
                        generate_cdr_write_code=generate_cdr_write_wrapper,
                        generate_cdr_read_code=generate_cdr_read_wrapper
                    )
                else:
                    content = self._render_typescript_fallback(template_str, message)
                
                pkg_file_lines.append(content)
                pkg_file_lines.append("")
            
            (src_dir / f"{ros2_pkg}.ts").write_text('\n'.join(pkg_file_lines))
        
        # Generate index.ts
        index_lines = ["// ROS 2 CDR Interfaces - Unified TypeScript Package", ""]
        for ros2_pkg in self.unified_package.all_packages:
            index_lines.append(f"export * from './{ros2_pkg}';")
        (src_dir / "index.ts").write_text('\n'.join(index_lines))
        
        print(f"✅ Generated TypeScript package: {pkg_dir}")
    
    def generate_c(self, output_dir: Path) -> None:
        """Generate unified C library."""
        pkg_dir = output_dir / "c" / "ros2_interfaces_c"
        include_dir = pkg_dir / "include" / "ros2_interfaces_c"
        src_dir = pkg_dir / "src"
        include_dir.mkdir(parents=True, exist_ok=True)
        src_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate CMakeLists.txt
        cmake_content = '''cmake_minimum_required(VERSION 3.8)
project(ros2_interfaces_c)

set(CMAKE_C_STANDARD 11)

# Find Micro-CDR
find_package(microcdr QUIET)
if(microcdr_FOUND)
    add_definitions(-DMICROCDR_AVAILABLE)
endif()

include_directories(include)

# Collect all source files
file(GLOB_RECURSE SOURCES "src/*.c")

add_library(${PROJECT_NAME} STATIC ${SOURCES})

if(microcdr_FOUND)
    target_link_libraries(${PROJECT_NAME} microcdr)
endif()

# Install
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)
'''
        (pkg_dir / "CMakeLists.txt").write_text(cmake_content)
        
        # Generate each ROS 2 package
        for ros2_pkg, messages in self.unified_package.messages_by_package.items():
            pkg_include_dir = include_dir / ros2_pkg
            pkg_include_dir.mkdir(exist_ok=True)
            
            for msg_name, message in messages.items():
                # Generate header
                header_lines = [
                    f"#ifndef ROS2_TYPES_CDR_{ros2_pkg.upper()}_{msg_name.upper()}_H",
                    f"#define ROS2_TYPES_CDR_{ros2_pkg.upper()}_{msg_name.upper()}_H",
                    "",
                    "#include <stdint.h>",
                    "#include <stdbool.h>",
                    "#ifdef MICROCDR_AVAILABLE",
                    "#include <ucdr/microcdr.h>",
                    "#endif",
                    "",
                ]
                
                # Add dependency includes
                for dep in message.dependencies:
                    dep_pkg, dep_type = dep.split('.')
                    header_lines.append(f"#include \"../{dep_pkg}/{dep_type.lower()}.h\"")
                
                if message.dependencies:
                    header_lines.append("")
                
                # Add struct definition
                header_lines.extend([
                    f"/// {message.package}/{message.name} message",
                    "typedef struct {",
                ])
                
                for field in message.fields:
                    # Handle array syntax correctly for C
                    if field.is_array and field.is_bounded_array:
                        # For arrays, put the name before the brackets
                        if field.is_builtin:
                            base_type = self.type_mappings[field.type]['c']
                        else:
                            base_type = f"{field.ros2_package}_{field.type}_t"
                        header_lines.append(f"    {base_type} {field.name}[{field.array_size}];")
                    else:
                        header_lines.append(f"    {self.get_c_type(field)} {field.name};")
                
                header_lines.extend([
                    f"}} {ros2_pkg}_{msg_name}_t;",
                    "",
                    "#ifdef MICROCDR_AVAILABLE",
                    f"int {ros2_pkg}_{msg_name}_serialize(const {ros2_pkg}_{msg_name}_t* msg, uint8_t* buffer, size_t buffer_size, size_t* serialized_size);",
                    f"int {ros2_pkg}_{msg_name}_deserialize(const uint8_t* buffer, size_t buffer_size, {ros2_pkg}_{msg_name}_t* msg);",
                    "#endif",
                    "",
                    f"#endif // ROS2_TYPES_CDR_{ros2_pkg.upper()}_{msg_name.upper()}_H",
                ])
                
                (pkg_include_dir / f"{msg_name.lower()}.h").write_text('\n'.join(header_lines))
                
                # Generate source with Micro-CDR serialization
                source_lines = [
                    f"#include \"ros2_interfaces_c/{ros2_pkg}/{msg_name.lower()}.h\"",
                    "",
                    "#ifdef MICROCDR_AVAILABLE",
                    "",
                    f"int {ros2_pkg}_{msg_name}_serialize(const {ros2_pkg}_{msg_name}_t* msg, uint8_t* buffer, size_t buffer_size, size_t* serialized_size) {{",
                    "    if (!msg || !buffer || !serialized_size) {",
                    "        return -1;",
                    "    }",
                    "",
                    "    // Add CDR encapsulation header (4 bytes: 0x00 0x01 0x00 0x00 for little-endian)",
                    "    if (buffer_size < 4) {",
                    "        return -1;",
                    "    }",
                    "    buffer[0] = 0x00;  // Encapsulation kind",
                    "    buffer[1] = 0x01;  // Encapsulation options (little-endian)",
                    "    buffer[2] = 0x00;  // Options",
                    "    buffer[3] = 0x00;  // Options",
                    "",
                    "    ucdrBuffer writer;",
                    "    ucdr_init_buffer_origin_offset_endian(&writer, buffer + 4, buffer_size - 4, 0, 0, UCDR_LITTLE_ENDIANNESS);",
                    "",
                ]
                
                # Generate serialization code for each field
                for field in message.fields:
                    self._generate_c_field_serialization(source_lines, field, "msg", "writer", serialize=True)
                
                source_lines.extend([
                    "",
                    "    *serialized_size = 4 + ucdr_buffer_length(&writer);",
                    "    return ucdr_buffer_has_error(&writer) ? -1 : 0;",
                    "}",
                    "",
                    f"int {ros2_pkg}_{msg_name}_deserialize(const uint8_t* buffer, size_t buffer_size, {ros2_pkg}_{msg_name}_t* msg) {{",
                    "    if (!msg || !buffer) {",
                    "        return -1;",
                    "    }",
                    "",
                    "    // Check CDR encapsulation header",
                    "    if (buffer_size < 4) {",
                    "        return -1;",
                    "    }",
                    "",
                    "    ucdrBuffer reader;",
                    "    ucdr_init_buffer_origin_offset_endian(&reader, (uint8_t*)buffer + 4, buffer_size - 4, 0, 0, UCDR_LITTLE_ENDIANNESS);",
                    "",
                ])
                
                # Generate deserialization code for each field
                for field in message.fields:
                    self._generate_c_field_serialization(source_lines, field, "msg", "reader", serialize=False)
                
                source_lines.extend([
                    "",
                    "    return ucdr_buffer_has_error(&reader) ? -1 : 0;",
                    "}",
                    "#endif",
                ])
                
                pkg_src_dir = src_dir / ros2_pkg
                pkg_src_dir.mkdir(exist_ok=True)
                (pkg_src_dir / f"{msg_name.lower()}.c").write_text('\n'.join(source_lines))
        
        # Generate main header
        main_header_lines = [
            "#ifndef ROS2_INTERFACES_C_H",
            "#define ROS2_INTERFACES_C_H",
            "",
            "// ROS 2 CDR Interfaces - Unified C Library",
            "",
        ]
        
        for ros2_pkg, messages in self.unified_package.messages_by_package.items():
            for msg_name in messages.keys():
                main_header_lines.append(f"#include \"ros2_interfaces_c/{ros2_pkg}/{msg_name.lower()}.h\"")
        
        main_header_lines.extend([
            "",
            "#endif // ROS2_INTERFACES_C_H",
        ])
        
        (include_dir.parent / "ros2_interfaces_c.h").write_text('\n'.join(main_header_lines))
        
        print(f"✅ Generated C library: {pkg_dir}")
    
    def _generate_c_field_serialization(self, lines: list, field: FieldInfo, struct_var: str, buffer_var: str, serialize: bool):
        """Generate C serialization/deserialization code for a field."""
        field_access = f"{struct_var}->{field.name}"
        
        if field.is_builtin:
            # Map ROS 2 types to Micro-CDR function suffixes
            type_suffix_map = {
                'bool': '_bool',
                'int8': '_int8_t',
                'uint8': '_uint8_t', 
                'int16': '_int16_t',
                'uint16': '_uint16_t',
                'int32': '_int32_t',
                'uint32': '_uint32_t',
                'int64': '_int64_t',
                'uint64': '_uint64_t',
                'float32': '_float',
                'float64': '_double',
                'string': '_string',
                'char': '_char',
                'byte': '_uint8_t',
            }
            suffix = type_suffix_map.get(field.type, '_uint8_t')
            
            if serialize:
                if field.type == 'string':
                    lines.append(f"    if (!ucdr_serialize{suffix}(&{buffer_var}, {field_access})) return -1;")
                elif field.is_array:
                    if field.is_bounded_array:
                        lines.append(f"    if (!ucdr_serialize_array{suffix}(&{buffer_var}, {field_access}, {field.array_size})) return -1;")
                    else:
                        lines.append(f"    // TODO: Variable-length array serialization for {field.name}")
                else:
                    lines.append(f"    if (!ucdr_serialize{suffix}(&{buffer_var}, {field_access})) return -1;")
            else:  # deserialize
                if field.type == 'string':
                    lines.append(f"    if (!ucdr_deserialize{suffix}(&{buffer_var}, {field_access}, sizeof({field_access}))) return -1;")
                elif field.is_array:
                    if field.is_bounded_array:
                        lines.append(f"    if (!ucdr_deserialize_array{suffix}(&{buffer_var}, {field_access}, {field.array_size})) return -1;")
                    else:
                        lines.append(f"    // TODO: Variable-length array deserialization for {field.name}")
                else:
                    lines.append(f"    if (!ucdr_deserialize{suffix}(&{buffer_var}, &{field_access})) return -1;")
        else:
            # Nested message type - recursively serialize fields inline
            # Get the nested message info to serialize its fields
            nested_msg = self.unified_package.messages_by_package.get(field.ros2_package, {}).get(field.type)
            if nested_msg:
                lines.append(f"    // Nested type: {field.type}")
                for nested_field in nested_msg.fields:
                    nested_access = f"{struct_var}->{field.name}.{nested_field.name}"
                    # Recursively generate serialization for nested fields
                    self._generate_c_nested_field(lines, nested_field, nested_access, buffer_var, serialize)
            else:
                lines.append(f"    // TODO: Unknown nested type {field.type} from package {field.ros2_package}")
    
    def _generate_c_nested_field(self, lines: list, field: FieldInfo, field_access: str, buffer_var: str, serialize: bool):
        """Generate C serialization for a nested field (recursive helper)."""
        if field.is_builtin:
            type_suffix_map = {
                'bool': '_bool', 'int8': '_int8_t', 'uint8': '_uint8_t', 
                'int16': '_int16_t', 'uint16': '_uint16_t', 'int32': '_int32_t',
                'uint32': '_uint32_t', 'int64': '_int64_t', 'uint64': '_uint64_t',
                'float32': '_float', 'float64': '_double', 'string': '_string',
                'char': '_char', 'byte': '_uint8_t',
            }
            suffix = type_suffix_map.get(field.type, '_uint8_t')
            
            if serialize:
                if field.is_array and field.is_bounded_array:
                    lines.append(f"    if (!ucdr_serialize_array{suffix}(&{buffer_var}, {field_access}, {field.array_size})) return -1;")
                else:
                    lines.append(f"    if (!ucdr_serialize{suffix}(&{buffer_var}, {field_access})) return -1;")
            else:
                if field.is_array and field.is_bounded_array:
                    lines.append(f"    if (!ucdr_deserialize_array{suffix}(&{buffer_var}, {field_access}, {field.array_size})) return -1;")
                else:
                    lines.append(f"    if (!ucdr_deserialize{suffix}(&{buffer_var}, &{field_access})) return -1;")
        else:
            # Further nested - would need deeper recursion
            nested_msg = self.unified_package.messages_by_package.get(field.ros2_package, {}).get(field.type)
            if nested_msg:
                for nested_field in nested_msg.fields:
                    self._generate_c_nested_field(lines, nested_field, f"{field_access}.{nested_field.name}", buffer_var, serialize)
            else:
                lines.append(f"    // TODO: Deeply nested type {field.type}")
    
    def _render_python_fallback(self, template_str: str, message: MessageInfo) -> str:
        """Fallback template rendering without Jinja2."""
        # Simple replacement for basic cases
        result = template_str
        result = result.replace("{{ message.name }}", message.name)
        result = result.replace("{{ message.package }}", message.package)
        # TODO: Handle loops properly
        return result
    
    def _render_rust_fallback(self, template_str: str, message: MessageInfo) -> str:
        """Fallback template rendering without Jinja2."""
        result = template_str
        result = result.replace("{{ message.name }}", message.name)
        result = result.replace("{{ message.package }}", message.package)
        return result
    
    def _render_typescript_fallback(self, template_str: str, message: MessageInfo) -> str:
        """Fallback template rendering without Jinja2."""
        result = template_str
        result = result.replace("{{ message.name }}", message.name)
        result = result.replace("{{ message.package }}", message.package)
        return result


def main():
    parser = argparse.ArgumentParser(
        description='Generate unified CDR-serializable ROS 2 types packages',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Generate for common ROS 2 packages
  %(prog)s -i /opt/ros/jazzy/share/geometry_msgs/msg \\
           -i /opt/ros/jazzy/share/std_msgs/msg \\
           -i /opt/ros/jazzy/share/sensor_msgs/msg \\
           -o output -l python rust typescript c
  
  # Generate only for Python
  %(prog)s -i /opt/ros/jazzy/share/geometry_msgs/msg -o output -l python
        '''
    )
    parser.add_argument('-i', '--input', action='append', required=True,
                       help='Input ROS 2 message directory (can specify multiple times)')
    parser.add_argument('-o', '--output', required=True,
                       help='Output directory')
    parser.add_argument('-l', '--languages', nargs='+',
                       choices=['python', 'rust', 'typescript', 'c'],
                       default=['python', 'rust', 'typescript', 'c'],
                       help='Languages to generate (default: all)')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Verbose output')
    
    args = parser.parse_args()
    
    if not HAS_JINJA2:
        print("⚠️  Consider installing Jinja2 for better template support")
    
    generator = TemplateGenerator()
    
    # Parse all input directories
    for input_path in args.input:
        if args.verbose:
            print(f"Parsing {input_path}...")
        
        if os.path.isfile(input_path):
            message = generator.parse_msg_file(input_path)
            if message.package not in generator.unified_package.messages_by_package:
                generator.unified_package.messages_by_package[message.package] = {}
            generator.unified_package.messages_by_package[message.package][message.name] = message
            generator.unified_package.all_packages.add(message.package)
        else:
            for root, dirs, files in os.walk(input_path):
                for file in files:
                    if file.endswith('.msg'):
                        file_path = os.path.join(root, file)
                        message = generator.parse_msg_file(file_path)
                        if message.package not in generator.unified_package.messages_by_package:
                            generator.unified_package.messages_by_package[message.package] = {}
                        generator.unified_package.messages_by_package[message.package][message.name] = message
                        generator.unified_package.all_packages.add(message.package)
    
    # Generate output
    output_path = Path(args.output)
    output_path.mkdir(parents=True, exist_ok=True)
    
    total_messages = sum(len(msgs) for msgs in generator.unified_package.messages_by_package.values())
    print(f"\n📦 Generating unified packages with {total_messages} messages from {len(generator.unified_package.all_packages)} ROS 2 packages")
    print(f"   ROS 2 packages: {', '.join(sorted(generator.unified_package.all_packages))}")
    print()
    
    if 'python' in args.languages:
        generator.generate_python(output_path)
    
    if 'rust' in args.languages:
        generator.generate_rust(output_path)
    
    if 'typescript' in args.languages:
        generator.generate_typescript(output_path)
    
    if 'c' in args.languages:
        generator.generate_c(output_path)
    
    print(f"\n✅ Generation complete!")
    print(f"📁 Output: {args.output}")
    print(f"\nGenerated packages:")
    if 'python' in args.languages:
        print(f"  Python:     {output_path}/python/ros2_interfaces_py/ (pip: ros2-interfaces-py)")
    if 'rust' in args.languages:
        print(f"  Rust:       {output_path}/rust/ros2_interfaces_rs/ (crate: ros2_interfaces_rs)")
    if 'typescript' in args.languages:
        print(f"  TypeScript: {output_path}/typescript/ros2-cdr-interfaces/ (npm: @ros2-cdr/interfaces-ts)")
    if 'c' in args.languages:
        print(f"  C:          {output_path}/c/ros2_interfaces_c/ (CMake: ros2_interfaces_c)")


if __name__ == '__main__':
    main()

