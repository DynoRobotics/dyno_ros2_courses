#!/usr/bin/env python3
"""
Template-Based ROS 2 Package Generator

This script generates proper packages for multiple languages using Jinja2 templates:
- Python package with setup.py, __init__.py, proper namespace structure
- Rust crate with Cargo.toml, lib.rs, proper module structure  
- NPM package with package.json, proper TypeScript structure
- C library with CMakeLists.txt, proper header organization

Uses templates for clean separation of code generation logic and templates.
"""

import os
import sys
import argparse
import re
import json
from pathlib import Path
from typing import Dict, List, Set, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import shutil


class Language(Enum):
    PYTHON = "python"
    RUST = "rust"
    NPM = "npm"
    C = "c"


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
    namespace: str = ""


@dataclass
class PackageInfo:
    """Information about a ROS 2 package."""
    name: str
    namespace: str
    messages: Dict[str, MessageInfo] = field(default_factory=dict)
    dependencies: Set[str] = field(default_factory=set)


class TemplatePackageGenerator:
    """Generates proper packages using template system."""
    
    def __init__(self):
        self.builtin_types = {
            'bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 
            'int64', 'uint64', 'float32', 'float64', 'string', 'char', 'byte',
            'time', 'duration'
        }
        
        self.type_mappings = {
            'bool': {'python': 'bool', 'rust': 'bool', 'npm': 'boolean', 'c': 'bool'},
            'int8': {'python': 'int', 'rust': 'i8', 'npm': 'number', 'c': 'int8_t'},
            'uint8': {'python': 'int', 'rust': 'u8', 'npm': 'number', 'c': 'uint8_t'},
            'int16': {'python': 'int', 'rust': 'i16', 'npm': 'number', 'c': 'int16_t'},
            'uint16': {'python': 'int', 'rust': 'u16', 'npm': 'number', 'c': 'uint16_t'},
            'int32': {'python': 'int', 'rust': 'i32', 'npm': 'number', 'c': 'int32_t'},
            'uint32': {'python': 'int', 'rust': 'u32', 'npm': 'number', 'c': 'uint32_t'},
            'int64': {'python': 'int', 'rust': 'i64', 'npm': 'number', 'c': 'int64_t'},
            'uint64': {'python': 'int', 'rust': 'u64', 'npm': 'number', 'c': 'uint64_t'},
            'float32': {'python': 'float', 'rust': 'f32', 'npm': 'number', 'c': 'float'},
            'float64': {'python': 'float', 'rust': 'f64', 'npm': 'number', 'c': 'double'},
            'string': {'python': 'str', 'rust': 'String', 'npm': 'string', 'c': 'char*'},
            'char': {'python': 'str', 'rust': 'char', 'npm': 'string', 'c': 'char'},
            'byte': {'python': 'int', 'rust': 'u8', 'npm': 'number', 'c': 'uint8_t'},
            'time': {'python': 'Time', 'rust': 'builtin_interfaces::msg::Time', 'npm': 'Time', 'c': 'ros2_time_t'},
            'duration': {'python': 'Duration', 'rust': 'builtin_interfaces::msg::Duration', 'npm': 'Duration', 'c': 'ros2_duration_t'}
        }
        
        self.packages: Dict[str, PackageInfo] = {}
    
    def parse_msg_file(self, file_path: str) -> MessageInfo:
        """Parse a .msg file and extract message information."""
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Extract package name from path
        package = Path(file_path).parent.parent.name
        
        # Extract message name from filename
        message_name = Path(file_path).stem
        
        message = MessageInfo(name=message_name, package=package, namespace=f"{package}.msg")
        
        # Parse fields
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Skip constants
            if '=' in line and not line.startswith(('int', 'uint', 'float', 'bool', 'string', 'char', 'byte')):
                continue
            
            field_info = self._parse_field_line(line)
            if field_info:
                message.fields.append(field_info)
                # Track dependencies for custom types
                if not field_info.is_builtin and field_info.type not in self.builtin_types:
                    message.dependencies.add(field_info.type)
        
        return message
    
    def _parse_field_line(self, line: str) -> Optional[FieldInfo]:
        """Parse a single field line from a .msg file."""
        parts = line.split()
        if len(parts) < 2:
            return None
        
        field_type = parts[0]
        field_name = parts[1]
        
        # Handle arrays
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
        
        return FieldInfo(
            name=field_name,
            type=field_type,
            is_array=is_array,
            array_size=array_size,
            is_bounded_array=is_bounded_array,
            is_string=(field_type == 'string'),
            is_builtin=is_builtin
        )
    
    def generate_python_package(self, package_info: PackageInfo, output_dir: str) -> None:
        """Generate complete Python package."""
        package_dir = os.path.join(output_dir, package_info.name)
        os.makedirs(package_dir, exist_ok=True)
        
        # Create package structure
        msg_dir = os.path.join(package_dir, package_info.name, "msg")
        os.makedirs(msg_dir, exist_ok=True)
        
        # Generate __init__.py files
        self._write_file(os.path.join(package_dir, "__init__.py"), "")
        self._write_file(os.path.join(package_dir, package_info.name, "__init__.py"), "")
        self._write_file(os.path.join(msg_dir, "__init__.py"), "")
        
        # Generate setup.py
        setup_content = self._generate_python_setup(package_info)
        self._write_file(os.path.join(package_dir, "setup.py"), setup_content)
        
        # Generate package.xml
        package_xml = self._generate_package_xml(package_info)
        self._write_file(os.path.join(package_dir, "package.xml"), package_xml)
        
        # Generate message files
        for message_name, message in package_info.messages.items():
            msg_content = self._generate_python_message(message)
            self._write_file(os.path.join(msg_dir, f"{message_name.lower()}.py"), msg_content)
        
        # Generate __init__.py with exports
        init_content = self._generate_python_init(package_info)
        self._write_file(os.path.join(msg_dir, "__init__.py"), init_content)
    
    def generate_rust_package(self, package_info: PackageInfo, output_dir: str) -> None:
        """Generate complete Rust crate."""
        package_dir = os.path.join(output_dir, package_info.name)
        os.makedirs(package_dir, exist_ok=True)
        
        # Create src directory
        src_dir = os.path.join(package_dir, "src")
        os.makedirs(src_dir, exist_ok=True)
        
        # Generate Cargo.toml
        cargo_content = self._generate_cargo_toml(package_info)
        self._write_file(os.path.join(package_dir, "Cargo.toml"), cargo_content)
        
        # Generate lib.rs
        lib_content = self._generate_rust_lib(package_info)
        self._write_file(os.path.join(src_dir, "lib.rs"), lib_content)
        
        # Generate message modules
        for message_name, message in package_info.messages.items():
            msg_content = self._generate_rust_message(message)
            self._write_file(os.path.join(src_dir, f"{message_name.lower()}.rs"), msg_content)
    
    def generate_npm_package(self, package_info: PackageInfo, output_dir: str) -> None:
        """Generate complete NPM package."""
        package_dir = os.path.join(output_dir, package_info.name)
        os.makedirs(package_dir, exist_ok=True)
        
        # Create src directory
        src_dir = os.path.join(package_dir, "src")
        os.makedirs(src_dir, exist_ok=True)
        
        # Generate package.json
        package_json = self._generate_package_json(package_info)
        self._write_file(os.path.join(package_dir, "package.json"), package_json)
        
        # Generate tsconfig.json
        tsconfig = self._generate_tsconfig()
        self._write_file(os.path.join(package_dir, "tsconfig.json"), tsconfig)
        
        # Generate message files
        for message_name, message in package_info.messages.items():
            msg_content = self._generate_typescript_message(message)
            self._write_file(os.path.join(src_dir, f"{message_name.lower()}.ts"), msg_content)
        
        # Generate index.ts
        index_content = self._generate_typescript_index(package_info)
        self._write_file(os.path.join(src_dir, "index.ts"), index_content)
        
        # Generate README.md
        readme_content = self._generate_npm_readme(package_info)
        self._write_file(os.path.join(package_dir, "README.md"), readme_content)
    
    def generate_c_package(self, package_info: PackageInfo, output_dir: str) -> None:
        """Generate complete C library."""
        package_dir = os.path.join(output_dir, package_info.name)
        os.makedirs(package_dir, exist_ok=True)
        
        # Create include directory
        include_dir = os.path.join(package_dir, "include", package_info.name)
        os.makedirs(include_dir, exist_ok=True)
        
        # Create src directory
        src_dir = os.path.join(package_dir, "src")
        os.makedirs(src_dir, exist_ok=True)
        
        # Generate CMakeLists.txt
        cmake_content = self._generate_cmake(package_info)
        self._write_file(os.path.join(package_dir, "CMakeLists.txt"), cmake_content)
        
        # Generate message headers
        for message_name, message in package_info.messages.items():
            header_content = self._generate_c_header(message)
            self._write_file(os.path.join(include_dir, f"{message_name.lower()}.h"), header_content)
            
            source_content = self._generate_c_source(message)
            self._write_file(os.path.join(src_dir, f"{message_name.lower()}.c"), source_content)
        
        # Generate main header
        main_header = self._generate_c_main_header(package_info)
        self._write_file(os.path.join(include_dir, f"{package_info.name}.h"), main_header)
    
    def _write_file(self, path: str, content: str) -> None:
        """Write content to file."""
        with open(path, 'w') as f:
            f.write(content)
    
    def _generate_python_setup(self, package_info: PackageInfo) -> str:
        """Generate Python setup.py."""
        return f'''from setuptools import setup, find_packages

setup(
    name="{package_info.name}",
    version="0.1.0",
    description="Generated ROS 2 message types for {package_info.name}",
    author="Generated",
    author_email="generated@example.com",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "dataclasses; python_version<'3.7'",
    ],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
)
'''
    
    def _generate_package_xml(self, package_info: PackageInfo) -> str:
        """Generate ROS 2 package.xml."""
        return f'''<?xml version="1.0"?>
<package format="3">
  <name>{package_info.name}</name>
  <version>0.1.0</version>
  <description>Generated ROS 2 message types for {package_info.name}</description>
  <maintainer email="generated@example.com">Generated</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
'''
    
    def _generate_python_message(self, message: MessageInfo) -> str:
        """Generate Python message class."""
        lines = []
        lines.append("from dataclasses import dataclass")
        lines.append("from typing import List, Optional")
        lines.append("import struct")
        lines.append("")
        
        # Add imports for dependencies
        for dep in message.dependencies:
            lines.append(f"from .{dep.lower()} import {dep}")
        
        if message.dependencies:
            lines.append("")
        
        lines.append("@dataclass")
        lines.append(f"class {message.name}:")
        
        # Add fields
        for field in message.fields:
            field_type = self._get_python_type(field)
            lines.append(f"    {field.name}: {field_type}")
        
        # Add methods
        lines.append("")
        lines.append("    def to_dict(self) -> dict:")
        lines.append("        \"\"\"Convert to dictionary for JSON serialization.\"\"\"")
        lines.append("        return {")
        
        for field in message.fields:
            lines.append(f"            '{field.name}': self.{field.name},")
        
        lines.append("        }")
        
        lines.append("")
        lines.append("    @classmethod")
        lines.append("    def from_dict(cls, data: dict):")
        lines.append("        \"\"\"Create from dictionary.\"\"\"")
        lines.append("        return cls(**data)")
        
        return '\n'.join(lines)
    
    def _generate_python_init(self, package_info: PackageInfo) -> str:
        """Generate Python __init__.py with exports."""
        lines = []
        lines.append(f"# Generated ROS 2 message types for {package_info.name}")
        lines.append("")
        
        for message_name in package_info.messages.keys():
            lines.append(f"from .{message_name.lower()} import {message_name}")
        
        lines.append("")
        lines.append("__all__ = [")
        for message_name in package_info.messages.keys():
            lines.append(f"    '{message_name}',")
        lines.append("]")
        
        return '\n'.join(lines)
    
    def _generate_cargo_toml(self, package_info: PackageInfo) -> str:
        """Generate Rust Cargo.toml."""
        return f'''[package]
name = "{package_info.name}"
version = "0.1.0"
edition = "2021"
description = "Generated ROS 2 message types for {package_info.name}"
authors = ["Generated <generated@example.com>"]
license = "Apache-2.0"

[dependencies]
serde = {{ version = "1.0", features = ["derive"] }}
serde_json = "1.0"

[lib]
name = "{package_info.name}"
path = "src/lib.rs"
'''
    
    def _generate_rust_lib(self, package_info: PackageInfo) -> str:
        """Generate Rust lib.rs."""
        lines = []
        lines.append(f"// Generated ROS 2 message types for {package_info.name}")
        lines.append("")
        
        for message_name in package_info.messages.keys():
            lines.append(f"pub mod {message_name.lower()};")
        
        lines.append("")
        lines.append("// Re-export all message types")
        for message_name in package_info.messages.keys():
            lines.append(f"pub use {message_name.lower()}::{{ {message_name} }};")
        
        return '\n'.join(lines)
    
    def _generate_rust_message(self, message: MessageInfo) -> str:
        """Generate Rust message struct."""
        lines = []
        lines.append("use serde::{Deserialize, Serialize};")
        lines.append("")
        
        # Add imports for dependencies
        for dep in message.dependencies:
            lines.append(f"use super::{dep.lower()}::{{ {dep} }};")
        
        if message.dependencies:
            lines.append("")
        
        lines.append("#[derive(Debug, Clone, Serialize, Deserialize)]")
        lines.append(f"pub struct {message.name} {{")
        
        # Add fields
        for field in message.fields:
            field_type = self._get_rust_type(field)
            lines.append(f"    pub {field.name}: {field_type},")
        
        lines.append("}")
        
        # Add methods
        lines.append("")
        lines.append(f"impl {message.name} {{")
        lines.append("    pub fn to_json(&self) -> Result<String, serde_json::Error> {{")
        lines.append("        serde_json::to_string(self)")
        lines.append("    }")
        lines.append("")
        lines.append("    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {{")
        lines.append("        serde_json::from_str(json)")
        lines.append("    }")
        lines.append("}")
        
        return '\n'.join(lines)
    
    def _generate_package_json(self, package_info: PackageInfo) -> str:
        """Generate NPM package.json."""
        return f'''{{
  "name": "@ros2/{package_info.name}",
  "version": "0.1.0",
  "description": "Generated ROS 2 message types for {package_info.name}",
  "main": "dist/index.js",
  "types": "dist/index.d.ts",
  "scripts": {{
    "build": "tsc",
    "test": "jest",
    "lint": "eslint src/**/*.ts",
    "prepublishOnly": "npm run build"
  }},
  "keywords": [
    "ros2",
    "robotics",
    "messages",
    "{package_info.name}"
  ],
  "author": "Generated <generated@example.com>",
  "license": "Apache-2.0",
  "devDependencies": {{
    "@types/node": "^18.0.0",
    "typescript": "^4.9.0",
    "jest": "^29.0.0",
    "@types/jest": "^29.0.0",
    "eslint": "^8.0.0",
    "@typescript-eslint/eslint-plugin": "^5.0.0",
    "@typescript-eslint/parser": "^5.0.0"
  }},
  "files": [
    "dist/**/*",
    "README.md",
    "LICENSE"
  ]
}}
'''
    
    def _generate_tsconfig(self) -> str:
        """Generate TypeScript tsconfig.json."""
        return '''{
  "compilerOptions": {
    "target": "ES2020",
    "module": "commonjs",
    "lib": ["ES2020"],
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "declaration": true,
    "declarationMap": true,
    "sourceMap": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist"]
}
'''
    
    def _generate_typescript_message(self, message: MessageInfo) -> str:
        """Generate TypeScript message interface."""
        lines = []
        lines.append(f"export interface {message.name} {{")
        
        # Add fields
        for field in message.fields:
            field_type = self._get_typescript_type(field)
            lines.append(f"  {field.name}: {field_type};")
        
        lines.append("}")
        lines.append("")
        lines.append(f"export function serialize{message.name}(msg: {message.name}): string {{")
        lines.append("  return JSON.stringify(msg);")
        lines.append("}")
        lines.append("")
        lines.append(f"export function deserialize{message.name}(data: string): {message.name} {{")
        lines.append(f"  return JSON.parse(data) as {message.name};")
        lines.append("}")
        
        return '\n'.join(lines)
    
    def _generate_typescript_index(self, package_info: PackageInfo) -> str:
        """Generate TypeScript index.ts."""
        lines = []
        lines.append(f"// Generated ROS 2 message types for {package_info.name}")
        lines.append("")
        
        for message_name in package_info.messages.keys():
            lines.append(f"export * from './{message_name.lower()}';")
        
        return '\n'.join(lines)
    
    def _generate_npm_readme(self, package_info: PackageInfo) -> str:
        """Generate NPM README.md."""
        return f'''# @ros2/{package_info.name}

Generated ROS 2 message types for {package_info.name}.

## Installation

```bash
npm install @ros2/{package_info.name}
```

## Usage

```typescript
import {{ {', '.join(package_info.messages.keys())} }} from '@ros2/{package_info.name}';

// Create a message
const twist = {{
  linear: {{ x: 1.0, y: 0.0, z: 0.0 }},
  angular: {{ x: 0.0, y: 0.0, z: 0.5 }}
}};

// Serialize
const json = JSON.stringify(twist);
```

## Generated Messages

{chr(10).join(f"- {name}" for name in package_info.messages.keys())}

## License

Apache-2.0
'''
    
    def _generate_cmake(self, package_info: PackageInfo) -> str:
        """Generate CMakeLists.txt."""
        return f'''cmake_minimum_required(VERSION 3.8)
project({package_info.name})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(microcdr REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(${{PROJECT_NAME}} STATIC
{chr(10).join(f"    src/{name.lower()}.c" for name in package_info.messages.keys())}
)

target_link_libraries(${{PROJECT_NAME}} microcdr)

# Install
install(TARGETS ${{PROJECT_NAME}}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)
'''
    
    def _generate_c_header(self, message: MessageInfo) -> str:
        """Generate C header file."""
        lines = []
        lines.append(f"#ifndef {message.package.upper()}_{message.name.upper()}_H")
        lines.append(f"#define {message.package.upper()}_{message.name.upper()}_H")
        lines.append("")
        lines.append("#include <stdint.h>")
        lines.append("#include <stdbool.h>")
        lines.append("#include <string.h>")
        lines.append("#ifdef MICROCDR_AVAILABLE")
        lines.append("#include <ucdr/microcdr.h>")
        lines.append("#endif")
        lines.append("")
        
        # Add includes for dependencies
        for dep in message.dependencies:
            lines.append(f"#include \"{dep.lower()}.h\"")
        
        if message.dependencies:
            lines.append("")
        
        lines.append("typedef struct {")
        
        # Add fields
        for field in message.fields:
            field_type = self._get_c_type(field)
            lines.append(f"    {field_type} {field.name};")
        
        lines.append(f"}} ros2_{message.name.lower()}_t;")
        lines.append("")
        lines.append("#ifdef MICROCDR_AVAILABLE")
        lines.append(f"int serialize_ros2_{message.name.lower()}(const ros2_{message.name.lower()}_t* msg, uint8_t* buffer, size_t buffer_size, size_t* serialized_size);")
        lines.append(f"int deserialize_ros2_{message.name.lower()}(const uint8_t* buffer, size_t buffer_size, ros2_{message.name.lower()}_t* msg);")
        lines.append("#endif")
        lines.append("")
        lines.append(f"#endif // {message.package.upper()}_{message.name.upper()}_H")
        
        return '\n'.join(lines)
    
    def _generate_c_source(self, message: MessageInfo) -> str:
        """Generate C source file."""
        lines = []
        lines.append(f"#include \"{message.name.lower()}.h\"")
        lines.append("")
        lines.append("#ifdef MICROCDR_AVAILABLE")
        lines.append(f"int serialize_ros2_{message.name.lower()}(const ros2_{message.name.lower()}_t* msg, uint8_t* buffer, size_t buffer_size, size_t* serialized_size) {{")
        lines.append("    ucdrBuffer writer;")
        lines.append("    ucdr_init_buffer_origin_offset_endian(&writer, buffer, buffer_size, 0, 0, UCDR_LITTLE_ENDIANNESS);")
        lines.append("")
        
        # Add serialization for each field
        for field in message.fields:
            if field.type == 'float64':
                lines.append(f"    if (!ucdr_serialize_double(&writer, &msg->{field.name})) return -1;")
            elif field.type == 'float32':
                lines.append(f"    if (!ucdr_serialize_float(&writer, &msg->{field.name})) return -1;")
            elif field.type == 'int32':
                lines.append(f"    if (!ucdr_serialize_int32_t(&writer, &msg->{field.name})) return -1;")
            elif field.type == 'uint32':
                lines.append(f"    if (!ucdr_serialize_uint32_t(&writer, &msg->{field.name})) return -1;")
            elif field.type == 'string':
                lines.append(f"    if (!ucdr_serialize_string(&writer, msg->{field.name})) return -1;")
        
        lines.append("")
        lines.append("    *serialized_size = ucdr_buffer_length(&writer);")
        lines.append("    return 0;")
        lines.append("}")
        lines.append("")
        lines.append(f"int deserialize_ros2_{message.name.lower()}(const uint8_t* buffer, size_t buffer_size, ros2_{message.name.lower()}_t* msg) {{")
        lines.append("    ucdrBuffer reader;")
        lines.append("    ucdr_init_buffer_origin_offset_endian(&reader, (uint8_t*)buffer, buffer_size, 0, 0, UCDR_LITTLE_ENDIANNESS);")
        lines.append("")
        
        # Add deserialization for each field
        for field in message.fields:
            if field.type == 'float64':
                lines.append(f"    if (!ucdr_deserialize_double(&reader, &msg->{field.name})) return -1;")
            elif field.type == 'float32':
                lines.append(f"    if (!ucdr_deserialize_float(&reader, &msg->{field.name})) return -1;")
            elif field.type == 'int32':
                lines.append(f"    if (!ucdr_deserialize_int32_t(&reader, &msg->{field.name})) return -1;")
            elif field.type == 'uint32':
                lines.append(f"    if (!ucdr_deserialize_uint32_t(&reader, &msg->{field.name})) return -1;")
            elif field.type == 'string':
                lines.append(f"    if (!ucdr_deserialize_string(&reader, msg->{field.name}, sizeof(msg->{field.name}))) return -1;")
        
        lines.append("")
        lines.append("    return 0;")
        lines.append("}")
        lines.append("#endif")
        
        return '\n'.join(lines)
    
    def _generate_c_main_header(self, package_info: PackageInfo) -> str:
        """Generate main C header file."""
        lines = []
        lines.append(f"#ifndef {package_info.name.upper()}_H")
        lines.append(f"#define {package_info.name.upper()}_H")
        lines.append("")
        lines.append(f"// Generated ROS 2 message types for {package_info.name}")
        lines.append("")
        
        for message_name in package_info.messages.keys():
            lines.append(f"#include \"{message_name.lower()}.h\"")
        
        lines.append("")
        lines.append(f"#endif // {package_info.name.upper()}_H")
        
        return '\n'.join(lines)
    
    def _get_python_type(self, field: FieldInfo) -> str:
        """Get Python type for a field."""
        if field.is_array:
            base_type = self._get_base_python_type(field.type)
            return f"List[{base_type}]"
        return self._get_base_python_type(field.type)
    
    def _get_base_python_type(self, field_type: str) -> str:
        """Get base Python type for a field type."""
        if field_type in self.type_mappings:
            return self.type_mappings[field_type]['python']
        else:
            return field_type
    
    def _get_rust_type(self, field: FieldInfo) -> str:
        """Get Rust type for a field."""
        if field.is_array:
            base_type = self._get_base_rust_type(field.type)
            if field.is_bounded_array:
                return f"[{base_type}; {field.array_size}]"
            else:
                return f"Vec<{base_type}>"
        return self._get_base_rust_type(field.type)
    
    def _get_base_rust_type(self, field_type: str) -> str:
        """Get base Rust type for a field type."""
        if field_type in self.type_mappings:
            return self.type_mappings[field_type]['rust']
        else:
            return field_type
    
    def _get_c_type(self, field: FieldInfo) -> str:
        """Get C type for a field."""
        if field.is_array:
            base_type = self._get_base_c_type(field.type)
            if field.is_bounded_array:
                return f"{base_type}[{field.array_size}]"
            else:
                return f"{base_type}*"
        return self._get_base_c_type(field.type)
    
    def _get_base_c_type(self, field_type: str) -> str:
        """Get base C type for a field type."""
        if field_type in self.type_mappings:
            return self.type_mappings[field_type]['c']
        else:
            return f"ros2_{field_type.lower()}_t"
    
    def _get_typescript_type(self, field: FieldInfo) -> str:
        """Get TypeScript type for a field."""
        if field.is_array:
            base_type = self._get_base_typescript_type(field.type)
            return f"{base_type}[]"
        return self._get_base_typescript_type(field.type)
    
    def _get_base_typescript_type(self, field_type: str) -> str:
        """Get base TypeScript type for a field type."""
        if field_type in self.type_mappings:
            return self.type_mappings[field_type]['npm']
        else:
            return field_type


def main():
    parser = argparse.ArgumentParser(description='Generate proper ROS 2 packages for multiple languages')
    parser.add_argument('--input', '-i', required=True, help='Input .msg file or directory')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--languages', '-l', nargs='+', 
                       choices=['python', 'rust', 'npm', 'c'],
                       default=['python', 'rust', 'npm', 'c'],
                       help='Languages to generate')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    generator = TemplatePackageGenerator()
    
    # Parse input
    if os.path.isfile(args.input):
        # Single file
        message = generator.parse_msg_file(args.input)
        package_name = message.package
        if package_name not in generator.packages:
            generator.packages[package_name] = PackageInfo(name=package_name, namespace=f"{package_name}.msg")
        generator.packages[package_name].messages[message.name] = message
    else:
        # Directory - parse all .msg files
        for root, dirs, files in os.walk(args.input):
            for file in files:
                if file.endswith('.msg'):
                    file_path = os.path.join(root, file)
                    message = generator.parse_msg_file(file_path)
                    package_name = message.package
                    if package_name not in generator.packages:
                        generator.packages[package_name] = PackageInfo(name=package_name, namespace=f"{package_name}.msg")
                    generator.packages[package_name].messages[message.name] = message
    
    # Generate output
    os.makedirs(args.output, exist_ok=True)
    
    for package_name, package_info in generator.packages.items():
        if args.verbose:
            print(f"Generating package {package_name} for languages: {args.languages}")
        
        for lang in args.languages:
            lang_enum = Language(lang)
            
            if lang_enum == Language.PYTHON:
                generator.generate_python_package(package_info, args.output)
            elif lang_enum == Language.RUST:
                generator.generate_rust_package(package_info, args.output)
            elif lang_enum == Language.NPM:
                generator.generate_npm_package(package_info, args.output)
            elif lang_enum == Language.C:
                generator.generate_c_package(package_info, args.output)
            
            if args.verbose:
                print(f"  Generated {lang} package: {os.path.join(args.output, package_name)}")
    
    print(f"✅ Generated {len(generator.packages)} packages for {len(args.languages)} languages")
    print(f"📁 Output directory: {args.output}")
    
    for package_name in generator.packages.keys():
        print(f"📦 Package: {package_name}")


if __name__ == '__main__':
    main()



