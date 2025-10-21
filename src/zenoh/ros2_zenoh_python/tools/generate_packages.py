#!/usr/bin/env python3
"""
Template-Based ROS 2 Package Generator

Generates proper packages for multiple languages using Jinja2 templates.
Output structure:
  output/
    python/
      geometry_msgs/
      std_msgs/
      sensor_msgs/
    rust/
      geometry_msgs/
      std_msgs/
      sensor_msgs/
    typescript/
      geometry_msgs/
      std_msgs/
      sensor_msgs/
"""

import os
import sys
import argparse
import re
from pathlib import Path
from typing import Dict, List, Set, Optional
from dataclasses import dataclass, field
from enum import Enum

try:
    from jinja2 import Environment, FileSystemLoader, Template
    HAS_JINJA2 = True
except ImportError:
    HAS_JINJA2 = False
    print("Warning: Jinja2 not found. Install with: pip install jinja2")


class Language(Enum):
    PYTHON = "python"
    RUST = "rust"
    TYPESCRIPT = "typescript"
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


@dataclass
class MessageInfo:
    """Information about a ROS 2 message."""
    name: str
    package: str
    fields: List[FieldInfo] = field(default_factory=list)
    dependencies: Set[str] = field(default_factory=set)


@dataclass
class PackageInfo:
    """Information about a ROS 2 package."""
    name: str
    messages: Dict[str, MessageInfo] = field(default_factory=dict)
    dependencies: Set[str] = field(default_factory=set)


# Inline templates (will be extracted to files later)
PYTHON_SETUP_TEMPLATE = '''from setuptools import setup, find_packages

setup(
    name="{{ package.name }}-cdr",
    version="0.1.0",
    description="CDR-serializable {{ package.name }} message types (no ROS 2 dependency)",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
    ],
)
'''

PYTHON_MESSAGE_TEMPLATE = '''from dataclasses import dataclass
from typing import List, Optional

{% for dep in message.dependencies %}
from .{{ dep.lower() }} import {{ dep }}
{% endfor %}

@dataclass
class {{ message.name }}:
{% for field in message.fields %}
    {{ field.name }}: {{ get_python_type(field) }}
{% endfor %}

    def to_dict(self) -> dict:
        return {
{% for field in message.fields %}
            '{{ field.name }}': self.{{ field.name }},
{% endfor %}
        }
    
    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)
'''

PYTHON_INIT_TEMPLATE = '''# {{ package.name }} - ROS 2 message types
{% for name in message_names %}
from .{{ name.lower() }} import {{ name }}
{% endfor %}

__all__ = [
{% for name in message_names %}
    '{{ name }}',
{% endfor %}
]
'''

RUST_CARGO_TEMPLATE = '''[package]
name = "{{ package.name }}_cdr"
version = "0.1.0"
edition = "2021"
description = "CDR-serializable {{ package.name }} message types (no ROS 2 dependency)"
license = "Apache-2.0"

[dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
'''

RUST_MESSAGE_TEMPLATE = '''use serde::{Deserialize, Serialize};

{% for dep in message.dependencies %}
use crate::{{ dep.lower() }}::{{ dep }};
{% endfor %}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct {{ message.name }} {
{% for field in message.fields %}
    pub {{ field.name }}: {{ get_rust_type(field) }},
{% endfor %}
}

impl {{ message.name }} {
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }
    
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}
'''

RUST_LIB_TEMPLATE = '''// {{ package.name }} - ROS 2 message types
{% for name in message_names %}
pub mod {{ name.lower() }};
{% endfor %}

// Re-exports
{% for name in message_names %}
pub use {{ name.lower() }}::{{ name }};
{% endfor %}
'''

TS_PACKAGE_JSON_TEMPLATE = '''{
  "name": "@ros2-cdr/{{ package.name }}",
  "version": "0.1.0",
  "description": "CDR-serializable {{ package.name }} message types (no ROS 2 dependency)",
  "main": "dist/index.js",
  "types": "dist/index.d.ts",
  "scripts": {
    "build": "tsc",
    "prepublishOnly": "npm run build"
  },
  "keywords": ["ros2", "cdr", "{{ package.name }}", "serialization"],
  "license": "Apache-2.0",
  "devDependencies": {
    "typescript": "^5.0.0"
  }
}
'''

TS_TSCONFIG_TEMPLATE = '''{
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

TS_MESSAGE_TEMPLATE = '''export interface {{ message.name }} {
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
'''

TS_INDEX_TEMPLATE = '''// {{ package.name }} - ROS 2 message types
{% for name in message_names %}
export * from './{{ name.lower() }}';
{% endfor %}
'''


class PackageGenerator:
    """Generates proper packages using templates."""
    
    def __init__(self):
        self.builtin_types = {
            'bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 
            'int64', 'uint64', 'float32', 'float64', 'string', 'char', 'byte',
            'time', 'duration'
        }
        
        self.type_mappings = {
            'bool': {'python': 'bool', 'rust': 'bool', 'typescript': 'boolean', 'c': 'bool'},
            'int8': {'python': 'int', 'rust': 'i8', 'typescript': 'number', 'c': 'int8_t'},
            'uint8': {'python': 'int', 'rust': 'u8', 'typescript': 'number', 'c': 'uint8_t'},
            'int16': {'python': 'int', 'rust': 'i16', 'typescript': 'number', 'c': 'int16_t'},
            'uint16': {'python': 'int', 'rust': 'u16', 'typescript': 'number', 'c': 'uint16_t'},
            'int32': {'python': 'int', 'rust': 'i32', 'typescript': 'number', 'c': 'int32_t'},
            'uint32': {'python': 'int', 'rust': 'u32', 'typescript': 'number', 'c': 'uint32_t'},
            'int64': {'python': 'int', 'rust': 'i64', 'typescript': 'number', 'c': 'int64_t'},
            'uint64': {'python': 'int', 'rust': 'u64', 'typescript': 'number', 'c': 'uint64_t'},
            'float32': {'python': 'float', 'rust': 'f32', 'typescript': 'number', 'c': 'float'},
            'float64': {'python': 'float', 'rust': 'f64', 'typescript': 'number', 'c': 'double'},
            'string': {'python': 'str', 'rust': 'String', 'typescript': 'string', 'c': 'char*'},
        }
        
        self.packages: Dict[str, PackageInfo] = {}
        
        # Setup Jinja2 environment
        if HAS_JINJA2:
            self.env = Environment()
            self.env.filters['lower'] = lambda x: x.lower()
        
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
            
            if '=' in line:
                continue
            
            field_info = self._parse_field_line(line)
            if field_info:
                message.fields.append(field_info)
                if not field_info.is_builtin and field_info.type not in self.builtin_types:
                    message.dependencies.add(field_info.type)
        
        return message
    
    def _parse_field_line(self, line: str) -> Optional[FieldInfo]:
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
        
        return FieldInfo(
            name=field_name,
            type=field_type,
            is_array=is_array,
            array_size=array_size,
            is_bounded_array=is_bounded_array,
            is_string=(field_type == 'string'),
            is_builtin=is_builtin
        )
    
    def get_python_type(self, field: FieldInfo) -> str:
        """Get Python type for a field."""
        base_type = self.type_mappings.get(field.type, {}).get('python', field.type)
        if field.is_array:
            return f"List[{base_type}]"
        return base_type
    
    def get_rust_type(self, field: FieldInfo) -> str:
        """Get Rust type for a field."""
        base_type = self.type_mappings.get(field.type, {}).get('rust', field.type)
        if field.is_array:
            if field.is_bounded_array:
                return f"[{base_type}; {field.array_size}]"
            return f"Vec<{base_type}>"
        return base_type
    
    def get_typescript_type(self, field: FieldInfo) -> str:
        """Get TypeScript type for a field."""
        base_type = self.type_mappings.get(field.type, {}).get('typescript', field.type)
        if field.is_array:
            return f"{base_type}[]"
        return base_type
    
    def generate_python_package(self, package_info: PackageInfo, output_dir: str) -> None:
        """Generate Python package."""
        pkg_dir = Path(output_dir) / package_info.name
        pkg_dir.mkdir(parents=True, exist_ok=True)
        
        # Create __init__.py
        (pkg_dir / "__init__.py").write_text("")
        
        # Generate setup.py
        if HAS_JINJA2:
            template = self.env.from_string(PYTHON_SETUP_TEMPLATE)
            content = template.render(package=package_info)
        else:
            content = PYTHON_SETUP_TEMPLATE.replace("{{ package.name }}", package_info.name)
        (pkg_dir / "setup.py").write_text(content)
        
        # Generate messages
        for message_name, message in package_info.messages.items():
            if HAS_JINJA2:
                template = self.env.from_string(PYTHON_MESSAGE_TEMPLATE)
                content = template.render(
                    message=message,
                    get_python_type=self.get_python_type
                )
            else:
                # Fallback without Jinja2
                content = self._generate_python_message_fallback(message)
            (pkg_dir / f"{message_name.lower()}.py").write_text(content)
        
        # Generate __init__.py with exports
        if HAS_JINJA2:
            template = self.env.from_string(PYTHON_INIT_TEMPLATE)
            content = template.render(
                package=package_info,
                message_names=list(package_info.messages.keys())
            )
        else:
            content = self._generate_python_init_fallback(package_info)
        (pkg_dir / "__init__.py").write_text(content)
    
    def generate_rust_package(self, package_info: PackageInfo, output_dir: str) -> None:
        """Generate Rust package."""
        pkg_dir = Path(output_dir) / package_info.name
        src_dir = pkg_dir / "src"
        src_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate Cargo.toml
        if HAS_JINJA2:
            template = self.env.from_string(RUST_CARGO_TEMPLATE)
            content = template.render(package=package_info)
        else:
            content = RUST_CARGO_TEMPLATE.replace("{{ package.name }}", package_info.name)
        (pkg_dir / "Cargo.toml").write_text(content)
        
        # Generate messages
        for message_name, message in package_info.messages.items():
            if HAS_JINJA2:
                template = self.env.from_string(RUST_MESSAGE_TEMPLATE)
                content = template.render(
                    message=message,
                    get_rust_type=self.get_rust_type
                )
            else:
                content = self._generate_rust_message_fallback(message)
            (src_dir / f"{message_name.lower()}.rs").write_text(content)
        
        # Generate lib.rs
        if HAS_JINJA2:
            template = self.env.from_string(RUST_LIB_TEMPLATE)
            content = template.render(
                package=package_info,
                message_names=list(package_info.messages.keys())
            )
        else:
            content = self._generate_rust_lib_fallback(package_info)
        (src_dir / "lib.rs").write_text(content)
    
    def generate_typescript_package(self, package_info: PackageInfo, output_dir: str) -> None:
        """Generate TypeScript package."""
        pkg_dir = Path(output_dir) / package_info.name
        src_dir = pkg_dir / "src"
        src_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate package.json
        if HAS_JINJA2:
            template = self.env.from_string(TS_PACKAGE_JSON_TEMPLATE)
            content = template.render(package=package_info)
        else:
            content = TS_PACKAGE_JSON_TEMPLATE.replace("{{ package.name }}", package_info.name)
        (pkg_dir / "package.json").write_text(content)
        
        # Generate tsconfig.json
        (pkg_dir / "tsconfig.json").write_text(TS_TSCONFIG_TEMPLATE)
        
        # Generate messages
        for message_name, message in package_info.messages.items():
            if HAS_JINJA2:
                template = self.env.from_string(TS_MESSAGE_TEMPLATE)
                content = template.render(
                    message=message,
                    get_typescript_type=self.get_typescript_type
                )
            else:
                content = self._generate_typescript_message_fallback(message)
            (src_dir / f"{message_name.lower()}.ts").write_text(content)
        
        # Generate index.ts
        if HAS_JINJA2:
            template = self.env.from_string(TS_INDEX_TEMPLATE)
            content = template.render(
                package=package_info,
                message_names=list(package_info.messages.keys())
            )
        else:
            content = self._generate_typescript_index_fallback(package_info)
        (src_dir / "index.ts").write_text(content)
    
    def _generate_python_message_fallback(self, message: MessageInfo) -> str:
        """Fallback Python message generation without Jinja2."""
        lines = ["from dataclasses import dataclass", "from typing import List, Optional", ""]
        
        for dep in message.dependencies:
            lines.append(f"from .{dep.lower()} import {dep}")
        if message.dependencies:
            lines.append("")
        
        lines.append("@dataclass")
        lines.append(f"class {message.name}:")
        for field in message.fields:
            lines.append(f"    {field.name}: {self.get_python_type(field)}")
        
        lines.extend(["", "    def to_dict(self) -> dict:", "        return {"])
        for field in message.fields:
            lines.append(f"            '{field.name}': self.{field.name},")
        lines.extend(["        }", "", "    @classmethod", "    def from_dict(cls, data: dict):", "        return cls(**data)"])
        
        return '\n'.join(lines)
    
    def _generate_python_init_fallback(self, package_info: PackageInfo) -> str:
        """Fallback Python __init__ generation."""
        lines = [f"# {package_info.name} - ROS 2 message types", ""]
        for name in package_info.messages.keys():
            lines.append(f"from .{name.lower()} import {name}")
        lines.append("")
        lines.append("__all__ = [")
        for name in package_info.messages.keys():
            lines.append(f"    '{name}',")
        lines.append("]")
        return '\n'.join(lines)
    
    def _generate_rust_message_fallback(self, message: MessageInfo) -> str:
        """Fallback Rust message generation."""
        lines = ["use serde::{Deserialize, Serialize};", ""]
        
        for dep in message.dependencies:
            lines.append(f"use crate::{dep.lower()}::{dep};")
        if message.dependencies:
            lines.append("")
        
        lines.append("#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]")
        lines.append(f"pub struct {message.name} {{")
        for field in message.fields:
            lines.append(f"    pub {field.name}: {self.get_rust_type(field)},")
        lines.append("}")
        
        lines.extend(["", f"impl {message.name} {{", "    pub fn to_json(&self) -> Result<String, serde_json::Error> {",
                     "        serde_json::to_string(self)", "    }", "",
                     "    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {",
                     "        serde_json::from_str(json)", "    }", "}"])
        
        return '\n'.join(lines)
    
    def _generate_rust_lib_fallback(self, package_info: PackageInfo) -> str:
        """Fallback Rust lib.rs generation."""
        lines = [f"// {package_info.name} - ROS 2 message types", ""]
        for name in package_info.messages.keys():
            lines.append(f"pub mod {name.lower()};")
        lines.append("")
        lines.append("// Re-exports")
        for name in package_info.messages.keys():
            lines.append(f"pub use {name.lower()}::{name};")
        return '\n'.join(lines)
    
    def _generate_typescript_message_fallback(self, message: MessageInfo) -> str:
        """Fallback TypeScript message generation."""
        lines = [f"export interface {message.name} {{"]
        for field in message.fields:
            lines.append(f"  {field.name}: {self.get_typescript_type(field)};")
        lines.extend(["}", "", f"export function serialize{message.name}(msg: {message.name}): string {{",
                     "  return JSON.stringify(msg);", "}", "",
                     f"export function deserialize{message.name}(data: string): {message.name} {{",
                     f"  return JSON.parse(data) as {message.name};", "}"])
        return '\n'.join(lines)
    
    def _generate_typescript_index_fallback(self, package_info: PackageInfo) -> str:
        """Fallback TypeScript index generation."""
        lines = [f"// {package_info.name} - ROS 2 message types", ""]
        for name in package_info.messages.keys():
            lines.append(f"export * from './{name.lower()}';")
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='Generate ROS 2 packages for multiple languages')
    parser.add_argument('--input', '-i', required=True, action='append', help='Input .msg directories (can specify multiple)')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--languages', '-l', nargs='+', 
                       choices=['python', 'rust', 'typescript', 'c'],
                       default=['python', 'rust', 'typescript'],
                       help='Languages to generate')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    if not HAS_JINJA2:
        print("⚠️  Jinja2 not available, using fallback templates")
    
    generator = PackageGenerator()
    
    # Parse all input directories
    for input_path in args.input:
        if os.path.isfile(input_path):
            message = generator.parse_msg_file(input_path)
            package_name = message.package
            if package_name not in generator.packages:
                generator.packages[package_name] = PackageInfo(name=package_name)
            generator.packages[package_name].messages[message.name] = message
        else:
            for root, dirs, files in os.walk(input_path):
                for file in files:
                    if file.endswith('.msg'):
                        file_path = os.path.join(root, file)
                        message = generator.parse_msg_file(file_path)
                        package_name = message.package
                        if package_name not in generator.packages:
                            generator.packages[package_name] = PackageInfo(name=package_name)
                        generator.packages[package_name].messages[message.name] = message
    
    # Generate output for each language
    for lang in args.languages:
        lang_dir = Path(args.output) / lang
        lang_dir.mkdir(parents=True, exist_ok=True)
        
        for package_name, package_info in generator.packages.items():
            if args.verbose:
                print(f"Generating {lang}/{package_name}...")
            
            if lang == 'python':
                generator.generate_python_package(package_info, str(lang_dir))
            elif lang == 'rust':
                generator.generate_rust_package(package_info, str(lang_dir))
            elif lang == 'typescript':
                generator.generate_typescript_package(package_info, str(lang_dir))
    
    print(f"\n✅ Generated {len(generator.packages)} packages for {len(args.languages)} languages")
    print(f"📁 Output directory: {args.output}")
    print(f"\nStructure:")
    for lang in args.languages:
        print(f"  {args.output}/{lang}/")
        for package_name in generator.packages.keys():
            print(f"    └── {package_name}/")


if __name__ == '__main__':
    main()
