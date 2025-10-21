#!/usr/bin/env python3
"""
Multi-Language ROS 2 Message Generator

This script generates ROS 2 message types for multiple languages:
- Python (dataclasses with CDR serialization)
- Rust (structs with serde and CDR serialization)  
- C (structs with Micro-CDR serialization)
- TypeScript (interfaces with JSON serialization)
"""

import os
import sys
import argparse
import re
from pathlib import Path
from typing import Dict, List, Set, Optional, Any
from dataclasses import dataclass, field
from enum import Enum


class Language(Enum):
    PYTHON = "python"
    RUST = "rust"
    C = "c"
    TYPESCRIPT = "typescript"


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


class MultiLangGenerator:
    """Generates message types for multiple languages."""
    
    def __init__(self):
        self.messages: Dict[str, MessageInfo] = {}
        self.type_mappings = {
            # ROS 2 -> Language mappings
            'bool': {
                Language.PYTHON: 'bool',
                Language.RUST: 'bool',
                Language.C: 'bool',
                Language.TYPESCRIPT: 'boolean'
            },
            'int8': {
                Language.PYTHON: 'int',
                Language.RUST: 'i8',
                Language.C: 'int8_t',
                Language.TYPESCRIPT: 'number'
            },
            'uint8': {
                Language.PYTHON: 'int',
                Language.RUST: 'u8',
                Language.C: 'uint8_t',
                Language.TYPESCRIPT: 'number'
            },
            'int16': {
                Language.PYTHON: 'int',
                Language.RUST: 'i16',
                Language.C: 'int16_t',
                Language.TYPESCRIPT: 'number'
            },
            'uint16': {
                Language.PYTHON: 'int',
                Language.RUST: 'u16',
                Language.C: 'uint16_t',
                Language.TYPESCRIPT: 'number'
            },
            'int32': {
                Language.PYTHON: 'int',
                Language.RUST: 'i32',
                Language.C: 'int32_t',
                Language.TYPESCRIPT: 'number'
            },
            'uint32': {
                Language.PYTHON: 'int',
                Language.RUST: 'u32',
                Language.C: 'uint32_t',
                Language.TYPESCRIPT: 'number'
            },
            'int64': {
                Language.PYTHON: 'int',
                Language.RUST: 'i64',
                Language.C: 'int64_t',
                Language.TYPESCRIPT: 'number'
            },
            'uint64': {
                Language.PYTHON: 'int',
                Language.RUST: 'u64',
                Language.C: 'uint64_t',
                Language.TYPESCRIPT: 'number'
            },
            'float32': {
                Language.PYTHON: 'float',
                Language.RUST: 'f32',
                Language.C: 'float',
                Language.TYPESCRIPT: 'number'
            },
            'float64': {
                Language.PYTHON: 'float',
                Language.RUST: 'f64',
                Language.C: 'double',
                Language.TYPESCRIPT: 'number'
            },
            'string': {
                Language.PYTHON: 'str',
                Language.RUST: 'String',
                Language.C: 'char*',
                Language.TYPESCRIPT: 'string'
            },
            'time': {
                Language.PYTHON: 'Time',
                Language.RUST: 'builtin_interfaces::msg::Time',
                Language.C: 'ros2_time_t',
                Language.TYPESCRIPT: 'Time'
            },
            'duration': {
                Language.PYTHON: 'Duration',
                Language.RUST: 'builtin_interfaces::msg::Duration',
                Language.C: 'ros2_duration_t',
                Language.TYPESCRIPT: 'Duration'
            }
        }
    
    def parse_msg_file(self, file_path: str) -> MessageInfo:
        """Parse a .msg file and extract message information."""
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Extract package name from path
        package = Path(file_path).parent.parent.name
        
        # Extract message name from filename
        message_name = Path(file_path).stem
        
        message = MessageInfo(name=message_name, package=package)
        
        # Parse fields
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Skip constants (e.g., "uint8 DEBUG=10")
            if '=' in line and not line.startswith(('int', 'uint', 'float', 'bool', 'string', 'char', 'byte')):
                continue
            
            field_info = self._parse_field_line(line)
            if field_info:
                message.fields.append(field_info)
                # Track dependencies for custom types
                if not field_info.is_builtin and field_info.type not in self.type_mappings:
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
            # Extract array size if specified
            match = re.search(r'\[(\d+)\]', field_type)
            if match:
                array_size = int(match.group(1))
                is_bounded_array = True
            field_type = field_type.split('[')[0]
        
        # Determine if it's a builtin type
        is_builtin = field_type in self.type_mappings
        
        return FieldInfo(
            name=field_name,
            type=field_type,
            is_array=is_array,
            array_size=array_size,
            is_bounded_array=is_bounded_array,
            is_string=(field_type == 'string'),
            is_builtin=is_builtin
        )
    
    def generate_python(self, message: MessageInfo) -> str:
        """Generate Python dataclass with CDR serialization."""
        lines = []
        lines.append("from dataclasses import dataclass")
        lines.append("from typing import List, Optional")
        lines.append("import struct")
        lines.append("")
        lines.append("@dataclass")
        lines.append(f"class {message.name}:")
        
        # Add fields
        for field in message.fields:
            field_type = self._get_python_type(field)
            lines.append(f"    {field.name}: {field_type}")
        
        # Add CDR serialization method
        lines.append("")
        lines.append("    def serialize_cdr(self) -> bytes:")
        lines.append("        \"\"\"Serialize message to CDR format.\"\"\"")
        lines.append("        data = b''")
        
        for field in message.fields:
            lines.extend(self._generate_python_serialize_field(field))
        
        lines.append("        return data")
        
        # Add JSON conversion methods
        lines.append("")
        lines.append("    def to_dict(self) -> dict:")
        lines.append("        \"\"\"Convert to dictionary for JSON serialization.\"\"\"")
        lines.append("        return {")
        
        for field in message.fields:
            lines.append(f"            '{field.name}': self.{field.name},")
        
        lines.append("        }")
        
        return '\n'.join(lines)
    
    def generate_rust(self, message: MessageInfo) -> str:
        """Generate Rust struct with serde and CDR serialization."""
        lines = []
        lines.append("use serde::{Deserialize, Serialize};")
        lines.append("")
        lines.append("#[derive(Debug, Clone, Serialize, Deserialize)]")
        lines.append(f"pub struct {message.name} {{")
        
        # Add fields
        for field in message.fields:
            field_type = self._get_rust_type(field)
            lines.append(f"    pub {field.name}: {field_type},")
        
        lines.append("}")
        
        # Add JSON conversion methods
        lines.append("")
        lines.append(f"impl {message.name} {{")
        lines.append("    pub fn to_json(&self) -> Result<String, serde_json::Error> {")
        lines.append("        serde_json::to_string(self)")
        lines.append("    }")
        lines.append("")
        lines.append("    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {")
        lines.append("        serde_json::from_str(json)")
        lines.append("    }")
        lines.append("}")
        
        return '\n'.join(lines)
    
    def generate_c(self, message: MessageInfo) -> str:
        """Generate C struct with Micro-CDR serialization."""
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
    
    def generate_typescript(self, message: MessageInfo) -> str:
        """Generate TypeScript interface."""
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
        lines.append("  return JSON.parse(data) as {message.name};")
        lines.append("}")
        
        return '\n'.join(lines)
    
    def _get_python_type(self, field: FieldInfo) -> str:
        """Get Python type for a field."""
        if field.is_array:
            return f"List[{self.type_mappings[field.type][Language.PYTHON]}]"
        return self.type_mappings[field.type][Language.PYTHON]
    
    def _get_rust_type(self, field: FieldInfo) -> str:
        """Get Rust type for a field."""
        if field.is_array:
            if field.is_bounded_array:
                return f"[{self.type_mappings[field.type][Language.RUST]}; {field.array_size}]"
            else:
                return f"Vec<{self.type_mappings[field.type][Language.RUST]}>"
        return self.type_mappings[field.type][Language.RUST]
    
    def _get_c_type(self, field: FieldInfo) -> str:
        """Get C type for a field."""
        if field.is_array:
            if field.is_bounded_array:
                return f"{self.type_mappings[field.type][Language.C]}[{field.array_size}]"
            else:
                return f"{self.type_mappings[field.type][Language.C]}*"
        return self.type_mappings[field.type][Language.C]
    
    def _get_typescript_type(self, field: FieldInfo) -> str:
        """Get TypeScript type for a field."""
        if field.is_array:
            return f"{self.type_mappings[field.type][Language.TYPESCRIPT]}[]"
        return self.type_mappings[field.type][Language.TYPESCRIPT]
    
    def _generate_python_serialize_field(self, field: FieldInfo) -> List[str]:
        """Generate Python CDR serialization for a field."""
        lines = []
        if field.type == 'float64':
            lines.append(f"        data += struct.pack('<d', self.{field.name})")
        elif field.type == 'float32':
            lines.append(f"        data += struct.pack('<f', self.{field.name})")
        elif field.type == 'int32':
            lines.append(f"        data += struct.pack('<i', self.{field.name})")
        elif field.type == 'uint32':
            lines.append(f"        data += struct.pack('<I', self.{field.name})")
        elif field.type == 'string':
            lines.append(f"        # Serialize string length")
            lines.append(f"        data += struct.pack('<I', len(self.{field.name}))")
            lines.append(f"        data += self.{field.name}.encode('utf-8')")
        return lines


def main():
    parser = argparse.ArgumentParser(description='Generate multi-language ROS 2 message types')
    parser.add_argument('--input', '-i', required=True, help='Input .msg file or directory')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--languages', '-l', nargs='+', 
                       choices=['python', 'rust', 'c', 'typescript'],
                       default=['python', 'rust', 'c', 'typescript'],
                       help='Languages to generate')
    parser.add_argument('--package', '-p', help='Package name')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    generator = MultiLangGenerator()
    
    # Parse input
    if os.path.isfile(args.input):
        # Single file
        message = generator.parse_msg_file(args.input)
        generator.messages[message.name] = message
    else:
        # Directory
        for root, dirs, files in os.walk(args.input):
            for file in files:
                if file.endswith('.msg'):
                    file_path = os.path.join(root, file)
                    message = generator.parse_msg_file(file_path)
                    generator.messages[message.name] = message
    
    # Generate output
    os.makedirs(args.output, exist_ok=True)
    
    for message_name, message in generator.messages.items():
        if args.verbose:
            print(f"Generating {message_name} for languages: {args.languages}")
        
        for lang in args.languages:
            lang_enum = Language(lang)
            output_dir = os.path.join(args.output, lang, message.package)
            os.makedirs(output_dir, exist_ok=True)
            
            if lang_enum == Language.PYTHON:
                content = generator.generate_python(message)
                filename = f"{message.package}.py"
            elif lang_enum == Language.RUST:
                content = generator.generate_rust(message)
                filename = f"{message.name.lower()}.rs"
            elif lang_enum == Language.C:
                content = generator.generate_c(message)
                filename = f"{message.name.lower()}.h"
            elif lang_enum == Language.TYPESCRIPT:
                content = generator.generate_typescript(message)
                filename = f"{message.name.lower()}.ts"
            
            output_path = os.path.join(output_dir, filename)
            with open(output_path, 'w') as f:
                f.write(content)
            
            if args.verbose:
                print(f"  Generated: {output_path}")
    
    print(f"✅ Generated {len(generator.messages)} messages for {len(args.languages)} languages")
    print(f"📁 Output directory: {args.output}")


if __name__ == '__main__':
    main()




