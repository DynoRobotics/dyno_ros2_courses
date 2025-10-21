"""
Python Code Generator

Generates Python code for ROS2 interfaces with pycdr2 CDR serialization.
Based on the proven generate_unified_types.py implementation.
"""

from pathlib import Path
from typing import Dict, Set, Tuple
import hashlib
import subprocess

try:
    from jinja2 import Environment, Template
    HAS_JINJA2 = True
except ImportError:
    HAS_JINJA2 = False


class PythonGenerator:
    """
    Python code generator for ROS2 interfaces with pycdr2 CDR serialization.
    
    Generates dataclass-based message types with:
    - pycdr2 IdlStruct base class for CDR serialization
    - TYPE_HASH and DDS_TYPE_NAME constants
    - serialize() and deserialize() methods
    """
    
    BUILTIN_TYPES = {
        'bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32',
        'int64', 'uint64', 'float32', 'float64', 'string', 'wstring',
        'char', 'byte', 'time', 'duration'
    }
    
    TYPE_MAPPINGS = {
        'bool': 'bool',
        'int8': 'int8',
        'uint8': 'uint8',
        'int16': 'int16',
        'uint16': 'uint16',
        'int32': 'int32',
        'uint32': 'uint32',
        'int64': 'int64',
        'uint64': 'uint64',
        'float32': 'float32',
        'float64': 'float64',
        'string': 'str',
        'wstring': 'str',
        'char': 'str',
        'byte': 'uint8',
    }
    
    def __init__(self, encoding: str = 'cdr'):
        self.encoding_name = encoding.lower()
        
        # Setup template environment
        if HAS_JINJA2:
            from jinja2 import FileSystemLoader
            template_dir = Path(__file__).parent.parent / "templates" / "python"
            if template_dir.exists():
                self.env = Environment(loader=FileSystemLoader(str(template_dir)))
            else:
                # Fallback to inline templates if directory doesn't exist
                self.env = Environment()
        else:
            self.env = None
    
    def generate(self, messages_by_package: Dict, output_dir: Path):
        """Generate Python package with all messages."""
        pkg_dir = output_dir / "ros2_interfaces_py"
        pkg_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate setup.py
        self._generate_setup_py(output_dir)
        
        # Generate shared encoding utilities
        self._generate_encodings_module(pkg_dir)
        
        # Create main package __init__.py
        (pkg_dir / "__init__.py").write_text("")
        
        # Generate each ROS2 package
        for ros2_pkg, messages in messages_by_package.items():
            self._generate_package(pkg_dir, ros2_pkg, messages)
        
        # Generate top-level __init__.py with imports
        self._generate_main_init(pkg_dir, messages_by_package.keys())
        
        print(f"  ✓ Generated Python package at {pkg_dir}")
    
    def _generate_setup_py(self, output_dir: Path):
        """Generate setup.py for the package."""
        if not HAS_JINJA2:
            raise RuntimeError("Jinja2 is required for code generation. Install with: pip install jinja2")
        
        template = self.env.get_template("setup.py.jinja2")
        content = template.render()
        (output_dir / "setup.py").write_text(content)
    
    def _generate_encodings_module(self, pkg_dir: Path):
        """Generate shared encoding utilities module."""
        if not HAS_JINJA2:
            raise RuntimeError("Jinja2 is required for code generation. Install with: pip install jinja2")
        
        template = self.env.get_template("_encodings.py.jinja2")
        content = template.render()
        (pkg_dir / "_encodings.py").write_text(content)
        print(f"  ✓ Generated shared encoding utilities")
    
    def _generate_package(self, pkg_dir: Path, ros2_pkg: str, messages: Dict):
        """Generate all messages for a ROS2 package."""
        # Create package directory structure
        pkg_subdir = pkg_dir / ros2_pkg
        pkg_subdir.mkdir(exist_ok=True)
        
        msg_dir = pkg_subdir / "msg"
        msg_dir.mkdir(exist_ok=True)
        
        # Generate each message
        for msg_name, message in messages.items():
            self._generate_message_file(msg_dir, message, ros2_pkg)
        
        # Generate msg/__init__.py
        self._generate_msg_init(msg_dir, messages.keys())
        
        # Generate package __init__.py
        (pkg_subdir / "__init__.py").write_text(f"# {ros2_pkg} package\nfrom . import msg\n")
    
    def _generate_message_file(self, msg_dir: Path, message, ros2_pkg: str):
        """Generate Python file for a single message."""
        if not HAS_JINJA2:
            raise RuntimeError("Jinja2 is required for code generation. Install with: pip install jinja2")
        
        filename = msg_dir / f"{message.name.lower()}.py"
        
        # Collect imports
        imports_same_pkg = set()
        imports_other_pkgs = set()
        
        for field in message.fields:
            if not field.is_builtin:
                if field.ros2_package == message.package:
                    imports_same_pkg.add((field.type.lower(), field.type))
                else:
                    imports_other_pkgs.add(field.ros2_package)
        
        # Load template
        template = self.env.get_template("message.py.jinja2")
        
        def get_type_wrapper(field):
            return self._get_python_type(field, message.package)
        
        # Always generate ROS2 metadata (type hash and DDS type name)
        # Multi-encoding support is provided via get_serializer/get_deserializer methods
        content = template.render(
            message=message,
            imports_same_pkg=sorted(imports_same_pkg),
            imports_other_pkgs=sorted(imports_other_pkgs),
            get_python_type=get_type_wrapper,
            encoding_name=self.encoding_name,
            needs_type_hash=True,  # Always needed for ROS2 interop
            needs_dds_type_name=True,  # Always needed for ROS2 interop
        )
        
        filename.write_text(content)
    
    def _get_python_type(self, field, current_package: str) -> str:
        """Get Python type annotation for a field."""
        if field.is_builtin:
            base_type = self.TYPE_MAPPINGS.get(field.type, 'str')
        else:
            # Import from same or different package
            if field.ros2_package == current_package:
                base_type = field.type
            else:
                base_type = f"'ros2_interfaces_py.{field.ros2_package}.msg.{field.type}'"
        
        if field.is_array:
            return f"List[{base_type}]"
        return base_type
    
    def _generate_msg_init(self, msg_dir: Path, message_names):
        """Generate msg/__init__.py."""
        lines = ["# Messages", ""]
        for msg_name in message_names:
            lines.append(f"from .{msg_name.lower()} import {msg_name}")
        lines.append("")
        lines.append("__all__ = [")
        for msg_name in message_names:
            lines.append(f"    '{msg_name}',")
        lines.append("]")
        (msg_dir / "__init__.py").write_text('\n'.join(lines))
    
    def _generate_main_init(self, pkg_dir: Path, ros2_packages):
        """Generate main __init__.py."""
        lines = ["# ROS 2 CDR Interfaces - Unified Python Package", ""]
        for ros2_pkg in ros2_packages:
            lines.append(f"from . import {ros2_pkg}")
        (pkg_dir / "__init__.py").write_text('\n'.join(lines))
    
