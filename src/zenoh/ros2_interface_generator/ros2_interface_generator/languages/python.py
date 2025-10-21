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
    
    def generate(self, messages_by_package: Dict, services_by_package: Dict, output_dir: Path):
        """Generate Python package with all messages and services."""
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
            services = services_by_package.get(ros2_pkg, {})
            self._generate_package(pkg_dir, ros2_pkg, messages, services)
        
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
    
    def _generate_package(self, pkg_dir: Path, ros2_pkg: str, messages: Dict, services: Dict = None):
        """Generate all messages and services for a ROS2 package."""
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
        
        # Generate services if any
        has_srv = False
        if services:
            srv_dir = pkg_subdir / "srv"
            srv_dir.mkdir(exist_ok=True)
            
            for srv_name, service in services.items():
                self._generate_service_file(srv_dir, service, ros2_pkg)
            
            # Generate srv/__init__.py
            self._generate_srv_init(srv_dir, services.keys())
            has_srv = True
        
        # Generate package __init__.py
        init_content = f"# {ros2_pkg} package\nfrom . import msg\n"
        if has_srv:
            init_content += "from . import srv\n"
        (pkg_subdir / "__init__.py").write_text(init_content)
    
    def _generate_message_file(self, msg_dir: Path, message, ros2_pkg: str):
        """Generate Python file for a single message."""
        if not HAS_JINJA2:
            raise RuntimeError("Jinja2 is required for code generation. Install with: pip install jinja2")
        
        filename = msg_dir / f"{message.name.lower()}.py"
        
        # Collect imports
        imports_same_pkg = set()
        imports_other_pkg_types = {}  # pkg -> set of types
        
        for field in message.fields:
            if not field.is_builtin:
                if field.ros2_package == message.package:
                    imports_same_pkg.add((field.type.lower(), field.type))
                else:
                    if field.ros2_package not in imports_other_pkg_types:
                        imports_other_pkg_types[field.ros2_package] = set()
                    imports_other_pkg_types[field.ros2_package].add(field.type)
        
        # Load template
        template = self.env.get_template("message.py.jinja2")
        
        def get_type_wrapper(field):
            return self._get_python_type(field, message.package)
        
        def get_default_wrapper(field):
            return self._get_default_value(field, message.package)
        
        # Determine DDS type name (use ::srv:: for service types, ::msg:: otherwise)
        if getattr(message, 'is_service_type', False):
            # Service Request/Response types use ::srv::
            dds_type_name = f"{message.package}::srv::dds_::{message.name}_"
        else:
            # Regular messages use ::msg::
            dds_type_name = f"{message.package}::msg::dds_::{message.name}_"
        
        # Always generate ROS2 metadata (type hash and DDS type name)
        # Multi-encoding support is provided via get_serializer/get_deserializer methods
        content = template.render(
            message=message,
            imports_same_pkg=sorted(imports_same_pkg),
            imports_other_pkg_types=imports_other_pkg_types,
            get_python_type=get_type_wrapper,
            get_default_value=get_default_wrapper,
            encoding_name=self.encoding_name,
            needs_type_hash=True,  # Always needed for ROS2 interop
            needs_dds_type_name=True,  # Always needed for ROS2 interop
            dds_type_name=dds_type_name,
        )
        
        filename.write_text(content)
    
    def _get_python_type(self, field, current_package: str) -> str:
        """Get Python type annotation for a field."""
        if field.is_builtin:
            base_type = self.TYPE_MAPPINGS.get(field.type, 'str')
        else:
            # Same package: use simple name (imported via from .module import Class)
            # Different package: use string annotation to avoid circular imports
            if field.ros2_package == current_package:
                base_type = field.type
            else:
                base_type = f"'ros2_interfaces_py.{field.ros2_package}.msg.{field.type}'"
        
        if field.is_array:
            return f"List[{base_type}]"
        return base_type
    
    def _get_default_value(self, field, current_package: str) -> str:
        """Get default value for a field."""
        if field.is_array:
            return "field(default_factory=list)"
        
        # Builtin types - can use direct defaults
        if field.is_builtin:
            if field.type in ('int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'):
                return "0"
            elif field.type in ('float32', 'float64'):
                return "0.0"
            elif field.type == 'bool':
                return "False"
            elif field.type in ('string', 'wstring'):
                return '""'
            elif field.type == 'byte':
                return "0"
            else:
                return "0"
        
        # Nested message types - use default_factory for mutable defaults
        if field.ros2_package == current_package:
            # Same package - class is imported, use directly
            return f"field(default_factory={field.type})"
        else:
            # Cross-package - use lambda with full path to avoid circular imports
            return f"field(default_factory=lambda: ros2_interfaces_py.{field.ros2_package}.msg.{field.type}())"
    
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
    
    def _generate_service_file(self, srv_dir: Path, service, ros2_pkg: str):
        """Generate Python file for a single service (composes from message template)."""
        if not HAS_JINJA2:
            raise RuntimeError("Jinja2 is required for code generation. Install with: pip install jinja2")
        
        filename = srv_dir / f"{service.name.lower()}.py"
        
        # Helper to get Python type for service message fields
        def get_type_wrapper(field):
            return self._get_python_type(field, service.package)
        
        # Collect imports for Request/Response
        # For services, nested messages must be imported from ../msg (not from srv)
        imports_same_pkg = set()
        imports_other_pkgs = set()
        for field in service.request.fields + service.response.fields:
            if not field.is_builtin:
                if field.ros2_package == service.package:
                    # Import from ../msg directory (template will add 'from .')
                    imports_same_pkg.add((f".msg.{field.type.lower()}", field.type))
                else:
                    imports_other_pkgs.add(field.ros2_package)
        
        # Load template
        template = self.env.get_template("service.py.jinja2")
        
        # Determine DDS type names for Request/Response (use ::srv::)
        request_dds_type_name = f"{service.package}::srv::dds_::{service.request.name}_"
        response_dds_type_name = f"{service.package}::srv::dds_::{service.response.name}_"
        
        def get_default_wrapper(field):
            return self._get_default_value(field, service.package)
        
        # Render with same context as messages, but we need a custom approach
        # since we're rendering two messages in one file
        content = template.render(
            service=service,
            get_python_type=get_type_wrapper,
            get_default_value=get_default_wrapper,
            encoding_name=self.encoding_name,
            imports_same_pkg=sorted(imports_same_pkg),
            imports_other_pkgs=sorted(imports_other_pkgs),
            needs_type_hash=True,
            needs_dds_type_name=True,
            request_dds_type_name=request_dds_type_name,
            response_dds_type_name=response_dds_type_name,
            service_type_hash=service.type_hash,
        )
        filename.write_text(content)
    
    def _generate_srv_init(self, srv_dir: Path, service_names):
        """Generate srv/__init__.py."""
        lines = ["# Services", ""]
        for srv_name in service_names:
            lines.append(f"from .{srv_name.lower()} import {srv_name}")
        lines.append("")
        lines.append("__all__ = [")
        for srv_name in service_names:
            lines.append(f"    '{srv_name}',")
        lines.append("]")
        (srv_dir / "__init__.py").write_text('\n'.join(lines))
    
    def _generate_main_init(self, pkg_dir: Path, ros2_packages):
        """Generate main __init__.py."""
        lines = ["# ROS 2 CDR Interfaces - Unified Python Package", ""]
        for ros2_pkg in ros2_packages:
            lines.append(f"from . import {ros2_pkg}")
        (pkg_dir / "__init__.py").write_text('\n'.join(lines))
    
