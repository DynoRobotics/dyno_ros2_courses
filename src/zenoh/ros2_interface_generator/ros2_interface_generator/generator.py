"""
Core Generator Module

Main orchestration for generating ROS2 interfaces across multiple
languages and encodings.
"""

from pathlib import Path
from typing import Dict, List, Optional, Set
from dataclasses import dataclass, field
import subprocess


@dataclass
class FieldInfo:
    """Information about a message field."""
    name: str
    type: str
    ros2_package: str = ""
    array_size: Optional[int] = None
    is_array: bool = False
    is_bounded_array: bool = False
    is_builtin: bool = False


@dataclass
class MessageInfo:
    """Information about a ROS 2 message."""
    name: str
    package: str
    fields: List[FieldInfo] = field(default_factory=list)
    dependencies: Set[str] = field(default_factory=set)
    type_hash: str = ""


class Generator:
    """
    Universal ROS2 interface generator.
    
    Generates code for ROS2 interfaces (messages, services, actions)
    in multiple target languages with multiple encoding backends.
    """
    
    BUILTIN_TYPES = {
        'bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32',
        'int64', 'uint64', 'float32', 'float64', 'string', 'wstring',
        'char', 'byte', 'time', 'duration'
    }
    
    def __init__(self, language: str = 'python', encoding: str = 'cdr'):
        """
        Initialize generator.
        
        Args:
            language: Target language (python, rust, typescript, c)
            encoding: Serialization encoding (cdr, json, msgpack, protobuf)
        """
        self.language = language.lower()
        self.encoding = encoding.lower()
        
        # Import appropriate backend
        if self.language == 'python':
            from .languages.python import PythonGenerator
            self.lang_backend = PythonGenerator(encoding=self.encoding)
        else:
            raise NotImplementedError(f"Language '{language}' not yet implemented")
    
    def generate(self, input_path: str = '/', output_path: str = 'output', 
                 packages: list = None):
        """
        Generate interfaces from ROS2 workspace.
        
        Args:
            input_path: Path to ROS2 workspace (or '/' for system packages)
            output_path: Output directory
            packages: List of package names to generate (None = discover all)
        """
        print(f"🔧 Generating {self.language} interfaces with {self.encoding} encoding")
        print(f"   Input: {input_path}")
        print(f"   Output: {output_path}")
        
        # Discover ROS2 packages and messages
        if packages:
            print(f"   Packages: {len(packages)} specified")
            messages = self._discover_specific_packages(packages)
        else:
            print(f"   Packages: auto-discovering...")
            messages = self._discover_messages(input_path)
        
        # Generate code using language backend
        self.lang_backend.generate(messages, Path(output_path))
        
        print(f"✅ Generation complete!")
    
    def _discover_specific_packages(self, package_names: list) -> Dict[str, Dict[str, MessageInfo]]:
        """
        Discover messages for specific packages by finding .msg files.
        
        Args:
            package_names: List of ROS2 package names
            
        Returns:
            Dictionary mapping package names to message definitions
        """
        import os
        import re
        from concurrent.futures import ThreadPoolExecutor, as_completed
        
        messages_by_package = {}
        
        # Common ROS2 install paths
        search_paths = [
            '/opt/ros/jazzy',
            '/opt/ros/humble', 
            '/opt/ros/iron',
            os.path.expanduser('~/ros2_ws/install'),
            os.path.expanduser('~/ws/install'),
        ]
        
        # Find .msg files for specified packages
        msg_files_by_package = {pkg: [] for pkg in package_names}
        
        for search_path in search_paths:
            if not os.path.exists(search_path):
                continue
                
            for root, dirs, files in os.walk(search_path):
                # Check if this is a msg directory
                if os.path.basename(root) == 'msg':
                    # Get package name from path (parent directory)
                    pkg_name = os.path.basename(os.path.dirname(root))
                    
                    if pkg_name in package_names:
                        for file in files:
                            if file.endswith('.msg'):
                                msg_files_by_package[pkg_name].append(os.path.join(root, file))
        
        # Parse the found .msg files
        for pkg_name, msg_files in msg_files_by_package.items():
            if not msg_files:
                print(f"  ⚠️  No .msg files found for {pkg_name}")
                continue
            
            messages_by_package[pkg_name] = {}
            
            for msg_file in msg_files:
                try:
                    msg_info = self._parse_msg_file(msg_file, pkg_name)
                    if msg_info:
                        messages_by_package[pkg_name][msg_info.name] = msg_info
                        print(f"  ✓ {pkg_name}/{msg_info.name}")
                except Exception as e:
                    print(f"  ⚠️  Error parsing {msg_file}: {e}")
        
        print(f"📦 Discovered {len(messages_by_package)} packages with {sum(len(msgs) for msgs in messages_by_package.values())} messages")
        
        # Compute type hashes in parallel (much faster!)
        if messages_by_package:
            print("⚡ Computing type hashes in parallel...")
            self._compute_hashes_parallel(messages_by_package)
        
        return messages_by_package
    
    def _compute_hashes_parallel(self, messages_by_package: Dict[str, Dict[str, MessageInfo]]):
        """Compute type hashes for all messages using RIHS01 algorithm."""
        from .rihs01_hasher import RIHS01Hasher
        
        # Use our RIHS01 implementation to calculate correct hashes
        hasher = RIHS01Hasher(messages_by_package)
        
        for pkg_name, messages in messages_by_package.items():
            for msg_name, msg_info in messages.items():
                if msg_info.type_hash is None:
                    try:
                        msg_info.type_hash = hasher.calculate_hash(pkg_name, msg_name)
                    except Exception as e:
                        print(f"  ⚠️  Error computing hash for {pkg_name}/{msg_name}: {e}")
                        # Fallback to simple hash
                        import hashlib
                        hash_input = f"{pkg_name}::{msg_name}"
                        hash_hex = hashlib.sha256(hash_input.encode()).hexdigest()
                        msg_info.type_hash = f"RIHS01_{hash_hex}"
    
    def _discover_messages(self, input_path: str) -> Dict[str, Dict[str, MessageInfo]]:
        """
        Discover all ROS2 message definitions.
        
        Returns:
            Dictionary mapping package names to message definitions
        """
        messages_by_package = {}
        
        # Use ros2 CLI to discover packages
        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'list'],
                capture_output=True,
                text=True,
                check=True
            )
            packages = result.stdout.strip().split('\n')
        except Exception as e:
            print(f"⚠️  Could not discover ROS2 packages: {e}")
            packages = []
        
        # For each package, discover messages
        for pkg_name in packages:
            try:
                result = subprocess.run(
                    ['ros2', 'interface', 'package', pkg_name],
                    capture_output=True,
                    text=True,
                    check=True,
                    timeout=2
                )
                
                interfaces = result.stdout.strip().split('\n')
                msg_interfaces = [i for i in interfaces if '/msg/' in i]
                
                if msg_interfaces:
                    messages_by_package[pkg_name] = {}
                    
                    for interface in msg_interfaces:
                        msg_name = interface.split('/msg/')[-1]
                        msg_info = self._parse_message_from_cli(pkg_name, msg_name)
                        if msg_info:
                            messages_by_package[pkg_name][msg_name] = msg_info
                            
            except Exception:
                continue  # Skip packages that don't have interfaces
        
        print(f"📦 Discovered {len(messages_by_package)} packages with messages")
        
        return messages_by_package
    
    def _parse_msg_file(self, file_path: str, package: str) -> Optional[MessageInfo]:
        """Parse a .msg file directly from the filesystem."""
        import re
        
        with open(file_path, 'r') as f:
            content = f.read()
        
        # ROS2 .msg files are already in PascalCase (e.g., PolygonInstance.msg)
        # Just use the stem directly - don't try to convert case!
        msg_name = Path(file_path).stem
        
        msg_info = MessageInfo(name=msg_name, package=package)
        
        # Parse field definitions
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if '=' in line:  # Constants
                continue
            
            field = self._parse_field_line(line, package)
            if field:
                msg_info.fields.append(field)
                if not field.is_builtin:
                    msg_info.dependencies.add(f"{field.ros2_package}.{field.type}")
        
        # Get type hash (lazy - only when needed for CDR encoding)
        # Computing hashes via CLI is slow (~580ms each), so we'll do it later in parallel
        msg_info.type_hash = None
        
        return msg_info
    
    def _parse_message_from_cli(self, package: str, name: str) -> Optional[MessageInfo]:
        """Parse message definition using ros2 CLI."""
        try:
            msg_type = f"{package}/msg/{name}"
            result = subprocess.run(
                ['ros2', 'interface', 'show', msg_type],
                capture_output=True,
                text=True,
                check=True,
                timeout=2
            )
            
            msg_info = MessageInfo(name=name, package=package)
            
            # Parse field definitions
            for line in result.stdout.split('\n'):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if '=' in line:  # Constants
                    continue
                
                field = self._parse_field_line(line, package)
                if field:
                    msg_info.fields.append(field)
            
            # Get type hash
            msg_info.type_hash = self._get_type_hash(package, name)
            
            return msg_info
            
        except Exception as e:
            return None
    
    def _parse_field_line(self, line: str, current_package: str) -> Optional[FieldInfo]:
        """Parse a single field definition line."""
        import re
        
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
        
        is_builtin = field_type in self.BUILTIN_TYPES
        
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
    
    def _get_type_hash(self, package: str, name: str) -> str:
        """
        Get RIHS01 type hash for a message type.
        
        NOTE: This method is deprecated. Use RIHS01Hasher directly in _compute_hashes_parallel.
        Kept for compatibility with any code that still calls it.
        """
        # Fallback to simple hash
        import hashlib
        hash_input = f"{package}::{name}"
        hash_hex = hashlib.sha256(hash_input.encode()).hexdigest()
        return f"RIHS01_{hash_hex}"


def generate(language: str = 'python',
             input_path: str = '/', output_path: str = 'output',
             packages: list = None, preset: str = None):
    """
    Convenience function to generate interfaces.
    
    Note: Encoding (CDR, JSON, MessagePack) is selected at RUNTIME, not generation time!
    All generated messages support multiple encodings automatically.
    
    Args:
        language: Target language ('python', 'rust', etc.)
        input_path: ROS2 workspace path
        output_path: Output directory
        packages: Specific packages to generate
        preset: Package preset ('essential', 'common', 'standard', 'all')
    
    Example:
        # Generate essential packages
        generate(language='python', preset='essential',
                output_path='ros2_interfaces_py')
        
        # Generate specific packages
        generate(language='python', 
                packages=['std_msgs', 'geometry_msgs'],
                output_path='ros2_interfaces_py')
        
        # Messages support runtime encoding selection:
        #   msg.serialize('cdr')   # or 'json' or 'msgpack'
        #   Twist.deserialize(data, 'json')
    """
    # Handle preset
    if preset and not packages:
        from .package_lists import get_package_list
        packages = get_package_list(preset=preset)
    
    # For Python, we default to CDR encoding metadata (TYPE_HASH, DDS_TYPE_NAME)
    # but the generated code supports all encodings at runtime
    encoding = 'cdr' if language == 'python' else 'cdr'
    
    gen = Generator(language=language, encoding=encoding)
    gen.generate(input_path=input_path, output_path=output_path, packages=packages)

