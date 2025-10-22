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
    is_fixed_size_array: bool = False  # True for [N], False for [<=N] or []
    is_builtin: bool = False
    namespace: str = "msg"  # Can be 'msg', 'srv', or 'action'
    default_value: Optional[str] = None  # Default value as string (to be parsed by backend)


@dataclass
class MessageInfo:
    """Information about a ROS 2 message."""
    name: str
    package: str
    fields: List[FieldInfo] = field(default_factory=list)
    dependencies: Set[str] = field(default_factory=set)
    type_hash: str = ""
    is_service_type: bool = False  # True for service Request/Response
    constants: Dict[str, tuple] = field(default_factory=dict)  # {name: (type, value)}


@dataclass
class ServiceInfo:
    """Information about a ROS 2 service."""
    name: str
    package: str
    request: MessageInfo
    response: MessageInfo
    type_hash: str = ""  # Service-level type hash (computed from Request+Response)


@dataclass
class ActionInfo:
    """Information about a ROS 2 action."""
    name: str
    package: str
    goal: MessageInfo
    result: MessageInfo
    feedback: MessageInfo
    # Generated types
    send_goal_service: Optional['ServiceInfo'] = None
    get_result_service: Optional['ServiceInfo'] = None
    feedback_message: Optional[MessageInfo] = None
    type_hash: str = ""  # Action-level type hash


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
        self.messages_by_package = {}
        self.services_by_package = {}
        self.actions_by_package = {}
        
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
            self.messages_by_package = self._discover_specific_packages(packages)
        else:
            print(f"   Packages: auto-discovering...")
            self.messages_by_package = self._discover_messages(input_path)
        
        # Generate code using language backend
        self.lang_backend.generate(self.messages_by_package, self.services_by_package, self.actions_by_package, Path(output_path))
        
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
            '/opt/ros2_rust/install',  # ros2_rust builds
            '/home/ubuntu/ws/install',  # Local workspace
            os.path.expanduser('~/ros2_ws/install'),
            os.path.expanduser('~/ws/install'),
        ]
        
        # Find .msg, .srv, and .action files for specified packages
        msg_files_by_package = {pkg: [] for pkg in package_names}
        srv_files_by_package = {pkg: [] for pkg in package_names}
        action_files_by_package = {pkg: [] for pkg in package_names}
        
        for search_path in search_paths:
            if not os.path.exists(search_path):
                continue
                
            for root, dirs, files in os.walk(search_path):
                dir_name = os.path.basename(root)
                # Get package name from path (parent directory)
                pkg_name = os.path.basename(os.path.dirname(root))
                
                if pkg_name in package_names:
                    # Check if this is a msg directory
                    if dir_name == 'msg':
                        for file in files:
                            if file.endswith('.msg'):
                                msg_files_by_package[pkg_name].append(os.path.join(root, file))
                    # Check if this is a srv directory
                    elif dir_name == 'srv':
                        for file in files:
                            if file.endswith('.srv'):
                                srv_files_by_package[pkg_name].append(os.path.join(root, file))
                    # Check if this is an action directory
                    elif dir_name == 'action':
                        for file in files:
                            if file.endswith('.action'):
                                action_files_by_package[pkg_name].append(os.path.join(root, file))
        
        # Parse the found .msg files
        for pkg_name, msg_files in msg_files_by_package.items():
            if not msg_files:
                continue
            
            if pkg_name not in messages_by_package:
                messages_by_package[pkg_name] = {}
            
            for msg_file in msg_files:
                try:
                    msg_info = self._parse_msg_file(msg_file, pkg_name)
                    if msg_info:
                        messages_by_package[pkg_name][msg_info.name] = msg_info
                        print(f"  ✓ {pkg_name}/msg/{msg_info.name}")
                except Exception as e:
                    print(f"  ⚠️  Error parsing {msg_file}: {e}")
        
        # Parse the found .srv files
        for pkg_name, srv_files in srv_files_by_package.items():
            if not srv_files:
                continue
            
            if pkg_name not in self.services_by_package:
                self.services_by_package[pkg_name] = {}
            if pkg_name not in messages_by_package:
                messages_by_package[pkg_name] = {}
            
            for srv_file in srv_files:
                try:
                    srv_info = self._parse_srv_file(srv_file, pkg_name)
                    if srv_info:
                        self.services_by_package[pkg_name][srv_info.name] = srv_info
                        # Mark Request/Response as service types (embedded in .srv file, not standalone)
                        srv_info.request.is_service_type = True
                        srv_info.response.is_service_type = True
                        # Don't add to messages_by_package - they're embedded in the service file
                        # and shouldn't be generated as standalone message files
                        print(f"  ✓ {pkg_name}/srv/{srv_info.name}")
                except Exception as e:
                    print(f"  ⚠️  Error parsing {srv_file}: {e}")
        
        # Parse the found .action files
        for pkg_name, action_files in action_files_by_package.items():
            if not action_files:
                continue
            
            if pkg_name not in self.actions_by_package:
                self.actions_by_package[pkg_name] = {}
            if pkg_name not in messages_by_package:
                messages_by_package[pkg_name] = {}
            
            for action_file in action_files:
                try:
                    action_info = self._parse_action_file(action_file, pkg_name)
                    if action_info:
                        self.actions_by_package[pkg_name][action_info.name] = action_info
                        # Actions are self-contained - everything in one file
                        # No pollution of msg/ or srv/ folders!
                        print(f"  ✓ {pkg_name}/action/{action_info.name}")
                except Exception as e:
                    print(f"  ⚠️  Error parsing {action_file}: {e}")
        
        # Detect and warn about messages with fixed-size arrays of empty structs
        # This is a known pycdr2 limitation - skip generation for these
        messages_to_skip = self._warn_about_empty_struct_arrays(messages_by_package)
        
        # Remove problematic messages from the generation queue
        for pkg_name, msg_name in messages_to_skip:
            if pkg_name in messages_by_package and msg_name in messages_by_package[pkg_name]:
                del messages_by_package[pkg_name][msg_name]
        
        msg_count = sum(len(msgs) for msgs in messages_by_package.values())
        srv_count = sum(len(srvs) for srvs in self.services_by_package.values())
        action_count = sum(len(acts) for acts in self.actions_by_package.values())
        print(f"📦 Discovered {len(messages_by_package)} packages with {msg_count} messages")
        if srv_count > 0:
            print(f"🔧 Discovered {srv_count} services")
        if action_count > 0:
            print(f"⚡ Discovered {action_count} actions")
        
        # Compute type hashes in parallel (much faster!)
        if messages_by_package:
            print("⚡ Computing type hashes in parallel...")
            self._compute_hashes_parallel(messages_by_package)
        
        return messages_by_package
    
    def _compute_hashes_parallel(self, messages_by_package: Dict[str, Dict[str, MessageInfo]]):
        """Compute type hashes for all messages and services using RIHS01 algorithm."""
        from .rihs01_hasher import RIHS01Hasher
        
        # For action hash computation, we need access to dependency packages
        # Discover commonly needed packages for reference resolution (cached to avoid repeated filesystem walks)
        if not hasattr(self, '_dependency_cache'):
            self._dependency_cache = {}
            common_deps = ['unique_identifier_msgs', 'builtin_interfaces', 'std_msgs', 'service_msgs']
            for dep_pkg in common_deps:
                if dep_pkg not in messages_by_package:
                    try:
                        dep_messages = self._discover_specific_packages([dep_pkg])
                        self._dependency_cache.update(dep_messages)
                    except:
                        pass  # Dependency not available, skip
        
        # Merge current packages with cached dependencies
        all_messages_for_hasher = dict(messages_by_package)
        all_messages_for_hasher.update(self._dependency_cache)
        
        # Use our RIHS01 implementation with extended messages for reference resolution
        hasher = RIHS01Hasher(all_messages_for_hasher)
        
        # Compute message hashes (skip service types, they'll be handled separately)
        for pkg_name, messages in messages_by_package.items():
            for msg_name, msg_info in messages.items():
                if not msg_info.type_hash and not msg_info.is_service_type:
                    try:
                        msg_info.type_hash = hasher.calculate_hash(pkg_name, msg_name)
                    except Exception as e:
                        print(f"  ⚠️  Error computing hash for {pkg_name}/{msg_name}: {e}")
                        # Fallback to simple hash
                        import hashlib
                        hash_input = f"{pkg_name}::{msg_name}"
                        hash_hex = hashlib.sha256(hash_input.encode()).hexdigest()
                        msg_info.type_hash = f"RIHS01_{hash_hex}"
        
        # Compute service hashes
        for pkg_name, services in self.services_by_package.items():
            for srv_name, srv_info in services.items():
                if not srv_info.type_hash:
                    try:
                        # Temporarily add Request/Response types to hasher's type_lookup
                        # so calculate_hash can find them
                        request_type_name = f"{pkg_name}/srv/{srv_name}_Request"
                        response_type_name = f"{pkg_name}/srv/{srv_name}_Response"
                        
                        hasher.type_lookup[request_type_name] = srv_info.request
                        hasher.type_lookup[response_type_name] = srv_info.response
                        
                        # Compute service-level hash
                        srv_info.type_hash = hasher.calculate_service_hash(
                            pkg_name, srv_name, srv_info.request, srv_info.response
                        )
                        # Also compute Request/Response hashes if not done (use 'srv' namespace)
                        if not srv_info.request.type_hash:
                            srv_info.request.type_hash = hasher.calculate_hash(pkg_name, f"{srv_name}_Request", namespace='srv')
                        if not srv_info.response.type_hash:
                            srv_info.response.type_hash = hasher.calculate_hash(pkg_name, f"{srv_name}_Response", namespace='srv')
                    except Exception as e:
                        print(f"  ⚠️  Error computing hash for {pkg_name}/{srv_name}: {e}")
                        import hashlib
                        hash_input = f"{pkg_name}::srv::{srv_name}"
                        hash_hex = hashlib.sha256(hash_input.encode()).hexdigest()
                        srv_info.type_hash = f"RIHS01_{hash_hex}"
        
        # Compute action hashes
        for pkg_name, actions in self.actions_by_package.items():
            for action_name, action_info in actions.items():
                if not action_info.type_hash:
                    try:
                        # Temporarily add action types to hasher's type_lookup
                        # so calculate_hash can find them
                        goal_type_name = f"{pkg_name}/action/{action_name}_Goal"
                        result_type_name = f"{pkg_name}/action/{action_name}_Result"
                        feedback_type_name = f"{pkg_name}/action/{action_name}_Feedback"
                        feedback_msg_type_name = f"{pkg_name}/action/{action_name}_FeedbackMessage"
                        sendgoal_req_type_name = f"{pkg_name}/action/{action_name}_SendGoal_Request"
                        sendgoal_resp_type_name = f"{pkg_name}/action/{action_name}_SendGoal_Response"
                        getresult_req_type_name = f"{pkg_name}/action/{action_name}_GetResult_Request"
                        getresult_resp_type_name = f"{pkg_name}/action/{action_name}_GetResult_Response"
                        
                        hasher.type_lookup[goal_type_name] = action_info.goal
                        hasher.type_lookup[result_type_name] = action_info.result
                        hasher.type_lookup[feedback_type_name] = action_info.feedback
                        hasher.type_lookup[feedback_msg_type_name] = action_info.feedback_message
                        hasher.type_lookup[sendgoal_req_type_name] = action_info.send_goal_service.request
                        hasher.type_lookup[sendgoal_resp_type_name] = action_info.send_goal_service.response
                        hasher.type_lookup[getresult_req_type_name] = action_info.get_result_service.request
                        hasher.type_lookup[getresult_resp_type_name] = action_info.get_result_service.response
                        
                        # Compute hashes for Goal, Result, Feedback (with namespace='action')
                        if not action_info.goal.type_hash:
                            action_info.goal.type_hash = hasher.calculate_hash(
                                pkg_name, f"{action_name}_Goal", namespace='action'
                            )
                        if not action_info.result.type_hash:
                            action_info.result.type_hash = hasher.calculate_hash(
                                pkg_name, f"{action_name}_Result", namespace='action'
                            )
                        if not action_info.feedback.type_hash:
                            action_info.feedback.type_hash = hasher.calculate_hash(
                                pkg_name, f"{action_name}_Feedback", namespace='action'
                            )
                        
                        # Compute hash for FeedbackMessage
                        if action_info.feedback_message and not action_info.feedback_message.type_hash:
                            action_info.feedback_message.type_hash = hasher.calculate_hash(
                                pkg_name, f"{action_name}_FeedbackMessage", namespace='action'
                            )
                        
                        # Compute hashes for SendGoal service (Request/Response/Service)
                        if action_info.send_goal_service:
                            if not action_info.send_goal_service.request.type_hash:
                                action_info.send_goal_service.request.type_hash = hasher.calculate_hash(
                                    pkg_name, f"{action_name}_SendGoal_Request", namespace='action'
                                )
                            if not action_info.send_goal_service.response.type_hash:
                                action_info.send_goal_service.response.type_hash = hasher.calculate_hash(
                                    pkg_name, f"{action_name}_SendGoal_Response", namespace='action'
                                )
                            if not action_info.send_goal_service.type_hash:
                                action_info.send_goal_service.type_hash = hasher.calculate_service_hash(
                                    pkg_name, f"{action_name}_SendGoal",
                                    action_info.send_goal_service.request,
                                    action_info.send_goal_service.response,
                                    namespace='action'
                                )
                        
                        # Compute hashes for GetResult service (Request/Response/Service)
                        if action_info.get_result_service:
                            if not action_info.get_result_service.request.type_hash:
                                action_info.get_result_service.request.type_hash = hasher.calculate_hash(
                                    pkg_name, f"{action_name}_GetResult_Request", namespace='action'
                                )
                            if not action_info.get_result_service.response.type_hash:
                                action_info.get_result_service.response.type_hash = hasher.calculate_hash(
                                    pkg_name, f"{action_name}_GetResult_Response", namespace='action'
                                )
                            if not action_info.get_result_service.type_hash:
                                action_info.get_result_service.type_hash = hasher.calculate_service_hash(
                                    pkg_name, f"{action_name}_GetResult",
                                    action_info.get_result_service.request,
                                    action_info.get_result_service.response,
                                    namespace='action'
                                )
                        
                        # Compute action-level hash (from Goal + Result + Feedback)
                        action_info.type_hash = hasher.calculate_action_hash(
                            pkg_name, action_name,
                            action_info.goal,
                            action_info.result,
                            action_info.feedback
                        )
                        
                    except Exception as e:
                        print(f"  ⚠️  Error computing hash for {pkg_name}/{action_name}: {e}")
                        import hashlib
                        hash_input = f"{pkg_name}::action::{action_name}"
                        hash_hex = hashlib.sha256(hash_input.encode()).hexdigest()
                        action_info.type_hash = f"RIHS01_{hash_hex}"
    
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
        
        srv_count = sum(len(srvs) for srvs in self.services_by_package.values())
        print(f"📦 Discovered {len(messages_by_package)} packages with messages")
        if srv_count > 0:
            print(f"🔧 Discovered {srv_count} services across {len(self.services_by_package)} packages")
        
        return messages_by_package
    
    def _warn_about_empty_struct_arrays(self, messages_by_package: Dict[str, Dict[str, MessageInfo]]) -> List[tuple]:
        """
        Detect and warn about messages with fixed-size arrays of empty structs.
        This is a known pycdr2 limitation that prevents proper serialization.
        
        Returns:
            List of (package_name, message_name) tuples to skip during generation
        """
        # First, identify empty structs (messages with no fields)
        empty_structs = set()
        for pkg_name, messages in messages_by_package.items():
            for msg_name, msg_info in messages.items():
                if len(msg_info.fields) == 0:
                    empty_structs.add(f"{pkg_name}/{msg_name}")
        
        # Now check for messages with fixed-size arrays of these empty structs
        problematic_messages = []
        messages_to_skip = set()
        
        for pkg_name, messages in messages_by_package.items():
            for msg_name, msg_info in messages.items():
                has_problematic_field = False
                for field in msg_info.fields:
                    # Check if this is a fixed-size array of an empty struct
                    if (field.is_fixed_size_array and 
                        not field.is_builtin and 
                        f"{field.ros2_package}/{field.type}" in empty_structs):
                        problematic_messages.append({
                            'message': f"{pkg_name}/{msg_name}",
                            'field': field.name,
                            'field_type': f"{field.type}[{field.array_size}]",
                            'empty_struct': f"{field.ros2_package}/{field.type}",
                            'reason': 'direct'
                        })
                        has_problematic_field = True
                
                # Mark this message for skipping
                if has_problematic_field:
                    messages_to_skip.add((pkg_name, msg_name))
        
        # Transitively skip messages that depend on skipped messages
        skipped_types = {f"{pkg}/{msg}" for pkg, msg in messages_to_skip}
        changed = True
        while changed:
            changed = False
            for pkg_name, messages in messages_by_package.items():
                for msg_name, msg_info in messages.items():
                    if (pkg_name, msg_name) in messages_to_skip:
                        continue
                    
                    for field in msg_info.fields:
                        if not field.is_builtin:
                            field_type = f"{field.ros2_package}/{field.type}"
                            if field_type in skipped_types:
                                problematic_messages.append({
                                    'message': f"{pkg_name}/{msg_name}",
                                    'field': field.name,
                                    'field_type': field.type,
                                    'depends_on': field_type,
                                    'reason': 'dependency'
                                })
                                messages_to_skip.add((pkg_name, msg_name))
                                skipped_types.add(f"{pkg_name}/{msg_name}")
                                changed = True
                                break
        
        # Issue warnings
        if problematic_messages:
            direct = [p for p in problematic_messages if p.get('reason') == 'direct']
            deps = [p for p in problematic_messages if p.get('reason') == 'dependency']
            
            print(f"\n⚠️  Found {len(direct)} message(s) with fixed-size arrays of empty structs:")
            print(f"   This is a known pycdr2 limitation - SKIPPING generation.")
            for prob in direct:
                print(f"   • {prob['message']} (SKIPPED)")
                print(f"     Field '{prob['field']}': {prob['field_type']} (contains empty {prob['empty_struct']})")
            
            if deps:
                print(f"\n⚠️  Also skipping {len(set(p['message'] for p in deps))} message(s) that depend on skipped messages:")
                for prob in deps:
                    print(f"   • {prob['message']} (SKIPPED)")
                    print(f"     Depends on: {prob['depends_on']}")
            
            print(f"\n   Note: Sequences (bounded/unbounded) work fine, only fixed-size arrays are affected.\n")
        
        return list(messages_to_skip)
    
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
            
            # Parse constants (TYPE NAME=value), but NOT bounded sequences ([<=N])
            # Check if '=' appears outside of brackets
            bracket_depth = 0
            has_constant_equals = False
            for char in line:
                if char == '[':
                    bracket_depth += 1
                elif char == ']':
                    bracket_depth -= 1
                elif char == '=' and bracket_depth == 0:
                    has_constant_equals = True
                    break
            
            if has_constant_equals:  # This is a constant declaration
                # Parse constant: TYPE NAME=value
                parts = line.split('=', 1)
                if len(parts) == 2:
                    left_part = parts[0].strip()
                    value = parts[1].strip()
                    
                    # Split type and name
                    type_and_name = left_part.split()
                    if len(type_and_name) == 2:
                        const_type, const_name = type_and_name
                        msg_info.constants[const_name] = (const_type, value)
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
    
    def _parse_srv_file(self, file_path: str, package: str) -> Optional[ServiceInfo]:
        """Parse a .srv file directly from the filesystem."""
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Service files are in PascalCase (e.g., AddTwoInts.srv)
        srv_name = Path(file_path).stem
        
        # Split into request and response sections
        if '---' not in content:
            print(f"⚠️  Warning: No '---' separator found in {file_path}, skipping")
            return None
        
        request_content, response_content = content.split('---', 1)
        
        # Parse request as a message (mark as service type)
        request_msg = MessageInfo(name=f"{srv_name}_Request", package=package, is_service_type=True)
        for line in request_content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Check for constants
            if '=' in line:
                # Parse constant: TYPE NAME=value
                parts = line.split('=', 1)
                if len(parts) == 2:
                    left_part = parts[0].strip()
                    value = parts[1].strip()
                    
                    # Split type and name
                    type_and_name = left_part.split()
                    if len(type_and_name) == 2:
                        const_type, const_name = type_and_name
                        request_msg.constants[const_name] = (const_type, value)
                continue
            
            field = self._parse_field_line(line, package)
            if field:
                request_msg.fields.append(field)
                if not field.is_builtin:
                    request_msg.dependencies.add(f"{field.ros2_package}.{field.type}")
        
        # Parse response as a message (mark as service type)
        response_msg = MessageInfo(name=f"{srv_name}_Response", package=package, is_service_type=True)
        for line in response_content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Check for constants
            if '=' in line:
                # Parse constant: TYPE NAME=value
                parts = line.split('=', 1)
                if len(parts) == 2:
                    left_part = parts[0].strip()
                    value = parts[1].strip()
                    
                    # Split type and name
                    type_and_name = left_part.split()
                    if len(type_and_name) == 2:
                        const_type, const_name = type_and_name
                        response_msg.constants[const_name] = (const_type, value)
                continue
            
            field = self._parse_field_line(line, package)
            if field:
                response_msg.fields.append(field)
                if not field.is_builtin:
                    response_msg.dependencies.add(f"{field.ros2_package}.{field.type}")
        
        srv_info = ServiceInfo(
            name=srv_name,
            package=package,
            request=request_msg,
            response=response_msg
        )
        
        # Type hash will be computed later in parallel
        srv_info.type_hash = None
        request_msg.type_hash = None
        response_msg.type_hash = None
        
        return srv_info
    
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
    
    def _parse_action_file(self, file_path: str, package: str) -> Optional[ActionInfo]:
        """Parse a .action file directly from the filesystem."""
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Action files are in PascalCase (e.g., Fibonacci.action)
        action_name = Path(file_path).stem
        
        # Split into goal, result, and feedback sections
        sections = content.split('---')
        if len(sections) != 3:
            print(f"⚠️  Warning: Action file {file_path} should have exactly 2 '---' separators (goal---result---feedback), skipping")
            return None
        
        goal_content, result_content, feedback_content = sections
        
        # Parse goal as a message
        goal_msg = MessageInfo(name=f"{action_name}_Goal", package=package)
        for line in goal_content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if '=' in line:  # Constants
                continue
            
            field = self._parse_field_line(line, package)
            if field:
                goal_msg.fields.append(field)
                if not field.is_builtin:
                    goal_msg.dependencies.add(f"{field.ros2_package}.{field.type}")
        
        # Parse result as a message
        result_msg = MessageInfo(name=f"{action_name}_Result", package=package)
        for line in result_content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if '=' in line:  # Constants
                continue
            
            field = self._parse_field_line(line, package)
            if field:
                result_msg.fields.append(field)
                if not field.is_builtin:
                    result_msg.dependencies.add(f"{field.ros2_package}.{field.type}")
        
        # Parse feedback as a message
        feedback_msg = MessageInfo(name=f"{action_name}_Feedback", package=package)
        for line in feedback_content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if '=' in line:  # Constants
                continue
            
            field = self._parse_field_line(line, package)
            if field:
                feedback_msg.fields.append(field)
                if not field.is_builtin:
                    feedback_msg.dependencies.add(f"{field.ros2_package}.{field.type}")
        
        # Create FeedbackMessage (wraps Feedback with goal_id)
        # This matches ROS2's generated structure
        feedback_message = MessageInfo(name=f"{action_name}_FeedbackMessage", package=package)
        # Add goal_id field (unique_identifier_msgs/UUID)
        feedback_message.fields.append(FieldInfo(
            name="goal_id",
            type="UUID",
            ros2_package="unique_identifier_msgs",
            is_builtin=False
        ))
        # Add feedback field
        feedback_message.fields.append(FieldInfo(
            name="feedback",
            type=f"{action_name}_Feedback",
            ros2_package=package,
            is_builtin=False,
            namespace="action"
        ))
        feedback_message.dependencies.add(f"unique_identifier_msgs.UUID")
        feedback_message.dependencies.add(f"{package}.{action_name}_Feedback")
        
        # Create SendGoal service (Goal -> goal_id + timestamp + accepted)
        send_goal_request = MessageInfo(name=f"{action_name}_SendGoal_Request", package=package, is_service_type=True)
        send_goal_request.fields.append(FieldInfo(
            name="goal_id",
            type="UUID",
            ros2_package="unique_identifier_msgs",
            is_builtin=False
        ))
        send_goal_request.fields.append(FieldInfo(
            name="goal",
            type=f"{action_name}_Goal",
            ros2_package=package,
            is_builtin=False,
            namespace="action"
        ))
        send_goal_request.dependencies.add(f"unique_identifier_msgs.UUID")
        send_goal_request.dependencies.add(f"{package}.{action_name}_Goal")
        
        send_goal_response = MessageInfo(name=f"{action_name}_SendGoal_Response", package=package, is_service_type=True)
        send_goal_response.fields.append(FieldInfo(
            name="accepted",
            type="bool",
            is_builtin=True
        ))
        send_goal_response.fields.append(FieldInfo(
            name="stamp",
            type="Time",
            ros2_package="builtin_interfaces",
            is_builtin=False
        ))
        send_goal_response.dependencies.add(f"builtin_interfaces.Time")
        
        send_goal_service = ServiceInfo(
            name=f"{action_name}_SendGoal",
            package=package,
            request=send_goal_request,
            response=send_goal_response
        )
        
        # Create GetResult service (goal_id -> result + status)
        get_result_request = MessageInfo(name=f"{action_name}_GetResult_Request", package=package, is_service_type=True)
        get_result_request.fields.append(FieldInfo(
            name="goal_id",
            type="UUID",
            ros2_package="unique_identifier_msgs",
            is_builtin=False
        ))
        get_result_request.dependencies.add(f"unique_identifier_msgs.UUID")
        
        get_result_response = MessageInfo(name=f"{action_name}_GetResult_Response", package=package, is_service_type=True)
        get_result_response.fields.append(FieldInfo(
            name="status",
            type="int8",
            is_builtin=True
        ))
        get_result_response.fields.append(FieldInfo(
            name="result",
            type=f"{action_name}_Result",
            ros2_package=package,
            is_builtin=False,
            namespace="action"
        ))
        get_result_response.dependencies.add(f"{package}.{action_name}_Result")
        
        get_result_service = ServiceInfo(
            name=f"{action_name}_GetResult",
            package=package,
            request=get_result_request,
            response=get_result_response
        )
        
        action_info = ActionInfo(
            name=action_name,
            package=package,
            goal=goal_msg,
            result=result_msg,
            feedback=feedback_msg,
            send_goal_service=send_goal_service,
            get_result_service=get_result_service,
            feedback_message=feedback_message
        )
        
        # Type hashes will be computed later in parallel
        action_info.type_hash = None
        goal_msg.type_hash = None
        result_msg.type_hash = None
        feedback_msg.type_hash = None
        feedback_message.type_hash = None
        
        return action_info
    
    def _parse_field_line(self, line: str, current_package: str) -> Optional[FieldInfo]:
        """Parse a single field definition line."""
        import re
        
        # Remove comments (everything after #)
        if '#' in line:
            line = line.split('#')[0].strip()
        
        parts = line.split()
        if len(parts) < 2:
            return None
        
        field_type = parts[0]
        field_name = parts[1]
        
        # Parse default value if present (parts[2:])
        default_value = None
        if len(parts) > 2:
            # Join remaining parts and extract default value
            # Format: [value1, value2, ...] or just value
            default_str = ' '.join(parts[2:])
            if default_str.startswith('[') and default_str.endswith(']'):
                # Array default value
                default_value = default_str
            elif not default_str.startswith('#'):
                # Scalar default value (not a comment)
                default_value = default_str
        
        is_array = False
        array_size = None
        is_bounded_array = False
        is_fixed_size_array = False
        
        if '[' in field_type and ']' in field_type:
            is_array = True
            # Try fixed-size array first: [N]
            fixed_match = re.search(r'\[(\d+)\]', field_type)
            if fixed_match:
                array_size = int(fixed_match.group(1))
                is_bounded_array = True
                is_fixed_size_array = True
            else:
                # Try bounded sequence: [<=N]
                bounded_match = re.search(r'\[<=(\d+)\]', field_type)
                if bounded_match:
                    array_size = int(bounded_match.group(1))
                    is_bounded_array = True
                    is_fixed_size_array = False
                # else: unbounded [] - is_bounded_array stays False
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
            is_fixed_size_array=is_fixed_size_array,
            is_builtin=is_builtin,
            default_value=default_value
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

