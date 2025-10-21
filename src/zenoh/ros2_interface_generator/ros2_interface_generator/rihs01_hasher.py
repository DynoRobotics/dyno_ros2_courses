"""
RIHS01 Type Hash Calculator

Implements the ROS Interface Hashing Standard (RIHS) version 01,
compatible with rosidl_generator_type_description.

Reference: REP-2011 https://ros.org/reps/rep-2011.html
Implementation: https://github.com/ros2/rosidl/blob/rolling/rosidl_generator_type_description/
"""

import hashlib
import json
from typing import Dict, List, Any
from copy import deepcopy


# Type ID mappings from rosidl_generator_type_description
FIELD_TYPE_IDS = {
    # Basic types
    'bool': 15,
    'byte': 16,
    'char': 13,
    'float32': 10,
    'float64': 11,
    'int8': 2,
    'uint8': 3,
    'int16': 4,
    'uint16': 5,
    'int32': 6,
    'uint32': 7,
    'int64': 8,
    'uint64': 9,
    'string': 17,
    'wstring': 18,
    # Nested type (from another message)
    '__nested__': 1,
}


class RIHS01Hasher:
    """Calculate RIHS01 type hashes for ROS2 messages."""
    
    def __init__(self, all_messages: Dict[str, Dict]):
        """
        Initialize hasher with all parsed messages.
        
        Args:
            all_messages: Dictionary mapping 'package/msg/Name' to MessageInfo objects
        """
        self.all_messages = all_messages
        self.type_cache = {}
    
    def calculate_hash(self, package: str, name: str) -> str:
        """
        Calculate RIHS01 hash for a message type.
        
        Args:
            package: Package name (e.g., 'geometry_msgs')
            name: Message name (e.g., 'Twist')
            
        Returns:
            RIHS01 hash string (e.g., 'RIHS01_9c45bf16...')
        """
        type_name = f"{package}/msg/{name}"
        
        # Build full type description with references
        full_description = self._build_full_type_description(package, name)
        
        # Calculate hash
        return self._calculate_type_hash(full_description)
    
    def _build_full_type_description(self, package: str, name: str) -> Dict:
        """Build complete type description including all referenced types."""
        type_name = f"{package}/msg/{name}"
        
        if package not in self.all_messages or name not in self.all_messages[package]:
            raise ValueError(f"Message {type_name} not found in parsed messages")
        
        msg_info = self.all_messages[package][name]
        
        # Serialize the main type
        main_type = self._serialize_type(msg_info, type_name)
        
        # Collect all referenced types recursively
        referenced_types = {}
        self._collect_references(msg_info, referenced_types)
        
        return {
            'type_description': main_type,
            'referenced_type_descriptions': list(referenced_types.values())
        }
    
    def _serialize_type(self, msg_info, type_name: str) -> Dict:
        """Serialize a message type to the RIHS01 format."""
        fields = []
        
        for field in msg_info.fields:
            field_dict = {
                'name': field.name,
                'type': self._serialize_field_type(field),
                'default_value': ''  # We don't parse default values from .msg files
            }
            fields.append(field_dict)
        
        return {
            'type_name': type_name,
            'fields': fields
        }
    
    def _serialize_field_type(self, field) -> Dict:
        """Serialize a field type."""
        # Determine type_id
        if field.is_builtin:
            type_id = FIELD_TYPE_IDS.get(field.type, 0)
            nested_type_name = ''
        else:
            type_id = FIELD_TYPE_IDS['__nested__']
            # Format: package/msg/Type
            nested_type_name = f"{field.ros2_package}/msg/{field.type}"
        
        # Handle arrays
        if field.is_array:
            if type_id != FIELD_TYPE_IDS['__nested__']:
                type_id += 48  # Array offset (e.g., int32=6 -> int32_array=54)
            else:
                type_id = 49  # NESTED_TYPE_ARRAY
        
        return {
            'type_id': type_id,
            'capacity': field.array_size if field.is_bounded_array else 0,
            'string_capacity': 0,  # We don't handle bounded strings yet
            'nested_type_name': nested_type_name
        }
    
    def _collect_references(self, msg_info, referenced_types: Dict):
        """Recursively collect all referenced message types."""
        for field in msg_info.fields:
            if not field.is_builtin:
                ref_type_name = f"{field.ros2_package}/msg/{field.type}"
                
                if ref_type_name not in referenced_types:
                    # Get the referenced message
                    if field.ros2_package in self.all_messages:
                        if field.type in self.all_messages[field.ros2_package]:
                            ref_msg = self.all_messages[field.ros2_package][field.type]
                            referenced_types[ref_type_name] = self._serialize_type(ref_msg, ref_type_name)
                            
                            # Recursively collect its references
                            self._collect_references(ref_msg, referenced_types)
    
    def _calculate_type_hash(self, full_description: Dict) -> str:
        """
        Calculate RIHS01 hash from full type description.
        
        Implementation matches rosidl_generator_type_description exactly.
        """
        # Create a copy and remove all default values
        hashable_dict = deepcopy(full_description)
        
        for field in hashable_dict['type_description']['fields']:
            del field['default_value']
        
        for referenced_td in hashable_dict['referenced_type_descriptions']:
            for field in referenced_td['fields']:
                del field['default_value']
        
        # JSON dump with exact formatting (matches libyaml in C)
        hashable_repr = json.dumps(
            hashable_dict,
            skipkeys=False,
            ensure_ascii=True,
            check_circular=True,
            allow_nan=False,
            indent=None,
            separators=(', ', ': '),  # Exact format required
            sort_keys=False
        )
        
        # SHA256 hash
        sha = hashlib.sha256()
        sha.update(hashable_repr.encode('utf-8'))
        
        return 'RIHS01_' + sha.hexdigest()

