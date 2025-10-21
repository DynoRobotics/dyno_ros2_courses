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
            all_messages: Dictionary mapping package -> {name -> MessageInfo}
        """
        # Convert nested dict to flat dict keyed by full type name
        self.all_messages = all_messages
        self.type_lookup = {}
        for package, messages in all_messages.items():
            for name, msg_info in messages.items():
                # Determine namespace from is_service_type flag
                namespace = 'srv' if msg_info.is_service_type else 'msg'
                full_type_name = f"{package}/{namespace}/{name}"
                self.type_lookup[full_type_name] = msg_info
        self.type_cache = {}
    
    def calculate_hash(self, package: str, name: str, namespace: str = 'msg') -> str:
        """
        Calculate RIHS01 hash for a message type.
        
        Args:
            package: Package name (e.g., 'geometry_msgs')
            name: Message name (e.g., 'Twist')
            namespace: Type namespace ('msg' or 'srv'), default 'msg'
            
        Returns:
            RIHS01 hash string (e.g., 'RIHS01_9c45bf16...')
        """
        type_name = f"{package}/{namespace}/{name}"
        
        # Build full type description with references
        full_description = self._build_full_type_description(package, name, namespace)
        
        # Calculate hash
        return self._calculate_type_hash(full_description)
    
    def calculate_service_hash(self, package: str, name: str, request_msg, response_msg) -> str:
        """
        Calculate RIHS01 hash for a service type.
        
        Services have their own type description that references Request, Response, and Event types.
        Per ROS2 standard, services have 3 members: request_message, response_message, event_message
        
        Args:
            package: Package name (e.g., 'example_interfaces')
            name: Service name (e.g., 'AddTwoInts')
            request_msg: MessageInfo for the Request
            response_msg: MessageInfo for the Response
            
        Returns:
            RIHS01 hash string for the service
        """
        # Build service type description
        # Services reference Request, Response, and Event message types
        service_type = {
            'type_name': f"{package}/srv/{name}",
            'fields': [
                {
                    'name': 'request_message',
                    'type': {
                        'type_id': 1,  # Nested type
                        'capacity': 0,
                        'string_capacity': 0,
                        'nested_type_name': f"{package}/srv/{name}_Request"
                    },
                    'default_value': ''
                },
                {
                    'name': 'response_message',
                    'type': {
                        'type_id': 1,  # Nested type
                        'capacity': 0,
                        'string_capacity': 0,
                        'nested_type_name': f"{package}/srv/{name}_Response"
                    },
                    'default_value': ''
                },
                {
                    'name': 'event_message',
                    'type': {
                        'type_id': 1,  # Nested type
                        'capacity': 0,
                        'string_capacity': 0,
                        'nested_type_name': f"{package}/srv/{name}_Event"
                    },
                    'default_value': ''
                }
            ]
        }
        
        # Collect referenced types (Request, Response, Event, and their dependencies)
        referenced_types = {}
        
        # Add Request type
        request_type_name = f"{package}/srv/{name}_Request"
        referenced_types[request_type_name] = self._serialize_type(request_msg, request_type_name)
        self._collect_references(request_msg, referenced_types)
        
        # Add Response type  
        response_type_name = f"{package}/srv/{name}_Response"
        referenced_types[response_type_name] = self._serialize_type(response_msg, response_type_name)
        self._collect_references(response_msg, referenced_types)
        
        # Add Event type (auto-generated for all services)
        # Event has 3 fields: info (ServiceEventInfo), request (bounded sequence[1]), response (bounded sequence[1])
        event_type_name = f"{package}/srv/{name}_Event"
        referenced_types[event_type_name] = {
            'type_name': event_type_name,
            'fields': [
                {
                    'name': 'info',
                    'type': {
                        'type_id': 1,  # Nested type
                        'capacity': 0,
                        'string_capacity': 0,
                        'nested_type_name': 'service_msgs/msg/ServiceEventInfo'
                    },
                    'default_value': ''
                },
                {
                    'name': 'request',
                    'type': {
                        'type_id': 97,  # FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE
                        'capacity': 1,
                        'string_capacity': 0,
                        'nested_type_name': request_type_name
                    },
                    'default_value': ''
                },
                {
                    'name': 'response',
                    'type': {
                        'type_id': 97,  # FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE
                        'capacity': 1,
                        'string_capacity': 0,
                        'nested_type_name': response_type_name
                    },
                    'default_value': ''
                }
            ]
        }
        
        # Add ServiceEventInfo (standard ROS2 message from service_msgs package)
        # We need to add this as well since Event references it
        # ServiceEventInfo has fixed fields per ROS2 standard
        referenced_types['service_msgs/msg/ServiceEventInfo'] = {
            'type_name': 'service_msgs/msg/ServiceEventInfo',
            'fields': [
                {'name': 'event_type', 'type': {'type_id': 3, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}, 'default_value': ''},  # uint8
                {'name': 'stamp', 'type': {'type_id': 1, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': 'builtin_interfaces/msg/Time'}, 'default_value': ''},
                {'name': 'client_gid', 'type': {'type_id': 51, 'capacity': 16, 'string_capacity': 0, 'nested_type_name': ''}, 'default_value': ''},  # uint8[16]
                {'name': 'sequence_number', 'type': {'type_id': 8, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}, 'default_value': ''},  # int64
            ]
        }
        
        # Add builtin_interfaces/msg/Time if not already present
        if 'builtin_interfaces/msg/Time' not in referenced_types:
            referenced_types['builtin_interfaces/msg/Time'] = {
                'type_name': 'builtin_interfaces/msg/Time',
                'fields': [
                    {'name': 'sec', 'type': {'type_id': 6, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}, 'default_value': ''},  # int32
                    {'name': 'nanosec', 'type': {'type_id': 7, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}, 'default_value': ''},  # uint32
                ]
            }
        
        # Sort referenced types alphabetically by type_name (as ROS2 does)
        sorted_refs = sorted(referenced_types.values(), key=lambda x: x['type_name'])
        
        full_description = {
            'type_description': service_type,
            'referenced_type_descriptions': sorted_refs
        }
        
        return self._calculate_type_hash(full_description)
    
    def _build_full_type_description(self, package: str, name: str, namespace: str = 'msg') -> Dict:
        """Build complete type description including all referenced types."""
        type_name = f"{package}/{namespace}/{name}"
        
        if type_name not in self.type_lookup:
            raise ValueError(f"Message {type_name} not found in parsed messages")
        
        msg_info = self.type_lookup[type_name]
        
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

