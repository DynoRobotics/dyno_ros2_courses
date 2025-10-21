# Encoding Backend Architecture

## Overview

The `ros2_interface_generator` separates **language generation** from **encoding/serialization**. This allows:
- Adding new languages without duplicating encoding logic
- Adding new encodings without modifying language generators
- Mix-and-match: Python+CDR, Python+JSON, Rust+CDR, TypeScript+JSON, etc.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Generator Core                        │
│  - Message discovery                                     │
│  - Type hash computation                                 │
│  - Dependency resolution                                 │
└────────────────┬────────────────────────────────────────┘
                 │
      ┌──────────┴──────────┐
      │                     │
┌─────▼──────┐      ┌───────▼────────┐
│  Language  │      │    Encoding    │
│  Backends  │      │    Backends    │
├────────────┤      ├────────────────┤
│ - Python   │      │ - CDR          │
│ - Rust     │      │ - JSON         │
│ - TypeScript│     │ - MessagePack  │
│ - C/C++    │      │ - Protobuf     │
└────────────┘      └────────────────┘
      │                     │
      └──────────┬──────────┘
                 │
         ┌───────▼────────┐
         │  Generated     │
         │  Code          │
         └────────────────┘
```

## Components

### 1. Encoding Base Class (`encodings/base.py`)

All encoding backends inherit from `EncodingBackend`:

```python
class EncodingBackend(ABC):
    @abstractmethod
    def get_name(self) -> str:
        """Encoding name (cdr, json, msgpack)"""
    
    @abstractmethod
    def get_python_dependencies(self) -> list:
        """Python packages needed (e.g., ['pycdr2>=0.3.0'])"""
    
    @abstractmethod
    def get_python_serialize_method(self, msg_info) -> str:
        """Generate Python serialize() method code"""
    
    @abstractmethod
    def get_python_deserialize_method(self, msg_info) -> str:
        """Generate Python deserialize() method code"""
    
    @abstractmethod
    def supports_type_hash(self) -> bool:
        """Whether ROS2 type hashes are needed"""
    
    @abstractmethod
    def supports_dds_type_name(self) -> bool:
        """Whether DDS type names are needed"""
```

### 2. Implemented Encodings

#### CDR (`encodings/cdr.py`) ✅ Fully Implemented
- **Purpose**: ROS2/DDS interoperability
- **Library**: `pycdr2`
- **Type Hashes**: Required (RIHS01)
- **DDS Names**: Required
- **Use Cases**: Native ROS2 communication, Zenoh-DDS bridge

```python
class CDREncoding(EncodingBackend):
    def get_python_dependencies(self) -> list:
        return ['pycdr2>=0.3.0']
    
    def supports_type_hash(self) -> bool:
        return True  # Needed for DDS compatibility
```

#### JSON (`encodings/json_encoding.py`) ✅ Fully Implemented
- **Purpose**: Human-readable, web APIs, debugging
- **Library**: Python `json` (stdlib)
- **Type Hashes**: Not required
- **DDS Names**: Not required
- **Use Cases**: REST APIs, debugging, language interop

```python
class JSONEncoding(EncodingBackend):
    def get_python_dependencies(self) -> list:
        return []  # json is in stdlib
    
    def supports_type_hash(self) -> bool:
        return False  # Not for DDS
```

#### MessagePack (`encodings/msgpack_encoding.py`) 🚧 Stub/Example
- **Purpose**: Compact binary format
- **Library**: `msgpack`
- **Status**: Example stub showing how to add encodings
- **Use Cases**: Efficient wire protocol, microservices

### 3. Language Backends (`languages/`)

Language backends use encoding backends to generate serialization code:

```python
class PythonGenerator:
    def __init__(self, encoding: str = 'cdr'):
        # Get encoding backend
        from ..encodings import get_encoding
        self.encoding_backend = get_encoding(encoding)
    
    def _generate_message_file(self, msg_dir, message, ros2_pkg):
        # Use encoding backend
        needs_type_hash = self.encoding_backend.supports_type_hash()
        needs_dds_name = self.encoding_backend.supports_dds_type_name()
        
        # Render template with encoding-specific data
        template.render(
            message=message,
            encoding_name=self.encoding_name,
            needs_type_hash=needs_type_hash,
            needs_dds_type_name=needs_dds_name,
        )
```

### 4. Universal Template (`templates/python/message_universal.py.jinja2`)

The template adapts to the encoding:

```jinja2
{% if encoding_name == 'cdr' %}
from pycdr2 import IdlStruct

@dataclass
class {{ message.name }}(IdlStruct, typename="{{ message.package }}/{{ message.name }}"):
{% elif encoding_name == 'json' %}
import json

@dataclass
class {{ message.name }}:
{% endif %}
    """{{ message.package }}/{{ message.name }} message."""
{% if needs_type_hash %}
    TYPE_HASH = "{{ message.type_hash }}"
{% endif %}
{% if needs_dds_type_name %}
    DDS_TYPE_NAME = "{{ message.package }}::msg::dds_::{{ message.name }}_"
{% endif %}

{% if encoding_name == 'cdr' %}
    def serialize(self) -> bytes:
        return IdlStruct.serialize(self)
{% elif encoding_name == 'json' %}
    def serialize(self) -> bytes:
        return json.dumps(self.to_dict()).encode('utf-8')
{% endif %}
```

## Adding a New Encoding

### Example: Add Protobuf Support

1. **Create encoding backend** (`encodings/protobuf_encoding.py`):

```python
from .base import EncodingBackend

class ProtobufEncoding(EncodingBackend):
    def get_name(self) -> str:
        return "protobuf"
    
    def get_python_dependencies(self) -> list:
        return ['protobuf>=4.0.0']
    
    def get_python_imports(self) -> list:
        return ['from google.protobuf import message']
    
    def get_python_serialize_method(self, msg_info) -> str:
        return """    def serialize(self) -> bytes:
        return self._to_proto().SerializeToString()"""
    
    def get_python_deserialize_method(self, msg_info) -> str:
        return """    @classmethod
    def deserialize(cls, data: bytes):
        proto = ProtoMessage()
        proto.ParseFromString(data)
        return cls._from_proto(proto)"""
    
    def supports_type_hash(self) -> bool:
        return False
    
    def supports_dds_type_name(self) -> bool:
        return False
```

2. **Register in `encodings/__init__.py`**:

```python
def get_encoding(encoding_name: str):
    if encoding_name == 'protobuf':
        from .protobuf_encoding import ProtobufEncoding
        return ProtobufEncoding()
```

3. **Update template** (if needed) in `message_universal.py.jinja2`:

```jinja2
{% elif encoding_name == 'protobuf' %}
from google.protobuf import message

@dataclass
class {{ message.name }}(message.Message):
    # ... protobuf-specific code
{% endif %}
```

4. **Use it**:

```bash
ros2-generate-interfaces -l python -e protobuf -p essential -o my_interfaces
```

## Encoding Comparison

| Encoding    | Binary | Compact | Human-Readable | ROS2 Compat | Type Hash | DDS Interop |
|-------------|--------|---------|----------------|-------------|-----------|-------------|
| CDR         | ✅     | ✅      | ❌             | ✅          | ✅        | ✅          |
| JSON        | ❌     | ❌      | ✅             | ❌          | ❌        | ❌          |
| MessagePack | ✅     | ✅      | ❌             | ❌          | ❌        | ❌          |
| Protobuf    | ✅     | ✅      | ❌             | ❌          | ❌        | ❌          |

## Usage Examples

### CDR for ROS2 Interop

```bash
ros2-generate-interfaces -l python -e cdr -p essential -o ros2_interfaces_py
```

```python
from ros2_interfaces_py.geometry_msgs.msg.twist import Twist
twist = Twist(...)
cdr_bytes = twist.serialize()  # CDR format for DDS/Zenoh
```

### JSON for Web APIs

```bash
ros2-generate-interfaces -l python -e json -p essential -o ros2_interfaces_json
```

```python
from ros2_interfaces_json.geometry_msgs.msg.twist import Twist
twist = Twist(...)
json_bytes = twist.serialize()  # JSON format for REST APIs
```

### MessagePack for Microservices

```bash
# ros2-generate-interfaces -l python -e msgpack -p essential -o ros2_interfaces_msgpack
# (Not yet fully implemented - see msgpack_encoding.py stub)
```

## Benefits

1. **Modularity**: Encodings and languages are independent
2. **Extensibility**: Add new encodings without touching language code
3. **Flexibility**: Generate same messages with different encodings
4. **Maintainability**: Single source of truth for each encoding
5. **Testing**: Test encodings independently

## Future Encodings

Potential encodings to add:
- **Protobuf**: Google's efficient binary format
- **MessagePack**: Compact binary JSON-like format
- **Apache Avro**: Schema-based serialization
- **Cap'n Proto**: Zero-copy serialization
- **FlatBuffers**: Memory-efficient serialization

Each can be added by creating a new encoding backend class!

