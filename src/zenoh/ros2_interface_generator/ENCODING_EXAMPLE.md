# Adding a New Encoding - Quick Example

This guide shows exactly how to add a new encoding in 3 simple steps.

## Step 1: Create Encoding Backend

Create `ros2_interface_generator/encodings/my_encoding.py`:

\`\`\`python
from .base import EncodingBackend

class MyEncoding(EncodingBackend):
    def get_name(self) -> str:
        return "my_encoding"
    
    def get_python_dependencies(self) -> list:
        return ['my_serializer>=1.0.0']
    
    def get_python_imports(self) -> list:
        return ['import my_serializer']
    
    def get_python_serialize_method(self, msg_info) -> str:
        return """    def serialize(self) -> bytes:
        return my_serializer.encode(self.to_dict())"""
    
    def get_python_deserialize_method(self, msg_info) -> str:
        return """    @classmethod
    def deserialize(cls, data: bytes):
        return cls.from_dict(my_serializer.decode(data))"""
    
    def supports_type_hash(self) -> bool:
        return False  # True if you need ROS2 type hashes
    
    def supports_dds_type_name(self) -> bool:
        return False  # True if you need DDS interop
\`\`\`

## Step 2: Register It

Edit `ros2_interface_generator/encodings/__init__.py`:

\`\`\`python
def get_encoding(encoding_name: str):
    # ... existing code ...
    elif encoding_name == 'my_encoding':
        from .my_encoding import MyEncoding
        return MyEncoding()
\`\`\`

## Step 3: Use It!

\`\`\`bash
ros2-generate-interfaces -l python -e my_encoding -p essential -o my_interfaces
\`\`\`

That's it! The template system automatically adapts to your encoding.

## Testing

\`\`\`python
from ros2_interface_generator.encodings import get_encoding

enc = get_encoding('my_encoding')
print(f"Name: {enc.get_name()}")
print(f"Dependencies: {enc.get_python_dependencies()}")
print(f"Supports type hash: {enc.supports_type_hash()}")
\`\`\`
