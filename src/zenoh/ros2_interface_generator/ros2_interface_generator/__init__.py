"""
ROS2 Interface Generator

A universal tool for generating ROS2 interface code (messages, services, actions)
for multiple target languages with multiple encoding backends.

Supported Languages:
- Python (pycdr2 for CDR serialization)
- Rust (future)
- TypeScript/JavaScript (future)
- C/C++ (future)

Supported Encodings:
- CDR (Common Data Representation - DDS standard)
- JSON (future)
- MessagePack (future)
- Protobuf (future)

Example usage:
    from ros2_interface_generator import Generator
    
    gen = Generator(language='python', encoding='cdr')
    gen.generate(input_path='/', output_path='ros2_interfaces_py')
"""

__version__ = "0.1.0"
__all__ = ["Generator", "generate"]

from .generator import Generator, generate


