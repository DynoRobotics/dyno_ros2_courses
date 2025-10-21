# ROS2 Interface Generator

**Universal code generator for ROS2 interfaces supporting multiple languages and encodings.**

Generate type-safe, serialization-ready code from ROS2 interface definitions (`.msg`, `.srv`, `.action` files) for Python, Rust, TypeScript, C/C++, and more.

## Features

✅ **Multi-Language Support**
- Python (pycdr2 for CDR serialization)
- Rust (future)
- TypeScript/JavaScript (future)
- C/C++ (future)

✅ **Multi-Encoding Support** (Pluggable Architecture)
- CDR (Common Data Representation - DDS standard) ✅ Implemented
- JSON (human-readable, web APIs) ✅ Implemented
- MessagePack (compact binary) 🚧 Stub/Example
- Protobuf (future - easy to add!)
- [See ENCODING_ARCHITECTURE.md for details](ENCODING_ARCHITECTURE.md)

✅ **ROS2 Compatible**
- RIHS01 type hashes for interoperability
- DDS-compatible CDR serialization
- Works with rmw_zenoh_cpp and standard ROS2 tools

✅ **Clean, Modular Architecture**
- **Separate language and encoding backends** - mix and match!
- **Easy to extend**: Add new encodings in ~100 lines of code
- **Template-based generation** with Jinja2
- [Adding new encodings is trivial](ENCODING_EXAMPLE.md)

## Installation

```bash
# Install the generator
cd ros2_interface_generator
pip install -e .

# For Python interface generation (adds pycdr2 dependency)
pip install -e ".[python]"
```

## Quick Start

### Generate Python Interfaces

```bash
# Generate all standard ROS2 messages for Python
ros2-generate-interfaces --lang python --encoding cdr --output ros2_interfaces_py

# Or use short flags
ros2-generate-interfaces -l python -e cdr -o ros2_interfaces_py
```

### Use Generated Interfaces

```python
# After generation, install the package
cd ros2_interfaces_py
pip install -e .

# Use in your code
from ros2_interfaces_py.geometry_msgs.msg.twist import Twist
from ros2_interfaces_py.geometry_msgs.msg.vector3 import Vector3

msg = Twist(
    linear=Vector3(x=1.0, y=0.0, z=0.0),
    angular=Vector3(x=0.0, y=0.0, z=0.5)
)

# Serialize to CDR bytes
data = msg.serialize()

# Deserialize from CDR bytes
msg2 = Twist.deserialize(data)
```

## Package Presets

The generator provides curated package lists to avoid slow auto-discovery:

- **essential** (4 packages): `builtin_interfaces`, `std_msgs`, `geometry_msgs`, `rcl_interfaces`
  - Core types for basic ROS2 communication and logging
  - Fast generation, minimal dependencies

- **common** (10 packages): essential + sensor_msgs, nav_msgs, trajectory_msgs, action_msgs, tf2_msgs, etc.
  - Covers most robotics applications
  - Recommended for typical projects

- **standard** (21 packages): common + diagnostic_msgs, shape_msgs, visualization_msgs, etc.
  - Comprehensive ROS2 standard messages
  - Full ROS2 compatibility

- **all** (32 packages): standard + example_interfaces, gps_msgs, turtlesim, vision_msgs, etc.
  - Complete coverage of available ROS2 packages

## CLI Usage

```bash
ros2-generate-interfaces [OPTIONS]

Options:
  -l, --lang LANGUAGE       Target language (python, rust, typescript, c)
  -e, --encoding ENCODING   Serialization format (cdr, json, msgpack, protobuf)
  -p, --preset PRESET       Package preset (essential, common, standard, all)
  --packages PKG [PKG ...]  Specific packages to generate
  --exclude PKG [PKG ...]   Packages to exclude from preset
  -i, --input PATH          Input ROS2 workspace (default: / for system packages)
  -o, --output PATH         Output directory (required)
  --version                 Show version
  --help                    Show help message
```

### Examples

```bash
# Generate essential packages (recommended for getting started)
ros2-generate-interfaces -l python -e cdr -p essential -o ros2_interfaces_py

# Generate common packages for robotics applications
ros2-generate-interfaces -l python -e cdr -p common -o ros2_interfaces_py

# Generate specific packages only
ros2-generate-interfaces -l python -e cdr --packages geometry_msgs std_msgs sensor_msgs -o my_interfaces

# Generate standard packages but exclude certain ones
ros2-generate-interfaces -l python -e cdr -p standard --exclude turtlesim example_interfaces -o ros2_interfaces_py

# Future: Rust with CDR
ros2-generate-interfaces -l rust -e cdr -p essential -o rust_out

# Future: TypeScript with JSON
ros2-generate-interfaces -l typescript -e json -p common -o ts_out

# Generate from custom workspace
ros2-generate-interfaces -l python -e cdr -i ~/my_ws -o my_interfaces_py
```

## Python API

```python
from ros2_interface_generator import Generator, generate

# Using the convenience function
generate(
    language='python',
    encoding='cdr',
    input_path='/',
    output_path='ros2_interfaces_py'
)

# Or using the Generator class directly
gen = Generator(language='python', encoding='cdr')
gen.generate(input_path='/', output_path='ros2_interfaces_py')
```

## Architecture

```
ros2_interface_generator/
├── ros2_interface_generator/
│   ├── __init__.py              # Public API
│   ├── generator.py             # Core orchestration
│   ├── languages/               # Language backends
│   │   ├── __init__.py
│   │   ├── python.py            # Python code generation
│   │   ├── rust.py              # Rust (future)
│   │   ├── typescript.py        # TypeScript (future)
│   │   └── c.py                 # C/C++ (future)
│   ├── encodings/               # Encoding backends
│   │   ├── __init__.py
│   │   ├── cdr.py               # CDR encoding (future)
│   │   ├── json.py              # JSON (future)
│   │   └── msgpack.py           # MessagePack (future)
│   └── templates/               # Code templates
│       ├── python/
│       ├── rust/
│       └── typescript/
├── bin/
│   └── ros2-generate-interfaces # CLI tool
├── tests/
├── setup.py
└── README.md
```

## How It Works

1. **Discovery**: Queries ROS2 system for available packages and interfaces using `ros2 CLI`
2. **Parsing**: Extracts message definitions, field types, and type hashes
3. **Generation**: Uses language-specific backends to generate code
4. **Output**: Creates installable packages with proper structure

### Type Hash Computation

The generator obtains RIHS01 type hashes by:
1. Querying `ros2 interface show --verbose` for authoritative hash
2. Looking up known hashes for common types
3. Computing a simplified hash as fallback (with warning)

This ensures generated types are compatible with native ROS2 nodes.

## Extending the Generator

### Adding a New Language

1. Create `ros2_interface_generator/languages/your_lang.py`
2. Implement `YourLangGenerator` class with `generate()` method
3. Add to `__init__.py` exports
4. Update CLI choices

### Adding a New Encoding

1. Create `ros2_interface_generator/encodings/your_encoding.py`
2. Implement serialization/deserialization logic
3. Update language backends to support the encoding
4. Add to CLI choices

## Comparison with rosidl_generator_py

| Feature | `rosidl_generator_py` | `ros2_interface_generator` |
|---------|----------------------|----------------------------|
| Purpose | Official ROS2 Python bindings | Lightweight, Zenoh-friendly |
| Dependencies | ROS2 build tools, rclpy | Standalone, minimal deps |
| Serialization | rcutils CDR | pycdr2 (pure Python) |
| Languages | Python only | Python, Rust, TS, C (future) |
| Encodings | CDR only | CDR, JSON, MsgPack (future) |
| Installation | ament/colcon build | pip install |

## Use Cases

### 1. Zenoh-ROS2 Bridge
Generate lightweight Python types for Zenoh <-> ROS2 communication without full rclpy dependency.

### 2. Multi-Language Systems
Generate consistent interfaces for polyglot systems (Python microservices, Rust nodes, TypeScript dashboards).

### 3. Custom Encodings
Use JSON or MessagePack for web/mobile clients instead of binary CDR.

### 4. Rapid Prototyping
Quickly generate and iterate on custom message types without full ROS2 build infrastructure.

## Development

```bash
# Clone the repository
git clone https://github.com/your-org/ros2_interface_generator
cd ros2_interface_generator

# Install in development mode
pip install -e ".[dev]"

# Run tests
pytest

# Format code
black ros2_interface_generator/

# Type check
mypy ros2_interface_generator/
```

## Roadmap

- [x] Core generator architecture
- [x] Python CDR backend (basic)
- [ ] Complete Python CDR serialization (pycdr2 integration)
- [ ] Rust CDR backend
- [ ] TypeScript CDR backend
- [ ] C/C++ backend
- [ ] JSON encoding support
- [ ] MessagePack encoding support
- [ ] Service (.srv) generation
- [ ] Action (.action) generation
- [ ] Template-based generation (Jinja2)
- [ ] Custom message support (local .msg files)

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## License

Apache 2.0

## Related Projects

- **[ros2_zenoh_python](https://github.com/your-org/ros2_zenoh_python)** - ROS2 transport library using Zenoh
- **[pycdr2](https://pypi.org/project/pycdr2/)** - Pure Python CDR serialization
- **[rmw_zenoh_cpp](https://github.com/ros2/rmw_zenoh)** - Official ROS2 Zenoh RMW implementation

## Acknowledgments

- Built for the Zenoh/ROS2 ecosystem
- Inspired by rosidl_generator_py
- Powered by pycdr2 for CDR serialization

