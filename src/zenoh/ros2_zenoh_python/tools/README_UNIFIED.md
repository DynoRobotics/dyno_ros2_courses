# ROS 2 CDR Types Generator

Generate unified, CDR-serializable ROS 2 message type packages for multiple languages **without ROS 2 dependencies**.

## Overview

This tool generates a **single package per language** containing all ROS 2 interface types you need:

- **Python**: `ros2-types-cdr`
- **Rust**: `ros2_types_cdr`
- **TypeScript**: `@ros2-cdr/types`
- **C**: `ros2_types_cdr`

### Key Features

✅ **No ROS 2 dependency** - Pure language implementations  
✅ **Unified packages** - One package per language, not dozens  
✅ **CDR serialization ready** - Compatible with ROS 2 message serialization  
✅ **Template-based** - Uses Jinja2 for clean code generation  
✅ **Cross-platform** - Works in embedded systems, web browsers, anywhere  
✅ **Type-safe** - Full type definitions for all languages  

## Installation

```bash
# Optional: Install Jinja2 for better template support
pip install jinja2
```

## Quick Start

### Generate All Types

```bash
python3 generate_unified_types.py \
  -i /opt/ros/jazzy/share/geometry_msgs/msg \
  -i /opt/ros/jazzy/share/std_msgs/msg \
  -i /opt/ros/jazzy/share/sensor_msgs/msg \
  -o output \
  -l python rust typescript c
```

### Output Structure

```
output/
├── python/
│   ├── setup.py
│   └── ros2_types_cdr/
│       ├── __init__.py
│       ├── geometry_msgs/
│       │   ├── __init__.py
│       │   ├── twist.py
│       │   ├── vector3.py
│       │   └── ...
│       ├── std_msgs/
│       └── sensor_msgs/
├── rust/
│   └── ros2_types_cdr/
│       ├── Cargo.toml
│       └── src/
│           ├── lib.rs
│           ├── geometry_msgs/
│           ├── std_msgs/
│           └── sensor_msgs/
├── typescript/
│   └── ros2-cdr-types/
│       ├── package.json
│       ├── tsconfig.json
│       └── src/
│           ├── index.ts
│           ├── geometry_msgs.ts
│           ├── std_msgs.ts
│           └── sensor_msgs.ts
└── c/
    └── ros2_types_cdr/
        ├── CMakeLists.txt
        ├── include/
        │   └── ros2_types_cdr/
        │       ├── ros2_types_cdr.h
        │       ├── geometry_msgs/
        │       ├── std_msgs/
        │       └── sensor_msgs/
        └── src/
            ├── geometry_msgs/
            ├── std_msgs/
            └── sensor_msgs/
```

## Usage Examples

### Python

```bash
# Install the package
cd output/python
pip install -e .
```

```python
from ros2_types_cdr.geometry_msgs import Twist, Vector3

# Create a message
twist = Twist(
    linear=Vector3(x=1.0, y=0.0, z=0.0),
    angular=Vector3(x=0.0, y=0.0, z=0.5)
)

# Convert to dict for JSON
data = twist.to_dict()

# Create from dict
twist2 = Twist.from_dict(data)
```

### Rust

```bash
# Use the crate
cd output/rust/ros2_types_cdr
cargo build
```

```toml
# Add to your Cargo.toml
[dependencies]
ros2_types_cdr = { path = "../path/to/output/rust/ros2_types_cdr" }
```

```rust
use ros2_types_cdr::geometry_msgs::{Twist, Vector3};

fn main() {
    let twist = Twist {
        linear: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
        angular: Vector3 { x: 0.0, y: 0.0, z: 0.5 },
    };
    
    // Serialize to JSON
    let json = twist.to_json().unwrap();
    println!("{}", json);
    
    // Deserialize from JSON
    let twist2 = Twist::from_json(&json).unwrap();
}
```

### TypeScript

```bash
# Build the package
cd output/typescript/ros2-cdr-types
npm install
npm run build
```

```typescript
import { Twist, Vector3, serializeTwist } from '@ros2-cdr/types';

const twist: Twist = {
  linear: { x: 1.0, y: 0.0, z: 0.0 },
  angular: { x: 0.0, y: 0.0, z: 0.5 }
};

// Serialize
const json = serializeTwist(twist);
console.log(json);
```

### C

```bash
# Build the library
cd output/c/ros2_types_cdr
mkdir build && cd build
cmake ..
make
```

```c
#include <ros2_types_cdr/ros2_types_cdr.h>

int main() {
    geometry_msgs_Twist_t twist = {
        .linear = { .x = 1.0, .y = 0.0, .z = 0.0 },
        .angular = { .x = 0.0, .y = 0.0, .z = 0.5 }
    };
    
    #ifdef MICROCDR_AVAILABLE
    uint8_t buffer[1024];
    size_t size;
    geometry_msgs_Twist_serialize(&twist, buffer, sizeof(buffer), &size);
    #endif
    
    return 0;
}
```

## Use Cases

### 1. Tauri Applications

Generate types for your Tauri app to communicate ROS 2 messages between Rust backend and TypeScript frontend:

```bash
python3 generate_unified_types.py \
  -i /opt/ros/jazzy/share/geometry_msgs/msg \
  -o tauri_types \
  -l rust typescript
```

Then use the generated types in your Tauri commands:

```rust
use ros2_types_cdr::geometry_msgs::Twist;

#[tauri::command]
pub async fn publish_twist(twist: Twist) -> Result<String, String> {
    // Your ROS 2 publishing logic
    Ok(twist.to_json().unwrap())
}
```

### 2. Embedded Systems

Generate C types for embedded systems without ROS 2:

```bash
python3 generate_unified_types.py \
  -i /opt/ros/jazzy/share/sensor_msgs/msg \
  -o embedded_types \
  -l c
```

### 3. Web Applications

Generate TypeScript types for browser-based ROS 2 applications:

```bash
python3 generate_unified_types.py \
  -i /opt/ros/jazzy/share/geometry_msgs/msg \
  -o web_types \
  -l typescript
```

### 4. Python Scripts Without ROS 2

Generate Python types for scripts that don't need full ROS 2:

```bash
python3 generate_unified_types.py \
  -i /opt/ros/jazzy/share/std_msgs/msg \
  -o python_types \
  -l python
```

## Command Line Options

```
usage: generate_unified_types.py [-h] -i INPUT [-o OUTPUT] [-l LANGUAGES] [-v]

Generate unified CDR-serializable ROS 2 types packages

optional arguments:
  -h, --help            Show help message
  -i, --input INPUT     Input ROS 2 message directory (can specify multiple times)
  -o, --output OUTPUT   Output directory (default: output)
  -l, --languages       Languages to generate (default: all)
                        Choices: python, rust, typescript, c
  -v, --verbose         Verbose output
```

## Package Naming

To avoid conflicts with official ROS 2 packages, all generated packages use `-cdr` or `_cdr` suffix:

- Python: `ros2-types-cdr` (PyPI-safe name)
- Rust: `ros2_types_cdr` (crates.io-safe name)
- TypeScript: `@ros2-cdr/types` (scoped npm package)
- C: `ros2_types_cdr` (library name)

## Dependencies

### Python Package
- Python 3.8+
- No external dependencies

### Rust Crate
- `serde = "1.0"` with `derive` feature
- `serde_json = "1.0"`

### TypeScript Package
- `typescript = "^5.0.0"` (dev dependency)

### C Library
- C11 compiler
- CMake 3.8+
- Micro-CDR (optional, for CDR serialization)

## Advanced Features

### Custom Message Dependencies

The generator automatically resolves dependencies between messages:

```python
# Header depends on Time from builtin_interfaces
from ros2_types_cdr.std_msgs import Header
from ros2_types_cdr.builtin_interfaces import Time

header = Header(
    stamp=Time(sec=123, nanosec=456789),
    frame_id="base_link"
)
```

### Type Safety

All generated types are fully typed:

```typescript
// TypeScript gets full IntelliSense support
const twist: Twist = {
  linear: { x: 1.0, y: 0.0, z: 0.0 },  // ✅ Type-checked
  angular: { x: 0.0, y: 0.0, z: "invalid" }  // ❌ Compile error
};
```

## Template System

The generator uses Jinja2 templates for clean code generation. Templates are inline but can be extracted to files for customization.

## Contributing

To add support for a new language:

1. Add type mappings to `type_mappings` dict
2. Implement `generate_<language>()` method
3. Add language to CLI choices

## License

Apache-2.0

## Why This Tool?

### Problem

- Official ROS 2 packages require full ROS 2 installation
- Each message package is separate (geometry_msgs, std_msgs, etc.)
- Not suitable for embedded systems, web browsers, or non-ROS environments
- Heavy dependencies for simple message types

### Solution

- **One package per language** - Single dependency
- **No ROS 2 required** - Pure language implementations
- **Lightweight** - Only message types, no middleware
- **Cross-platform** - Works anywhere the language runs
- **Type-safe** - Full type definitions

## Examples

See the `unified_output/` directory for examples of generated packages.



