# Multi-Language ROS 2 Message Generator

This tool generates ROS 2 message types for multiple languages with proper dependency resolution and cross-language conversion capabilities, especially designed for Tauri applications.

## Features

- **Multi-Language Support**: Generate message types for Python, Rust, C, and TypeScript
- **Dependency Resolution**: Automatically handles message dependencies (e.g., Twist depends on Vector3)
- **Tauri Integration**: Specialized generators for Tauri web applications
- **CDR Serialization**: Support for manual CDR serialization without ROS 2 dependencies
- **Cross-Language Conversion**: Built-in conversion functions between Rust and TypeScript

## Quick Start

### 1. Generate Multi-Language Types

```bash
# Generate types for a single message
python3 multi_lang_generator.py --input /opt/ros/jazzy/share/geometry_msgs/msg/Twist.msg --output output --languages python rust typescript --verbose

# Generate types for all messages in a package
python3 multi_lang_generator.py --input /opt/ros/jazzy/share/geometry_msgs/msg --output output --languages python rust typescript --tauri --verbose
```

### 2. Generate Complete Tauri Integration

```bash
# Generate complete Tauri integration for Twist message
python3 tauri_integration_generator.py /opt/ros/jazzy/share/geometry_msgs/msg/Twist.msg tauri_output

# Generate for multiple messages
python3 tauri_integration_generator.py /opt/ros/jazzy/share/geometry_msgs/msg tauri_output
```

## Generated Files

### Multi-Language Generator Output

```
output/
├── python/
│   └── geometry_msgs/
│       ├── twist.py
│       └── vector3.py
├── rust/
│   └── geometry_msgs/
│       ├── twist.rs
│       └── vector3.rs
├── c/
│   └── geometry_msgs/
│       ├── twist.h
│       └── vector3.h
├── typescript/
│   └── geometry_msgs/
│       ├── twist.ts
│       └── vector3.ts
└── tauri_conversions/
    ├── twist_conversions.rs
    └── vector3_conversions.rs
```

### Tauri Integration Output

```
tauri_output/
├── ros2_backend.rs          # Complete Rust backend with Tauri commands
├── ros2Types.ts             # TypeScript types and API functions
├── Ros2ControlPanel.tsx     # Complete React component
└── tauri_config.json        # Tauri configuration
```

## Usage Examples

### Python Usage

```python
from geometry_msgs.twist import Twist
from geometry_msgs.vector3 import Vector3

# Create a Twist message
twist = Twist(
    linear=Vector3(x=1.0, y=0.0, z=0.0),
    angular=Vector3(x=0.0, y=0.0, z=0.5)
)

# Convert to dictionary for JSON serialization
twist_dict = twist.to_dict()

# Create from dictionary
twist_from_dict = Twist.from_dict(twist_dict)
```

### Rust Usage

```rust
use geometry_msgs::{Twist, Vector3};

// Create a Twist message
let twist = Twist {
    linear: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
    angular: Vector3 { x: 0.0, y: 0.0, z: 0.5 },
};

// Convert to JSON
let json = twist.to_json()?;

// Create from JSON
let twist_from_json = Twist::from_json(&json)?;
```

### TypeScript Usage

```typescript
import { Twist, Vector3, publishTwist, useTwist } from './ros2Types';

// Create a Twist message
const twist: Twist = {
  linear: { x: 1.0, y: 0.0, z: 0.0 },
  angular: { x: 0.0, y: 0.0, z: 0.5 }
};

// Publish using Tauri command
await publishTwist(twist);

// Use React hook
const { messages, publish } = useTwist();
```

### C Usage

```c
#include "geometry_msgs/twist.h"
#include "geometry_msgs/vector3.h"

// Create a Twist message
ros2_twist_t twist = {
    .linear = { .x = 1.0, .y = 0.0, .z = 0.0 },
    .angular = { .x = 0.0, .y = 0.0, .z = 0.5 }
};

// Serialize to CDR
uint8_t buffer[1024];
size_t serialized_size;
serialize_ros2_twist(&twist, buffer, sizeof(buffer), &serialized_size);

// Deserialize from CDR
ros2_twist_t deserialized_twist;
deserialize_ros2_twist(buffer, serialized_size, &deserialized_twist);
```

## Tauri Integration

### 1. Backend Integration

Copy the generated `ros2_backend.rs` to your Tauri `src/` directory and add the commands to your `main.rs`:

```rust
// In src/main.rs
mod ros2_backend;

fn main() {
    tauri::Builder::default()
        .invoke_handler(tauri::generate_handler![
            ros2_backend::publish_twist,
            ros2_backend::subscribe_twist,
            ros2_backend::publish_vector3,
            ros2_backend::subscribe_vector3,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
```

### 2. Frontend Integration

Copy the generated `ros2Types.ts` to your frontend `src/` directory and use the React component:

```tsx
// In src/App.tsx
import { Ros2ControlPanel } from './Ros2ControlPanel';

function App() {
  return (
    <div>
      <Ros2ControlPanel />
    </div>
  );
}
```

### 3. Configuration

Update your `tauri.conf.json` with the generated commands:

```json
{
  "app": {
    "windows": [
      {
        "label": "main",
        "url": "/",
        "commands": [
          "publish_twist",
          "subscribe_twist",
          "publish_vector3",
          "subscribe_vector3"
        ]
      }
    ]
  }
}
```

## Advanced Features

### Custom Message Types

The generator automatically handles custom message types and their dependencies. For example, if `Twist` depends on `Vector3`, the generator will:

1. Generate `Vector3` first
2. Generate `Twist` with proper imports
3. Handle the dependency chain correctly

### CDR Serialization

For manual CDR serialization without ROS 2 dependencies:

```python
# Python CDR serialization
twist_bytes = twist.serialize_cdr()

# C CDR serialization
uint8_t buffer[1024];
size_t size;
serialize_ros2_twist(&twist, buffer, sizeof(buffer), &size);
```

### Cross-Language Conversion

Built-in conversion functions for Tauri applications:

```rust
// Rust to TypeScript
let json = twist.to_typescript()?;
```

```typescript
// TypeScript to Rust
const rustData = typescriptToRustTwist(twistData);
```

## Command Line Options

### Multi-Language Generator

```bash
python3 multi_lang_generator.py [OPTIONS]

Options:
  -i, --input PATH          Input .msg file or directory
  -o, --output PATH         Output directory
  -l, --languages LANG      Languages to generate (python, rust, c, typescript)
  -t, --tauri              Generate Tauri conversion functions
  -p, --package NAME        Package name override
  -v, --verbose            Verbose output
```

### Tauri Integration Generator

```bash
python3 tauri_integration_generator.py <input_msg_file_or_dir> <output_dir>
```

## Integration with Existing Projects

### For Your Tauri ROS 2 App

1. **Generate types for your specific messages**:
   ```bash
   python3 tauri_integration_generator.py /opt/ros/jazzy/share/geometry_msgs/msg/Twist.msg ./generated
   ```

2. **Replace your existing ros2.rs**:
   ```bash
   cp generated/ros2_backend.rs src/ros2.rs
   ```

3. **Update your App.tsx**:
   ```bash
   cp generated/Ros2ControlPanel.tsx src/Ros2ControlPanel.tsx
   ```

4. **Add the types to your frontend**:
   ```bash
   cp generated/ros2Types.ts src/ros2Types.ts
   ```

### For C Projects

1. **Generate C headers**:
   ```bash
   python3 multi_lang_generator.py --input /opt/ros/jazzy/share/geometry_msgs/msg --output c_output --languages c
   ```

2. **Include in your CMakeLists.txt**:
   ```cmake
   include_directories(c_output/c/geometry_msgs)
   ```

3. **Link with Micro-CDR**:
   ```cmake
   find_package(microcdr REQUIRED)
   target_link_libraries(your_target microcdr)
   ```

## Best Practices

1. **Generate all dependencies**: Always generate the entire package to ensure all dependencies are resolved
2. **Use Tauri integration**: For web applications, use the Tauri integration generator for complete setup
3. **Version control**: Commit generated files to version control for consistency
4. **CI/CD integration**: Add generation to your build pipeline for automated updates

## Troubleshooting

### Common Issues

1. **Missing dependencies**: Ensure you generate the entire package, not just individual messages
2. **Import errors**: Check that all generated files are in the correct directory structure
3. **Tauri commands not found**: Verify your `tauri.conf.json` includes all generated commands

### Debug Mode

Use `--verbose` flag for detailed output:

```bash
python3 multi_lang_generator.py --input /opt/ros/jazzy/share/geometry_msgs/msg --output output --verbose
```

This will show you exactly which files are being generated and any issues encountered.