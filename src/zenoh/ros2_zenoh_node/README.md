# ROS 2 over Zenoh - Rust + TypeScript Implementation

Complete implementation of ROS 2 communication over Zenoh 1.5.1, with three-layer architecture for maximum performance and ergonomics.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  TypeScript Layer (@ros2-zenoh/node)                        │
│  - High-level API with automatic serialization              │
│  - Type-safe message passing                                 │
│  - Ergonomic: publisher.publish({linear: {x: 1.0, ...}})   │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│  NAPI-RS Bindings (@ros2-zenoh/native)                      │
│  - Low-level byte-based API                                  │
│  - Thread-safe callbacks                                     │
│  - publishRaw(Buffer), callback(Buffer)                     │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│  Rust Core (ros2_zenoh_rs)                                   │
│  - Pure Rust with Zenoh 1.5.1                               │
│  - RawPublisher/RawSubscriber for NAPI                      │
│  - DDS key generation, attachments, liveliness             │
└─────────────────────────────────────────────────────────────┘
```

## Layer 1: Rust Core Library (`ros2_zenoh_rs`)

**Location**: `src/zenoh/ros2_zenoh_rs/`

Pure Rust implementation using Zenoh 1.5.1.

### Features
- ROS 2 Node abstraction
- RawPublisher/RawSubscriber (byte-based API for NAPI)
- DDS interop key generation
- Attachment handling for sequence numbers and publisher GIDs
- Liveliness tokens for ROS 2 metadata discovery
- Type registry for message type hashes

### Key Files
- `src/node.rs` - ROS 2 Node
- `src/raw.rs` - Raw byte-based publisher/subscriber
- `src/types.rs` - Type registry and DDS key generation
- `src/attachments.rs` - Attachment building/parsing
- `src/liveliness.rs` - Liveliness token management
- `examples/publisher.rs` - Rust usage example
- `examples/subscriber.rs` - Rust usage example

### Build
```bash
cd src/zenoh/ros2_zenoh_rs
cargo build --release
cargo build --examples
```

### Run Examples
```bash
# Publisher
cargo run --example publisher

# Subscriber
cargo run --example subscriber
```

## Layer 2: NAPI-RS Bindings (`@ros2-zenoh/native`)

**Location**: `src/zenoh/ros2_zenoh_node/`

Native Node.js addon built with NAPI-RS, exposing the Rust core to JavaScript.

### Features
- `NativeNode` - Node creation and management
- `NativePublisher` - Raw byte publishing
- `NativeSubscriber` - Raw byte subscription with callbacks
- Thread-safe function callbacks from Rust to JavaScript

### Key Files
- `src/lib.rs` - Module exports
- `src/node.rs` - NAPI Node wrapper
- `src/publisher.rs` - NAPI Publisher wrapper
- `src/subscriber.rs` - NAPI Subscriber wrapper with threadsafe callbacks
- `Cargo.toml` - Dependencies: napi 2.16, ros2_zenoh_rs
- `build.rs` - NAPI build script

### Build
```bash
cd src/zenoh/ros2_zenoh_node
cargo build --release
# The .so is automatically renamed to .node format
```

### API (Low-level)
```typescript
const native = require('@ros2-zenoh/native');

const node = new native.NativeNode('my_node', '/');
const pub = node.createRawPublisher('/topic', 'std_msgs/msg/String');
pub.publishRaw(Buffer.from(cdrBytes));

const sub = node.createRawSubscriber('/topic', 'std_msgs/msg/String', (buffer) => {
  // buffer is a Buffer containing CDR bytes
});
```

## Layer 3: TypeScript Wrapper (`@ros2-zenoh/node`)

**Location**: `src/zenoh/ros2_zenoh_node/typescript/`

High-level, ergonomic TypeScript API with automatic CDR serialization.

### Features
- Typed `Node`, `Publisher<T>`, `Subscriber<T>` classes
- Automatic CDR serialization/deserialization
- Full TypeScript type safety
- Re-exports all message types from `@ros2-cdr/interfaces-ts`

### Key Files
- `src/Node.ts` - High-level Node class
- `src/Publisher.ts` - Typed publisher with auto-serialization
- `src/Subscriber.ts` - Typed subscriber with auto-deserialization
- `src/types.ts` - SerializerFunctions interface
- `src/index.ts` - Main exports
- `examples/publisher_example.ts` - Usage example
- `examples/subscriber_example.ts` - Usage example

### Build
```bash
cd src/zenoh/ros2_zenoh_node/typescript
npm install
npm run build
```

### Usage

#### Publisher
```typescript
import { Node, Twist, serializeTwistCDR, deserializeTwistCDR } from '@ros2-zenoh/node';

const node = new Node('my_node');

const pub = node.createPublisher<Twist>(
  '/cmd_vel',
  'geometry_msgs/msg/Twist',
  { serialize: serializeTwistCDR, deserialize: deserializeTwistCDR }
);

// Just pass the TypeScript object!
pub.publish({
  linear: { x: 1.0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0.5 }
});
```

#### Subscriber
```typescript
import { Node, Twist, serializeTwistCDR, deserializeTwistCDR } from '@ros2-zenoh/node';

const node = new Node('my_node');

const sub = node.createSubscriber<Twist>(
  '/cmd_vel',
  'geometry_msgs/msg/Twist',
  { serialize: serializeTwistCDR, deserialize: deserializeTwistCDR },
  (msg) => {
    console.log('Received:', msg.linear.x);
  }
);
```

## CDR Serialization

All three layers use CDR (Common Data Representation) serialization for ROS 2 compatibility.

### Generated Message Packages
- **Python**: `ros2_interfaces_py` (using `pycdr2`)
- **Rust**: `ros2_interfaces_rs` (using `cdr` crate with serde)
- **TypeScript**: `@ros2-cdr/interfaces-ts` (using `@foxglove/cdr`)
- **C**: `ros2_interfaces_c` (using Micro-CDR)

All generated packages include:
- Message type definitions
- `serialize()` / `deserialize()` methods
- Full cross-language compatibility

## Testing Interoperability

### Rust Publisher ↔ TypeScript Subscriber
```bash
# Terminal 1: Rust publisher
cd src/zenoh/ros2_zenoh_rs
cargo run --example publisher

# Terminal 2: TypeScript subscriber
cd src/zenoh/ros2_zenoh_node/typescript
npx ts-node examples/subscriber_example.ts
```

### TypeScript Publisher ↔ Rust Subscriber
```bash
# Terminal 1: TypeScript publisher
cd src/zenoh/ros2_zenoh_node/typescript
npx ts-node examples/publisher_example.ts

# Terminal 2: Rust subscriber
cd src/zenoh/ros2_zenoh_rs
cargo run --example subscriber
```

### With ROS 2 Nodes
All implementations are compatible with standard ROS 2 nodes:
```bash
# ROS 2 subscriber
ros2 topic echo /turtle1/cmd_vel geometry_msgs/msg/Twist

# Any of our publishers will work!
```

## Dependencies

### Rust Core
- `zenoh = "1.5.1"` - Zenoh communication
- `serde = "1.0"` - Serialization framework
- `tokio = "1"` - Async runtime
- `anyhow = "1.0"` - Error handling
- `rand = "0.8"` - Random GID generation

### NAPI Bindings
- `napi = { version = "2.16", features = ["napi4"] }` - N-API bindings
- `napi-derive = "2.16"` - Derive macros
- `ros2_zenoh_rs` - Our Rust core library

### TypeScript Wrapper
- `@ros2-zenoh/native` - NAPI bindings (local)
- `@ros2-cdr/interfaces-ts` - Generated message types
- `typescript = "^5.0.0"` - TypeScript compiler

## Advantages

1. **Performance**: Rust core with zero-copy where possible
2. **Ergonomics**: TypeScript developers just pass objects
3. **Type Safety**: Full compile-time type checking
4. **Compatibility**: Works with standard ROS 2 nodes
5. **No ROS 2 Required**: Standalone implementation using Zenoh
6. **Cross-platform**: Linux x64/ARM64 support

## Tauri Compatibility

The same NAPI approach works for Tauri:
```rust
// Tauri command
#[tauri::command]
fn publish_message(data: Vec<u8>) -> Result<(), String> {
    // Use ros2_zenoh_rs directly
}
```

TypeScript frontend serializes before calling:
```typescript
const cdr = serializeTwistCDR(twist);
await invoke('publish_message', { data: Array.from(cdr) });
```

## License

Apache-2.0

## Authors

ROS 2 Zenoh Team

