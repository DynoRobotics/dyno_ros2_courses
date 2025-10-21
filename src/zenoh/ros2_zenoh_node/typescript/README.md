# @ros2-zenoh/node

High-level TypeScript API for ROS 2 communication over Zenoh, with automatic CDR serialization/deserialization.

## Features

- **Ergonomic API**: Pass TypeScript objects directly to `publish()`, no manual serialization needed
- **Type-safe**: Full TypeScript types for ROS 2 messages
- **Zero-copy where possible**: Built on native Rust implementation with NAPI bindings
- **ROS 2 compatible**: Works alongside standard ROS 2 nodes
- **No ROS 2 installation required**: Standalone implementation using Zenoh

## Installation

```bash
npm install @ros2-zenoh/node
```

## Quick Start

### Publisher

```typescript
import { Node, Twist, serializeTwistCDR, deserializeTwistCDR } from '@ros2-zenoh/node';

const node = new Node('my_node');

const pub = node.createPublisher<Twist>(
  '/cmd_vel',
  'geometry_msgs/msg/Twist',
  { serialize: serializeTwistCDR, deserialize: deserializeTwistCDR }
);

// Just pass the object!
pub.publish({
  linear: { x: 1.0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0.5 }
});
```

### Subscriber

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

## Architecture

This package is built on a three-layer architecture:

1. **ros2_zenoh_rs** - Pure Rust library with Zenoh 1.5.1
2. **@ros2-zenoh/native** - NAPI-RS bindings (low-level, byte-based)
3. **@ros2-zenoh/node** - High-level TypeScript API (this package)

## API

### Node

```typescript
class Node {
  constructor(name: string, namespace?: string);
  getName(): string;
  getNamespace(): string;
  createPublisher<T>(topic: string, msgType: string, serializer: SerializerFunctions<T>): Publisher<T>;
  createSubscriber<T>(topic: string, msgType: string, serializer: SerializerFunctions<T>, callback: (msg: T) => void): Subscriber<T>;
}
```

### Publisher

```typescript
class Publisher<T> {
  publish(message: T): void;
  getTopic(): string;
}
```

### Subscriber

```typescript
class Subscriber<T> {
  // Callback is set at construction
}
```

## Available Message Types

All standard ROS 2 message types are available with their serialization functions:

- `geometry_msgs`: `Point`, `Pose`, `Twist`, `Vector3`, etc.
- `std_msgs`: `Header`, `String`, `Int32`, etc.
- `builtin_interfaces`: `Time`, `Duration`

Each message type has corresponding `serialize<Type>CDR` and `deserialize<Type>CDR` functions.

## License

Apache-2.0

