# ROS 2 Zenoh Rust Implementation

This is the core Rust library for ROS 2 over Zenoh transport.

## Status

⚠️ **Work in Progress** - Currently being updated to match Zenoh 0.11 API

### Current Issues

The code needs updates for Zenoh 0.11 API changes:
- `res_sync()` → `wait()` or async/await pattern
- `Config` is now private, use `config::` module
- `Encoding::ZENOH_BYTES` → new encoding system
- `zenoh::pubsub::Subscriber` type changes

### Next Steps

1. Update to use async/await with tokio runtime
2. Fix encoding system for CDR data
3. Update liveliness token API
4. Test with ROS 2 DDS interop

### Alternative Approach

Consider using the existing Python implementation pattern with PyO3 bindings, or wait for Zenoh API stabilization.

## Architecture

- **raw.rs**: Low-level publisher/subscriber accepting bytes (for NAPI)
- **node.rs**: Node abstraction managing session
- **types.rs**: DDS interop key generation
- **attachments.rs**: ROS 2 metadata attachment handling
- **liveliness.rs**: ROS 2 discovery via liveliness tokens

## Usage (Once Fixed)

```rust
use ros2_zenoh_rs::Node;

let node = Node::new("my_node", "/")?;
let pub = node.create_raw_publisher("/topic", "geometry_msgs/msg/Twist")?;
pub.publish(&cdr_bytes)?;
```

