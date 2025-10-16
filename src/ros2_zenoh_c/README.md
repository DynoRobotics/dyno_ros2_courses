# ROS 2 Zenoh C Library

A pure C library for ROS 2 message publishing/subscribing over Zenoh networks, designed for Buildroot integration.

## Features

- Pure C implementation (no C++ dependencies)
- Micro-CDR serialization support
- Optional Zenoh networking support
- Simple Makefile-based build system
- Buildroot package configuration included

## Building

### Standalone Build

```bash
make clean
make
make examples
```

### Buildroot Integration

1. Copy this package to your Buildroot `package/` directory:
   ```bash
   cp -r ros2_zenoh_c /path/to/buildroot/package/
   ```

2. Add to your Buildroot configuration:
   ```
   BR2_PACKAGE_ROS2_ZENOH_C=y
   BR2_PACKAGE_ROS2_ZENOH_C_ZENOH=y
   BR2_PACKAGE_ROS2_ZENOH_C_MICROCDR=y
   ```

3. Build with Buildroot:
   ```bash
   make ros2_zenoh_c
   ```

## Dependencies

- **Micro-CDR**: For message serialization (optional)
- **Zenoh C**: For networking (optional)

## API Usage

```c
#include "ros2_zenoh_c/ros2_zenoh_c.h"

// Initialize node
ros2_zenoh_node_t* node;
ros2_zenoh_node_init(&node, "my_node");

// Create publisher
ros2_zenoh_publisher_t* publisher;
ros2_zenoh_create_publisher(node, &publisher, 
    "/cmd_vel", "geometry_msgs/msg/Twist", sizeof(ros2_twist_t));

// Publish message
ros2_twist_t twist = {0};
twist.linear.x = 1.0;
twist.angular.z = 0.5;
ros2_zenoh_publish(publisher, &twist);

// Cleanup
ros2_zenoh_publisher_destroy(publisher);
ros2_zenoh_node_destroy(node);
```

## Current Status

- ✅ Basic library structure
- ✅ Micro-CDR serialization
- ✅ Makefile build system
- ✅ Buildroot package configuration
- ⚠️ Zenoh integration (stub implementation)
- ⚠️ Subscriber functionality (stub implementation)

## Next Steps

1. Implement proper Zenoh C API integration
2. Add subscriber functionality
3. Add more ROS 2 message types
4. Add unit tests
5. Add documentation

## License

Apache License 2.0