# ROS 2 Zenoh Python

A Python package that provides Zenoh as an alternative transport to `rclpy` for ROS 2. This package enables direct Zenoh communication while maintaining ROS 2 compatibility.

## Features

- **ROS 2 Compatible**: Works with standard ROS 2 tools like `ros2 topic echo`
- **Zenoh Transport**: Uses Zenoh for efficient, scalable communication
- **Message Serialization**: Supports ROS 2 message types with CDR serialization
- **Liveliness Tokens**: Publishes ROS 2 metadata for topic discovery
- **Easy API**: Simple Publisher/Subscriber interface similar to `rclpy`

## Installation

### Prerequisites

- Python 3.8+
- Zenoh Python library
- pycdr2 for message serialization
- ros2-interfaces-python for message types
- ROS 2 (optional, for message definitions)

### Install from source

```bash
# Install interfaces package first
cd src/ros2_interfaces_python
pip install -e .

# Install zenoh transport package
cd ../ros2_zenoh_python
pip install -e .
```

### Install with ROS 2 support

```bash
pip install -e .[ros2]
```

## Package Structure

The package is organized with separate concerns:

```
ros2_interfaces_python/    # Separate interfaces package
├── msg/                   # Message types
│   ├── geometry_msgs.py
│   ├── std_msgs.py
│   └── ...
├── srv/                   # Service types (future)
└── action/                # Action types (future)

ros2_zenoh_python/         # Core Zenoh transport package
├── publisher.py           # Publisher class
├── subscriber.py          # Subscriber class
├── node.py               # Node class
├── message_serializer.py  # CDR serialization
├── liveliness_manager.py # ROS 2 metadata
└── ...
```

### Importing Message Types

```python
# Import from interfaces package
from ros2_interfaces_python.msg.geometry_msgs import Vector3, Twist
from ros2_interfaces_python.msg.std_msgs import Header, String

# Or import from interfaces package (convenience)
from ros2_interfaces_python import Vector3, Twist, Header, String

# Or import directly from zenoh package (re-exports interfaces)
from ros2_zenoh_python import Vector3, Twist, Header, String
```

## Quick Start

### Recommended: Node-based Usage (Shared Session)

```python
from ros2_zenoh_python import Node, Publisher, Subscriber
from geometry_msgs.msg import Twist

def callback(data):
    print(f"Received: {data}")

# Create a single node that manages the Zenoh session
with Node("my_node") as node:
    # Create multiple publishers and subscribers sharing the same session
    pub = node.create_publisher('/turtle1/cmd_vel', Twist)
    sub = node.create_subscriber('/turtle1/cmd_vel', Twist, callback)
    
    # Publish messages
    pub.publish_twist(linear_x=1.0, angular_z=0.5)
    
    # Keep running to receive messages
    import time
    time.sleep(10)
```

### Legacy: Individual Publishers/Subscribers (Separate Sessions)

```python
from ros2_zenoh_python import Publisher, Subscriber
from geometry_msgs.msg import Twist

def callback(data):
    print(f"Received: {data}")

# Create publisher (creates its own Zenoh session)
with Publisher('/turtle1/cmd_vel', Twist) as pub:
    pub.publish_twist(linear_x=1.0, angular_z=0.5)

# Create subscriber (creates its own Zenoh session)
with Subscriber('/turtle1/cmd_vel', Twist, callback) as sub:
    import time
    time.sleep(10)
```

## API Reference

### Node

```python
Node(node_name="zenoh_node", node_namespace="", zenoh_config=None)
```

- `node_name`: ROS 2 node name
- `node_namespace`: ROS 2 node namespace
- `zenoh_config`: Zenoh configuration

**Methods:**
- `create_publisher(topic_name, message_type, **kwargs)`: Create a publisher
- `create_subscriber(topic_name, message_type, callback, **kwargs)`: Create a subscriber
- `destroy_publisher(publisher)`: Destroy a publisher
- `destroy_subscriber(subscriber)`: Destroy a subscriber
- `close()`: Close node and clean up all resources

### Publisher

```python
Publisher(topic_name, message_type, node_name="zenoh_publisher", 
          node_namespace="", qos_profile=None, zenoh_config=None)
```

- `topic_name`: ROS 2 topic name (e.g., "/turtle1/cmd_vel")
- `message_type`: ROS 2 message type class
- `node_name`: ROS 2 node name
- `node_namespace`: ROS 2 node namespace
- `qos_profile`: QoS profile settings
- `zenoh_config`: Zenoh configuration

**Methods:**
- `publish(message)`: Publish a ROS 2 message
- `publish_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)`: Convenience method for Twist messages
- `close()`: Close publisher and clean up resources

### Subscriber

```python
Subscriber(topic_name, message_type, callback, node_name="zenoh_subscriber",
           node_namespace="", qos_profile=None, zenoh_config=None)
```

- `topic_name`: ROS 2 topic name
- `message_type`: ROS 2 message type class
- `callback`: Callback function to handle received messages
- `node_name`: ROS 2 node name
- `node_namespace`: ROS 2 node namespace
- `qos_profile`: QoS profile settings
- `zenoh_config`: Zenoh configuration

**Methods:**
- `close()`: Close subscriber and clean up resources

## Configuration

### Zenoh Configuration

You can provide custom Zenoh configuration:

```python
import zenoh
from ros2_zenoh_python import Publisher

config = zenoh.Config()
config.insert_json5("mode", '"client"')
config.insert_json5("connect/endpoints", '["tcp/192.168.1.100:7447"]')

with Publisher('/my_topic', MyMessageType, zenoh_config=config) as pub:
    # Use custom Zenoh configuration
    pass
```

### QoS Profiles

```python
qos_profile = {
    'reliability': 'reliable',
    'durability': 'volatile',
    'history': 'keep_last',
    'depth': 10
}

with Publisher('/my_topic', MyMessageType, qos_profile=qos_profile) as pub:
    pass
```

## ROS 2 Compatibility

This package is designed to work with standard ROS 2 tools:

- `ros2 topic list`: Shows topics published by Zenoh publishers
- `ros2 topic echo`: Can receive messages from Zenoh publishers
- `ros2 topic info`: Shows topic information
- `ros2 node list`: Shows Zenoh nodes

## Message Types

Currently supported message types:

- `geometry_msgs.msg.Twist`
- `geometry_msgs.msg.Vector3`
- `builtin_interfaces.msg.Time`
- `rcl_interfaces.msg.Log`

Additional message types can be added by extending the `MessageSerializer` class.

## Examples

See the `examples/` directory for complete examples:

- `publisher_example.py`: Publisher example with both ROS 2 and simplified message types
- `subscriber_example.py`: Subscriber example with both ROS 2 and simplified message types
- `complete_example.py`: Complete publisher-subscriber workflow
- `rclpy_like_example.py`: rclpy-like interface demonstration

## Development

### Running Tests

```bash
pytest tests/
```

### Code Formatting

```bash
black ros2_zenoh_python/
flake8 ros2_zenoh_python/
```

### Type Checking

```bash
mypy ros2_zenoh_python/
```

## License

This project is licensed under the Apache License 2.0 and Eclipse Public License 2.0. See LICENSE files for details.

## Contributing

Contributions are welcome! Please see CONTRIBUTING.md for guidelines.

## Support

- GitHub Issues: [Report bugs and request features](https://github.com/eclipse-zenoh/ros2_zenoh_python/issues)
- Zenoh Community: [Join the discussion](https://github.com/eclipse-zenoh/zenoh/discussions)
- Documentation: [Zenoh Documentation](https://zenoh.io/docs/)
