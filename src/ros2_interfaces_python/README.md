# ROS 2 Interfaces Python

A Python package providing simplified dataclasses for ROS 2 interface types (messages, services, actions). These lightweight alternatives don't require ROS 2 installation and can be used with any transport implementation.

## Features

- **Lightweight**: Pure Python dataclasses, no ROS 2 dependencies
- **Complete**: Supports messages, services, and actions
- **Compatible**: Can be converted to/from official ROS 2 message types
- **Extensible**: Easy to generate new interface types from ROS 2 definitions

## Installation

```bash
cd src/ros2_interfaces_python
pip install -e .
```

## Usage

### Importing Message Types

```python
# Import from specific package
from ros2_interfaces_python.msg.geometry_msgs import Vector3, Twist
from ros2_interfaces_python.msg.std_msgs import Header, String

# Or import from main package (convenience)
from ros2_interfaces_python import Vector3, Twist, Header, String
```

### Creating Messages

```python
from ros2_interfaces_python import Vector3, Twist, Header

# Create a Vector3
v3 = Vector3(x=1.0, y=2.0, z=3.0)

# Create a Twist
twist = Twist(linear=Vector3(x=1.0), angular=Vector3(z=0.5))

# Create a Header
header = Header(frame_id="map")
```

### Converting to/from ROS 2 Messages

```python
from ros2_interfaces_python import Vector3, Twist
from geometry_msgs.msg import Vector3 as ROS2Vector3, Twist as ROS2Twist

# Convert simplified to ROS 2
ros2_vector = ROS2Vector3()
ros2_vector.x = 1.0
ros2_vector.y = 2.0
ros2_vector.z = 3.0

# Convert ROS 2 to simplified
simple_vector = Vector3(x=ros2_vector.x, y=ros2_vector.y, z=ros2_vector.z)
```

## Package Structure

```
ros2_interfaces_python/
├── msg/                   # Message types
│   ├── geometry_msgs.py   # geometry_msgs message types
│   ├── std_msgs.py        # std_msgs message types
│   └── __init__.py
├── srv/                   # Service types (future)
│   └── __init__.py
├── action/                # Action types (future)
│   └── __init__.py
└── __init__.py
```

## Available Message Types

### geometry_msgs
- `Vector3`, `Point`, `Quaternion`
- `Pose`, `Twist`, `Transform`
- `PoseStamped`, `TwistStamped`, `TransformStamped`
- `Accel`, `Wrench`, `Pose2D`
- `PoseArray`, `Point32`

### std_msgs
- `Header`, `Empty`, `String`
- `Bool`, `Int8`, `Int16`, `Int32`, `Int64`
- `UInt8`, `UInt16`, `UInt32`, `UInt64`
- `Float32`, `Float64`, `Byte`, `Char`
- `ColorRGBA`, `MultiArrayLayout`
- Various `MultiArray` types

## Future Plans

- Service type support (`srv/`)
- Action type support (`action/`)
- Automatic generation from ROS 2 interface definitions
- Support for more ROS 2 packages
