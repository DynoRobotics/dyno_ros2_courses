# Module 5: ROS2 Debugging & Development Tools

**Duration**: 45 minutes  
**Prerequisites**: Modules 1-4 completed  
**Learning Objectives**: Master debugging ROS2 systems using Python and C++ with VSCode, introspection tools, and troubleshooting workflows

---

## Learning Objectives

By the end of this module, participants will be able to:

1. **Debug Python and C++ ROS2 nodes** using VSCode's integrated debugger
2. **Use ROS2 introspection tools** for system analysis and troubleshooting
3. **Record and analyze system data** with ros2 bag for offline debugging
4. **Identify and resolve common ROS2 issues** in both Python and C++ efficiently

---

## Section 1: Python Debugging with VSCode (15 minutes)

### Understanding Our Workshop System

We'll use the existing incrementer/relay system for debugging practice:

- **Incrementer Node** (Python): Receives numbers, adds 1, publishes result
- **Delayed Relay Node** (Python): Receives numbers, waits 1 second, republishes
- **System Flow**: `number` → `incrementer` → `incremented_number` → `delayed_relay` → `delayed_number` → back to `number`

### Enhanced VSCode Debug Configuration

Let's enhance the existing `.vscode/launch.json` with ROS2-specific debugging configurations:

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Python: Current File",
      "type": "python",
      "request": "launch",
      "program": "${file}",
      "console": "integratedTerminal",
      "justMyCode": false
    },
    {
      "name": "Debug Python Incrementer Node",
      "type": "python",
      "request": "launch",
      "program": "${workspaceFolder}/src/vscode_py/vscode_py/incrementer_node.py",
      "console": "integratedTerminal",
      "justMyCode": false,
      "env": {
        "ROS_DOMAIN_ID": "0",
        "PYTHONPATH": "${workspaceFolder}/install/vscode_py/lib/python3.10/site-packages:${env:PYTHONPATH}"
      },
      "cwd": "${workspaceFolder}"
    },
    {
      "name": "Debug Python Delayed Relay Node",
      "type": "python",
      "request": "launch",
      "program": "${workspaceFolder}/src/vscode_py/vscode_py/delayed_relay_node.py",
      "console": "integratedTerminal",
      "justMyCode": false,
      "env": {
        "ROS_DOMAIN_ID": "0",
        "PYTHONPATH": "${workspaceFolder}/install/vscode_py/lib/python3.10/site-packages:${env:PYTHONPATH}"
      },
      "cwd": "${workspaceFolder}"
    },
    {
      "name": "Python: Attach using Process Id",
      "type": "python",
      "request": "attach",
      "processId": "${command:pickProcess}",
      "justMyCode": false
    },
    {
      "name": "(gdbserver) Debug cpp incrementer",
      "request": "launch",
      "type": "cppdbg",
      "miDebuggerServerAddress": "localhost:3000",
      "cwd": "/",
      "program": "${workspaceFolder}/install/vscode_cpp/lib/vscode_cpp/incrementer_node"
    },
    {
      "name": "(gdb) Debug cpp incrementer locally",
      "type": "cppdbg",
      "request": "launch",
      "program": "${workspaceFolder}/install/vscode_cpp/lib/vscode_cpp/incrementer_node",
      "args": [],
      "stopAtEntry": false,
      "cwd": "${workspaceFolder}",
      "environment": [
        {
          "name": "ROS_DOMAIN_ID",
          "value": "0"
        }
      ],
      "externalConsole": false,
      "MIMode": "gdb",
      "setupCommands": [
        {
          "description": "Enable pretty-printing for gdb",
          "text": "-enable-pretty-printing",
          "ignoreFailures": true
        }
      ]
    }
  ]
}
```

### Hands-on: Debug the Python Incrementer Node

#### Step 1: Set Breakpoints in Incrementer Node

Open `src/vscode_py/vscode_py/incrementer_node.py` and examine the code:

```python
import rclpy
import rclpy.node

from rclpy.executors import ExternalShutdownException

import std_msgs.msg


class Incrementer:
    def __init__(self, node: rclpy.node.Node):
        self.node = node

        self.node.get_logger().info("Initailizing incrementer node")  # Note: Typo here!

        # Create publisher
        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.msg.Int32, topic="incremented_number", qos_profile=10
        )

        # Create subscriber
        self.subscriber = self.node.create_subscription(
            msg_type=std_msgs.msg.Int32,
            topic="number",
            callback=self.callback,
            qos_profile=10,
        )

    def callback(self, msg: std_msgs.msg.Int32):
        # SET BREAKPOINT HERE - Line 32
        self.node.get_logger().info(f"In callback with number: {msg.data}")

        number = msg.data
        incremented_number = number + 1  # SET BREAKPOINT HERE - Line 36

        # Publish incremented number
        self.publisher.publish(std_msgs.msg.Int32(data=incremented_number))
```

**Debugging Exercise:**

1. **Set breakpoints** on lines 32 and 36
2. **Launch debugger** using "Debug Python Incrementer Node" configuration
3. **Publish test message** in another terminal:
   ```bash
   ros2 topic pub /number std_msgs/msg/Int32 "data: 5" --once
   ```
4. **Inspect variables** when breakpoint hits:
   - `msg.data` (should be 5)
   - `number` (should be 5)
   - `incremented_number` (should be 6)

#### Step 2: Debug the Async Delayed Relay Node

Open `src/vscode_py/vscode_py/delayed_relay_node.py`:

```python
import rclpy
import rclpy.node

import std_msgs.msg

from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from vscode_py.async_primitives import async_sleep


class DelayedRelay:
    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.node.get_logger().info("Initializing delayed relay node")

        # Change the default callback group to allow for async callbacks
        self.node._default_callback_group = ReentrantCallbackGroup()

        # Create subscriber
        self.number_sub = node.create_subscription(
            std_msgs.msg.Int32, "number", self.callback, 10
        )

        # Create publisher
        self.number_pub = node.create_publisher(
            std_msgs.msg.Int32, "delayed_number", 10
        )

    async def callback(self, msg: std_msgs.msg.Int32):
        # SET BREAKPOINT HERE - Line 29
        # Sleep for one second
        await async_sleep(self.node, 1.0)  # SET BREAKPOINT HERE - Line 31

        # Publish message
        self.number_pub.publish(std_msgs.msg.Int32(data=msg.data))  # SET BREAKPOINT HERE - Line 34
```

**Advanced Debugging Exercise:**

1. **Debug async callback** - Set breakpoints on lines 29, 31, and 34
2. **Watch async behavior** - Notice how the debugger handles async/await
3. **Inspect timing** - Use debugger to understand the 1-second delay

---

## Section 2: C++ Debugging with GDB Integration (15 minutes)

### Understanding the C++ Incrementer Node

Examine `src/vscode_cpp/src/incrementer_node.cpp`:

```cpp
// Copyright 2023 Samuel Lindgren

#include "incrementer_node.hpp"

IncrementerNode::IncrementerNode() : Node("incrementer") {
  RCLCPP_INFO(this->get_logger(), "Incrementer node started");

  step_size_ = 1; // Default step size

  // Allow step size to be set from the launch file
  declare_parameter("step_size", step_size_);
  get_parameter("step_size", step_size_);

  incremented_number_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("incremented_number", 10);

  auto number_callback = [this](std_msgs::msg::Int32::SharedPtr msg) {
    // SET BREAKPOINT HERE - Line 18
    auto incremented_number_msg = std::make_unique<std_msgs::msg::Int32>();
    int number = msg->data;  // SET BREAKPOINT HERE - Line 20
    int incremented_number = number + step_size_;  // SET BREAKPOINT HERE - Line 21
    incremented_number_msg->data = incremented_number;
    incremented_number_pub_->publish(std::move(incremented_number_msg));
  };

  number_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "number", 10, number_callback);
}
```

### Method 1: GDB Server Debugging (Remote Debugging)

#### Step 1: Enable GDB Server in Launch File

Modify `src/vscode_bringup/launch/cpp_incr_relay_system.launch.py`:

```python
incrementer_node = Node(
    package="vscode_cpp",
    executable="incrementer_node",
    name="incrementer",
    output="screen",
    parameters=[{"step_size": 2}],
    prefix=["gdbserver localhost:3000"],  # Uncomment this line
)
```

#### Step 2: Launch System with GDB Server

```bash
# Build the workspace first
cd /workspace
colcon build --packages-select vscode_cpp

# Source the workspace
source install/setup.bash

# Launch with GDB server enabled
ros2 launch vscode_bringup cpp_incr_relay_system.launch.py
```

#### Step 3: Attach VSCode Debugger

1. **Set breakpoints** in the C++ code (lines 18, 20, 21)
2. **Start debugging** using "(gdbserver) Debug cpp incrementer" configuration
3. **Publish test message**:
   ```bash
   ros2 topic pub /number std_msgs/msg/Int32 "data: 10" --once
   ```
4. **Inspect variables**:
   - `msg->data` (should be 10)
   - `number` (should be 10)
   - `step_size_` (should be 2 from parameter)
   - `incremented_number` (should be 12)

### Method 2: Local GDB Debugging

#### Step 1: Build with Debug Symbols

```bash
cd /workspace
colcon build --packages-select vscode_cpp --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

#### Step 2: Debug Locally

1. **Use "(gdb) Debug cpp incrementer locally"** configuration
2. **Set breakpoints** in C++ code
3. **Start debugging** (F5)
4. **In another terminal**, publish messages to trigger breakpoints

### Advanced C++ Debugging Techniques

#### Debugging with Parameters

Test parameter debugging by modifying the launch file:

```python
incrementer_node = Node(
    package="vscode_cpp",
    executable="incrementer_node",
    name="incrementer",
    output="screen",
    parameters=[{"step_size": 5}],  # Change step size
)
```

**Debug Exercise:**

1. Set breakpoint where `step_size_` is used
2. Verify parameter value is correctly loaded
3. Test with different parameter values

---

## Section 3: ROS2 Introspection and System Debugging (10 minutes)

### Command-Line Debugging Tools

#### Essential ROS2 Commands for Debugging

```bash
# System Overview
ros2 node list                    # List all running nodes
ros2 topic list                   # List all active topics
ros2 service list                 # List all available services

# Node Information
ros2 node info /incrementer       # Detailed node information
ros2 node info /delayed_relay     # Check subscribers/publishers

# Topic Debugging
ros2 topic info /number           # Topic details (type, publishers, subscribers)
ros2 topic echo /number           # Monitor messages in real-time
ros2 topic hz /number             # Check publication frequency
ros2 topic bw /number             # Check bandwidth usage

# Message Publishing for Testing
ros2 topic pub /number std_msgs/msg/Int32 "data: 42" --once
ros2 topic pub /number std_msgs/msg/Int32 "data: 100" --rate 1

# Parameter Debugging
ros2 param list                   # List all parameters
ros2 param get /incrementer step_size  # Get specific parameter
ros2 param set /incrementer step_size 3  # Set parameter at runtime
```

### Visual Debugging with rqt Tools

#### rqt_graph - System Visualization

```bash
# Install rqt tools if not available
sudo apt update && sudo apt install ros-humble-rqt*

# Launch system visualization
rqt_graph
```

**What to look for:**

- **Node connections** - Are nodes properly connected?
- **Topic flow** - Is data flowing in the expected direction?
- **Missing connections** - Are there broken links?

#### rqt_console - Log Management

```bash
# Launch log console
rqt_console
```

**Debugging with logs:**

- **Filter by node** - Focus on specific node logs
- **Log levels** - Debug, Info, Warn, Error, Fatal
- **Search functionality** - Find specific log messages

#### rqt_topic - Real-time Message Monitoring

```bash
# Launch topic monitor
rqt_topic
```

**Features:**

- **Message inspection** - See message contents in real-time
- **Publication rate** - Monitor message frequency
- **Message plotting** - Visualize numeric data over time

### Hands-on: System Debugging Exercise

#### Step 1: Launch the Complete System

```bash
# Terminal 1: Launch Python system
ros2 launch vscode_bringup incr_relay_system.launch.py

# Terminal 2: Launch C++ system (alternative)
ros2 launch vscode_bringup cpp_incr_relay_system.launch.py
```

#### Step 2: System Analysis

```bash
# Terminal 3: Analyze the system
ros2 node list
ros2 topic list
rqt_graph

# Check connections
ros2 topic info /number
ros2 topic info /incremented_number
ros2 topic info /delayed_number
```

#### Step 3: Test System Behavior

```bash
# Publish initial number
ros2 topic pub /number std_msgs/msg/Int32 "data: 1" --once

# Monitor all topics
ros2 topic echo /number &
ros2 topic echo /incremented_number &
ros2 topic echo /delayed_number &

# Watch the feedback loop!
```

---

## Section 4: Data Recording and Analysis (5 minutes)

### Using ros2 bag for Debugging

#### Recording System Data

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /number /incremented_number /delayed_number

# Record with custom bag name
ros2 bag record -o debug_session_$(date +%Y%m%d_%H%M%S) /number /incremented_number
```

#### Analyzing Recorded Data

```bash
# List bag contents
ros2 bag info rosbag2_2024_01_15-10_30_45

# Play back data
ros2 bag play rosbag2_2024_01_15-10_30_45

# Play at different speeds
ros2 bag play rosbag2_2024_01_15-10_30_45 --rate 0.5  # Half speed
ros2 bag play rosbag2_2024_01_15-10_30_45 --rate 2.0  # Double speed

# Play specific topics only
ros2 bag play rosbag2_2024_01_15-10_30_45 --topics /number /incremented_number
```

#### Debugging with Bag Files

**Use Case: Timing Analysis**

```bash
# Record system behavior
ros2 bag record /number /incremented_number /delayed_number

# Analyze timing in another session
ros2 bag play your_bag_file
ros2 topic hz /number
ros2 topic hz /incremented_number
ros2 topic hz /delayed_number
```

---

## Common Issues and Solutions

### Python Debugging Issues

#### Issue 1: "Module not found" errors

**Solution:**

```bash
# Ensure workspace is built and sourced
cd /workspace
colcon build --packages-select vscode_py
source install/setup.bash
```

#### Issue 2: Breakpoints not hitting

**Solution:**

- Check `"justMyCode": false` in launch.json
- Verify correct Python interpreter is selected
- Ensure node is actually receiving messages

#### Issue 3: Async debugging complexity

**Solution:**

- Use step-over (F10) instead of step-into for async calls
- Set breakpoints after await statements
- Use watch expressions for async state variables

### C++ Debugging Issues

#### Issue 1: No debug symbols

**Solution:**

```bash
# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

#### Issue 2: GDB server connection fails

**Solution:**

- Check if port 3000 is available: `netstat -ln | grep 3000`
- Try different port in launch file and launch.json
- Ensure gdbserver is installed: `sudo apt install gdbserver`

#### Issue 3: Optimized code debugging

**Solution:**

- Use Debug build type
- Add `-O0` flag to disable optimization
- Use `-g` flag for debug symbols

### System Integration Issues

#### Issue 1: Nodes not communicating

**Debugging steps:**

```bash
ros2 node list          # Are both nodes running?
ros2 topic list         # Are topics created?
rqt_graph              # Visual connection check
ros2 topic info /topic  # Check publishers/subscribers
```

#### Issue 2: Message type mismatches

**Debugging steps:**

```bash
ros2 interface show std_msgs/msg/Int32  # Check message definition
ros2 topic echo /topic                  # See actual message format
```

#### Issue 3: Timing issues

**Debugging steps:**

```bash
ros2 topic hz /topic    # Check publication rate
ros2 bag record         # Record for offline analysis
rqt_plot               # Visualize timing patterns
```

---

## Module Summary

### What We Accomplished:

1. ✅ **Python debugging** with VSCode using real workshop nodes
2. ✅ **C++ debugging** with GDB integration and remote debugging
3. ✅ **ROS2 introspection tools** for system analysis
4. ✅ **Data recording and analysis** with ros2 bag
5. ✅ **Common issue resolution** for both Python and C++

### Key Debugging Skills Learned:

- **Breakpoint debugging** in both Python and C++
- **Variable inspection** and watch expressions
- **System visualization** with rqt_graph
- **Real-time monitoring** with command-line tools
- **Data recording** for offline analysis
- **Multi-language debugging** in the same system

### Professional Debugging Workflow:

1. **Reproduce the issue** consistently
2. **Use system tools** (ros2 commands, rqt) for overview
3. **Set strategic breakpoints** in suspected problem areas
4. **Inspect variables** and system state
5. **Record data** for offline analysis if needed
6. **Test fixes** systematically

### Next Steps:

- **Module 6**: Simulation with Gazebo
- **Practice**: Debug your own ROS2 systems
- **Advanced**: Multi-robot debugging scenarios

---

## Additional Resources

### VSCode Extensions for ROS2 Debugging:

- **Python** - Microsoft Python extension
- **C/C++** - Microsoft C++ extension
- **ROS** - Microsoft ROS extension
- **CMake Tools** - For C++ build debugging

### Documentation:

- [VSCode Python Debugging](https://code.visualstudio.com/docs/python/debugging)
- [VSCode C++ Debugging](https://code.visualstudio.com/docs/cpp/cpp-debug)
- [ROS2 Debugging Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [GDB Documentation](https://www.gnu.org/software/gdb/documentation/)

### Community Resources:

- [ROS Discourse Debugging](https://discourse.ros.org/c/general/debugging)
- [Stack Overflow ROS2](https://stackoverflow.com/questions/tagged/ros2)
