"""
Bundled Essential Messages

This package contains minimal ROS2 message types bundled with ros2_zenoh_python:

- rcl_interfaces.msg.Log      - For /rosout logging
- builtin_interfaces.msg.Time - For timestamps
- geometry_msgs.msg.Twist     - For examples and tests
- geometry_msgs.msg.Vector3   - For examples and tests

These messages allow ros2_zenoh_python to work standalone without requiring
the full ros2_interfaces_py package (which is 5MB+).

If you need other ROS2 standard messages, install ros2_interfaces_py:
    pip install ros2_interfaces_py

Or generate your own custom messages using the generator in tools/.
"""

__all__ = []

