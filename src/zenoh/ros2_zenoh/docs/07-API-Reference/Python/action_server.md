---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "ros2_zenoh_python/action_server.py"
last_generated: "2025-10-23T16:51:15.509238"
tags: ["api", "python", "auto-generated"]
---

# action_server

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `ros2_zenoh_python/action_server.py`

ROS2 Action Server implementation using Zenoh.

Actions in ROS2 are composed of:
- 3 Service Servers: send_goal, cancel_goal, get_result
- 2 Publishers: feedback, status

Based on rmw_zenoh design, actions are NOT RMW primitives but high-level
constructs built from existing services and topics.

