---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "ros2_zenoh_python/client.py"
last_generated: "2025-10-23T16:51:15.462750"
tags: ["api", "python", "auto-generated"]
---

# client

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `ros2_zenoh_python/client.py`

ROS2 Service Client implementation using Zenoh queries.

Based on rmw_zenoh design:
- Uses get (query) operation for service calls
- Service key expression: <domain_id>/<service_name>/<type_name>/<type_hash>
- Attachment contains: sequence_number, timestamp, client_gid

