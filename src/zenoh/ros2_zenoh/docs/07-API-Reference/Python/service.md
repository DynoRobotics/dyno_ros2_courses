---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "ros2_zenoh_python/service.py"
last_generated: "2025-10-23T16:51:15.451012"
tags: ["api", "python", "auto-generated"]
---

# service

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `ros2_zenoh_python/service.py`

ROS2 Service Server implementation using Zenoh queryables.

Based on rmw_zenoh design:
- Uses declare_queryable for service servers
- Service key expression: <domain_id>/<service_name>/<type_name>/<type_hash>
- Attachment contains: sequence_number, timestamp, client_gid

