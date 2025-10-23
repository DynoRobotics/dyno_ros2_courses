---
classification: public
llm_processing: allowed
schema_version: "1.0"
type: api-reference
auto_generated: true
source_file: "ros2_zenoh_python/logger.py"
last_generated: "2025-10-23T16:51:15.471991"
tags: ["api", "python", "auto-generated"]
---

# logger

⚠️ **This file is auto-generated. Do not edit manually.**

**Source**: `ros2_zenoh_python/logger.py`

Logger Module

Provides a RosoutHandler for Python's standard logging module.

Instead of ROS2's get_logger() pattern, use Python's standard:
    import logging
    logger = logging.getLogger(__name__)
    logger.info("Hello")

To enable /rosout publishing, add the handler:
    from ros2_zenoh_python.logger import RosoutHandler
    logging.root.addHandler(RosoutHandler(node))

## Functions

### `setup_logging`

```python
setup_logging(node: Node, level: int, publish_to_rosout: bool)
```

Setup standard Python logging with optional /rosout publishing.

This is the recommended way to configure logging for your ROS2 Zenoh nodes.

Args:
    node: Node instance (for publishing to /rosout)
    level: Logging level (e.g., logging.INFO, logging.DEBUG)
    publish_to_rosout: If True, add handler that publishes to /rosout topic

Example:
    from ros2_zenoh_python import Node
    from ros2_zenoh_python.logger import setup_logging
    import logging
    
    node = Node('my_node')
    setup_logging(node, level=logging.DEBUG)
    
    logger = logging.getLogger(__name__)
    logger.info("This goes to console AND /rosout!")

