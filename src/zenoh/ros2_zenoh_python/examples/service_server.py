#!/usr/bin/env python3
"""
Example ROS2 service server using ros2_zenoh_python.

Demonstrates creating a service server with Add Two Ints service.
"""

import asyncio
import logging
from pathlib import Path
import sys

# Add parent directory to path for imports
repo_root = Path(__file__).parent.parent
sys.path.insert(0, str(repo_root))
sys.path.insert(0, str(repo_root.parent / 'ros2_interfaces_py'))

from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs.example_interfaces.srv import AddTwoInts

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def main():
    """Main service server example."""
    logger.info("🚀 Starting service server example")
    
    # Define service callback
    def handle_add_two_ints(request: AddTwoInts.Request) -> AddTwoInts.Response:
        """Handle AddTwoInts service request."""
        result = request.a + request.b
        logger.info(f"📨 Request received: {request.a} + {request.b} = {result}")
        return AddTwoInts.Response(sum=result)
    
    # Create node and service
    async with Node('add_two_ints_server', enable_rosout=False) as node:
        logger.info(f"✅ Node created: {node.node_name}")
        
        # Create service
        srv = node.create_service(
            AddTwoInts,
            'add_two_ints',
            handle_add_two_ints
        )
        
        logger.info(f"🔧 Service ready: {srv.service_name}")
        logger.info(f"   Key expression: {srv.key_expr}")
        logger.info("\n   Waiting for service requests...")
        logger.info("   Press Ctrl+C to stop\n")
        
        # Spin
        await node.spin()
    
    logger.info("✅ Service server shutdown complete")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("\n🛑 Interrupted by user")


