#!/usr/bin/env python3
"""
Example ROS2 service client using ros2_zenoh_python.

Demonstrates creating a service client and calling Add Two Ints service.
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
    """Main service client example."""
    logger.info("🚀 Starting service client example")
    
    # Create node and client
    async with Node('add_two_ints_client', enable_rosout=False) as node:
        logger.info(f"✅ Node created: {node.node_name}")
        
        # Create client
        client = node.create_client(
            AddTwoInts,
            'add_two_ints'
        )
        
        logger.info(f"🔌 Client created: {client.service_name}")
        logger.info(f"   Key expression: {client.key_expr}")
        
        # Make several service calls
        test_cases = [
            (5, 7),
            (10, 20),
            (100, 200),
            (-5, 10),
            (0, 0),
        ]
        
        logger.info(f"\n📞 Making {len(test_cases)} service calls...\n")
        
        for i, (a, b) in enumerate(test_cases, 1):
            try:
                # Create request
                request = AddTwoInts.Request(a=a, b=b)
                
                logger.info(f"[{i}/{len(test_cases)}] Calling service: {a} + {b}")
                
                # Call service
                response = await client.call_async(request, timeout=5.0)
                
                logger.info(f"   ✅ Response: {response.sum}")
                
                # Brief delay between calls
                await asyncio.sleep(0.5)
                
            except TimeoutError:
                logger.error(f"   ❌ Service call timed out!")
            except Exception as e:
                logger.error(f"   ❌ Service call failed: {e}")
        
        logger.info("\n✅ All service calls complete")
    
    logger.info("✅ Client shutdown complete")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("\n🛑 Interrupted by user")


