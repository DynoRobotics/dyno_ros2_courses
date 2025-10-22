#!/usr/bin/env python3
"""
Standalone test of Zenoh service communication (peer-to-peer, no bridge needed).
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'src/zenoh/ros2_zenoh_python'))

from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs.example_interfaces.srv import AddTwoInts


async def main():
    print("=" * 70)
    print("ZENOH PEER-TO-PEER SERVICE TEST (no bridge needed)")
    print("=" * 70)
    
    # Define service handler
    def handle_add(request: AddTwoInts.Request) -> AddTwoInts.Response:
        result = request.a + request.b
        print(f"  [SERVER] Received: {request.a} + {request.b} = {result}")
        return AddTwoInts.Response(sum=result)
    
    # Create server node (will use peer mode by default since no session provided)
    print("\n1. Creating server node...")
    server_node = Node('standalone_server', enable_rosout=False)
    srv = server_node.create_service(AddTwoInts, 'test_add_standalone', handle_add)
    print(f"   ✅ Server ready: {srv.service_name}")
    
    # Create client node  
    print("\n2. Creating client node...")
    client_node = Node('standalone_client', enable_rosout=False)
    client = client_node.create_client(AddTwoInts, 'test_add_standalone')
    print(f"   ✅ Client ready: {client.service_name}")
    
    # Wait for discovery
    print("\n3. Waiting for discovery...")
    await asyncio.sleep(1.0)
    
    # Make service calls
    print("\n4. Making service calls...")
    test_cases = [(5, 7), (10, 20), (100, 200)]
    
    for a, b in test_cases:
        try:
            request = AddTwoInts.Request(a=a, b=b)
            print(f"  [CLIENT] Calling: {a} + {b}")
            response = await client.call_async(request, timeout=2.0)
            print(f"  [CLIENT] Response: {response.sum} ✅")
        except Exception as e:
            print(f"  [CLIENT] Failed: {e} ❌")
    
    # Cleanup
    print("\n5. Cleaning up...")
    await server_node.adestroy_node()
    await client_node.adestroy_node()
    
    print("\n" + "=" * 70)
    print("TEST COMPLETE!")
    print("=" * 70)


if __name__ == '__main__':
    asyncio.run(main())



