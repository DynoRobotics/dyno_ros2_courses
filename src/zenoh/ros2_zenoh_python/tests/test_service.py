"""
Test service functionality.
"""

import asyncio
import pytest
import sys
from pathlib import Path

# Add ros2_interfaces_py to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'ros2_interfaces_py'))

from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs.example_interfaces.srv import AddTwoInts


@pytest.mark.asyncio
async def test_service_basic():
    """Test basic service server and client communication."""
    import logging
    logging.basicConfig(level=logging.DEBUG)
    
    # Define service handler
    def handle_add(request: AddTwoInts.Request) -> AddTwoInts.Response:
        result = request.a + request.b
        print(f"[SERVER] Received request: {request.a} + {request.b} = {result}")
        return AddTwoInts.Response(sum=result)
    
    # Create server node
    server_node = Node('test_server', enable_rosout=False)
    print(f"[TEST] Server node created: {server_node.node_name}")
    
    # Create service
    srv = server_node.create_service(AddTwoInts, 'test_add', handle_add)
    print(f"[TEST] Service created with key: {srv.key_expr}")
    
    # Create client node
    client_node = Node('test_client', enable_rosout=False)
    print(f"[TEST] Client node created: {client_node.node_name}")
    
    # Create client
    client = client_node.create_client(AddTwoInts, 'test_add')
    print(f"[TEST] Client created with key: {client.key_expr}")
    
    # Give time for discovery
    print("[TEST] Waiting for discovery...")
    await asyncio.sleep(1.0)
    
    # Make service call
    request = AddTwoInts.Request(a=5, b=7)
    response = await client.call_async(request, timeout=2.0)
    
    # Verify response
    assert response.sum == 12
    
    # Cleanup
    await server_node.adestroy_node()
    await client_node.adestroy_node()


@pytest.mark.asyncio
async def test_service_multiple_calls():
    """Test multiple service calls."""
    
    call_count = 0
    
    def handle_add(request: AddTwoInts.Request) -> AddTwoInts.Response:
        nonlocal call_count
        call_count += 1
        return AddTwoInts.Response(sum=request.a + request.b)
    
    server_node = Node('test_server2', enable_rosout=False)
    srv = server_node.create_service(AddTwoInts, 'test_add2', handle_add)
    
    client_node = Node('test_client2', enable_rosout=False)
    client = client_node.create_client(AddTwoInts, 'test_add2')
    
    await asyncio.sleep(0.5)
    
    # Make multiple calls
    test_cases = [(1, 2), (10, 20), (100, 200)]
    
    for a, b in test_cases:
        request = AddTwoInts.Request(a=a, b=b)
        response = await client.call_async(request, timeout=2.0)
        assert response.sum == a + b
    
    # Verify handler was called correct number of times
    assert call_count == len(test_cases)
    
    await server_node.adestroy_node()
    await client_node.adestroy_node()


@pytest.mark.asyncio
async def test_service_async_handler():
    """Test async service handler."""
    async def async_handler(request: AddTwoInts.Request) -> AddTwoInts.Response:
        """Async handler with I/O simulation."""
        await asyncio.sleep(0.05)  # Simulate async I/O
        return AddTwoInts.Response(sum=request.a + request.b)
    
    async with Node('async_server', enable_rosout=False) as server_node:
        service = server_node.create_service(AddTwoInts, 'add_async', async_handler)
        
        async with Node('async_client', enable_rosout=False) as client_node:
            client = client_node.create_client(AddTwoInts, 'add_async')
            
            await asyncio.sleep(0.3)
            
            # Test async handler
            response = await client.call_async(AddTwoInts.Request(a=10, b=20), timeout=5.0)
            assert response.sum == 30


@pytest.mark.asyncio
async def test_service_async_concurrent():
    """Test concurrent async service calls."""
    call_count = 0
    
    async def async_handler(request: AddTwoInts.Request) -> AddTwoInts.Response:
        """Async handler that simulates I/O."""
        nonlocal call_count
        call_count += 1
        await asyncio.sleep(0.05)
        return AddTwoInts.Response(sum=request.a + request.b)
    
    async with Node('async_server', enable_rosout=False) as server_node:
        service = server_node.create_service(AddTwoInts, 'add_async', async_handler)
        
        async with Node('async_client', enable_rosout=False) as client_node:
            client = client_node.create_client(AddTwoInts, 'add_async')
            
            await asyncio.sleep(0.3)
            
            # Make concurrent calls
            tasks = [
                client.call_async(AddTwoInts.Request(a=i, b=i*2), timeout=5.0)
                for i in range(3)
            ]
            responses = await asyncio.gather(*tasks)
            
            assert len(responses) == 3
            for i, response in enumerate(responses):
                assert response.sum == i + i*2
            
            assert call_count == 3


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

