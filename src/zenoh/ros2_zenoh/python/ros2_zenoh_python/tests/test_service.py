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
async def test_service_basic(shared_zenoh_node):
    """Test basic service server and client communication."""
    import logging
    logging.basicConfig(level=logging.DEBUG)
    
    # Define service handler
    def handle_add(request: AddTwoInts.Request) -> AddTwoInts.Response:
        result = request.a + request.b
        print(f"[SERVER] Received request: {request.a} + {request.b} = {result}")
        return AddTwoInts.Response(sum=result)
    
    # Use shared node for both server and client (no naming conflicts)
    print(f"[TEST] Using shared node: {shared_zenoh_node.node_name}")
    
    # Create service
    srv = shared_zenoh_node.create_service(AddTwoInts, 'test_add', handle_add)
    print(f"[TEST] Service created with key: {srv.key_expr}")
    
    # Create client on same node
    client = shared_zenoh_node.create_client(AddTwoInts, 'test_add')
    print(f"[TEST] Client created with key: {client.key_expr}")
    
    # Wait for service to be ready
    print("[TEST] Waiting for service...")
    assert await client.wait_for_server(timeout=2.0), "Service not found"
    
    # Make service call
    request = AddTwoInts.Request(a=5, b=7)
    response = await client.call_async(request, timeout=2.0)
    
    # Verify response
    assert response.sum == 12
    
    # Cleanup entities (node is reused)
    srv.destroy()
    client.destroy()


@pytest.mark.asyncio
async def test_service_multiple_calls(shared_zenoh_node):
    """Test multiple service calls."""
    
    call_count = 0
    
    def handle_add(request: AddTwoInts.Request) -> AddTwoInts.Response:
        nonlocal call_count
        call_count += 1
        return AddTwoInts.Response(sum=request.a + request.b)
    
    # Use shared node
    srv = shared_zenoh_node.create_service(AddTwoInts, 'test_add2', handle_add)
    client = shared_zenoh_node.create_client(AddTwoInts, 'test_add2')
    
    # Wait for service
    assert await client.wait_for_server(timeout=2.0), "Service not found"
    
    # Make multiple calls
    test_cases = [(1, 2), (10, 20), (100, 200)]
    
    for a, b in test_cases:
        request = AddTwoInts.Request(a=a, b=b)
        response = await client.call_async(request, timeout=2.0)
        assert response.sum == a + b
    
    # Verify handler was called correct number of times
    assert call_count == len(test_cases)
    
    # Cleanup
    srv.destroy()
    client.destroy()


@pytest.mark.asyncio
async def test_service_async_handler(shared_zenoh_node):
    """Test async service handler."""
    async def async_handler(request: AddTwoInts.Request) -> AddTwoInts.Response:
        """Async handler with I/O simulation."""
        await asyncio.sleep(0.05)  # Simulate async I/O
        return AddTwoInts.Response(sum=request.a + request.b)
    
    # Use shared node
    service = shared_zenoh_node.create_service(AddTwoInts, 'add_async', async_handler)
    client = shared_zenoh_node.create_client(AddTwoInts, 'add_async')
    
    # Wait for service
    assert await client.wait_for_server(timeout=2.0), "Service not found"
    
    # Test async handler
    response = await client.call_async(AddTwoInts.Request(a=10, b=20), timeout=5.0)
    assert response.sum == 30
    
    # Cleanup
    service.destroy()
    client.destroy()


@pytest.mark.asyncio
async def test_service_async_concurrent(shared_zenoh_node):
    """Test concurrent async service calls."""
    call_count = 0
    
    async def async_handler(request: AddTwoInts.Request) -> AddTwoInts.Response:
        """Async handler that simulates I/O."""
        nonlocal call_count
        call_count += 1
        await asyncio.sleep(0.05)
        return AddTwoInts.Response(sum=request.a + request.b)
    
    # Use shared node
    service = shared_zenoh_node.create_service(AddTwoInts, 'add_async_concurrent', async_handler)
    client = shared_zenoh_node.create_client(AddTwoInts, 'add_async_concurrent')
    
    # Wait for service
    assert await client.wait_for_server(timeout=2.0), "Service not found"
    
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
    
    # Cleanup
    service.destroy()
    client.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

