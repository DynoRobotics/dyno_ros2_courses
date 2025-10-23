"""
Test service interoperability with rclpy.

NOTE: These tests require rmw_zenoh bridge to be running for full interop.
They are marked as experimental and may be skipped in CI.
"""

import asyncio
import pytest
import sys
from pathlib import Path

# Add ros2_interfaces_py to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'ros2_interfaces_py'))

from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs.example_interfaces.srv import AddTwoInts

# Check if rclpy is available
try:
    import rclpy
    from example_interfaces.srv import AddTwoInts as RclpyAddTwoInts
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    pytestmark = pytest.mark.skip(reason="rclpy not available")


@pytest.fixture(scope="function")
def rclpy_context():
    """Initialize and shutdown rclpy for each test."""
    if HAS_RCLPY:
        if not rclpy.ok():
            rclpy.init()
        yield
        # Don't shutdown here - let pytest handle it at session end
    else:
        yield


@pytest.mark.skipif(not HAS_RCLPY, reason="rclpy not available")
@pytest.mark.asyncio
async def test_zenoh_client_to_rclpy_server(rclpy_session, zenoh_session_client):
    """Test ros2_zenoh_python client calling rclpy service server.
    
    This test verifies full interoperability with rclpy using service-level type hashes."""
    
    import threading
    import time
    
    # Create rclpy service server
    rclpy_node = rclpy.create_node('rclpy_server_v2')
    
    def handle_add(request, response):
        response.sum = request.a + request.b
        return response
    
    rclpy_service = rclpy_node.create_service(
        RclpyAddTwoInts,
        'test_interop_add_v2',
        handle_add
    )
    
    # Spin rclpy in background thread
    spin_active = threading.Event()
    spin_active.set()
    
    def spin_continuously():
        while spin_active.is_set():
            rclpy.spin_once(rclpy_node, timeout_sec=0.1)
    
    spin_thread = threading.Thread(target=spin_continuously, daemon=True)
    spin_thread.start()
    
    try:
        # Create Zenoh client in pytest-asyncio event loop
        async with Node('zenoh_client_v2', enable_rosout=False, zenoh_session=zenoh_session_client) as node:
            client = node.create_client(AddTwoInts, 'test_interop_add_v2')
            
            # Wait for service server
            assert await client.wait_for_server(timeout=5.0), "Service server not found"
            
            request = AddTwoInts.Request(a=99, b=1)
            response = await client.call_async(request, timeout=5.0)
            
            assert response.sum == 100
            print(f"✅ pytest-asyncio version: 99 + 1 = {response.sum}")
            
    finally:
        spin_active.clear()
        spin_thread.join(timeout=1.0)
        rclpy_service.destroy()
        rclpy_node.destroy_node()


@pytest.mark.skipif(not HAS_RCLPY, reason="rclpy not available")
@pytest.mark.asyncio
async def test_rclpy_client_to_zenoh_server(rclpy_session, zenoh_session_client):
    """Test rclpy client calling ros2_zenoh_python service server.
    
    This test verifies full interoperability with rclpy using service-level type hashes."""
    
    # Create zenoh service server
    def handle_add(request: AddTwoInts.Request) -> AddTwoInts.Response:
        result = AddTwoInts.Response(sum=request.a + request.b)
        return result
    
    zenoh_node = Node('zenoh_server', enable_rosout=False, zenoh_session=zenoh_session_client)
    zenoh_service = zenoh_node.create_service(AddTwoInts, 'test_interop_add2', handle_add)
    
    # Create rclpy client
    rclpy_node = rclpy.create_node('rclpy_client')
    rclpy_client = rclpy_node.create_client(RclpyAddTwoInts, 'test_interop_add2')
    
    # Wait for service (handles discovery)
    if not rclpy_client.wait_for_service(timeout_sec=5.0):
        pytest.fail("Service not available")
    
    # Make service call
    request = RclpyAddTwoInts.Request()
    request.a = 123
    request.b = 456
    
    future = rclpy_client.call_async(request)
    
    # Spin until complete (with timeout)
    start = asyncio.get_event_loop().time()
    while not future.done():
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)
        await asyncio.sleep(0.01)
        if asyncio.get_event_loop().time() - start > 5.0:
            pytest.fail("Service call timed out")
    
    response = future.result()
    
    # Verify response
    assert response.sum == 579
    
    # Cleanup
    rclpy_client.destroy()
    rclpy_node.destroy_node()
    await zenoh_node.adestroy_node()


@pytest.mark.skipif(not HAS_RCLPY, reason="rclpy not available")
@pytest.mark.asyncio
async def test_multiple_zenoh_clients_to_rclpy_server_stress(rclpy_session, zenoh_session_client):
    """Stress test: Multiple concurrent Zenoh clients calling single rclpy server.
    
    This test verifies that multiple Zenoh clients can concurrently call an rclpy
    service server using asyncio.gather()."""
    
    import threading
    import time
    
    # Create rclpy service server
    rclpy_node = rclpy.create_node('rclpy_server_stress')
    
    call_count = 0
    
    def handle_add(request, response):
        nonlocal call_count
        call_count += 1
        response.sum = request.a + request.b
        return response
    
    rclpy_service = rclpy_node.create_service(
        RclpyAddTwoInts,
        'test_stress_add',
        handle_add
    )
    
    # Spin rclpy in background thread
    spin_active = threading.Event()
    spin_active.set()
    
    def spin_continuously():
        while spin_active.is_set():
            rclpy.spin_once(rclpy_node, timeout_sec=0.1)
    
    spin_thread = threading.Thread(target=spin_continuously, daemon=True)
    spin_thread.start()
    
    try:
        # Create multiple Zenoh clients and call concurrently
        num_clients = 10
        
        async def make_service_call(client_id: int):
            """Single client making a service call."""
            async with Node(f'zenoh_client_{client_id}', enable_rosout=False, zenoh_session=zenoh_session_client) as node:
                client = node.create_client(AddTwoInts, 'test_stress_add')
                
                a = client_id
                b = client_id * 10
                request = AddTwoInts.Request(a=a, b=b)
                
                response = await client.call_async(request, timeout=10.0)
                
                expected = a + b
                assert response.sum == expected, f"Client {client_id}: expected {expected}, got {response.sum}"
                print(f"✅ Client {client_id}: {a} + {b} = {response.sum}")
                return response.sum
        
        # Launch all clients concurrently with gather
        print(f"\n🚀 Launching {num_clients} concurrent Zenoh clients...")
        tasks = [make_service_call(i) for i in range(num_clients)]
        results = await asyncio.gather(*tasks)
        
        # Verify all calls succeeded
        assert len(results) == num_clients
        assert call_count == num_clients, f"Expected {num_clients} calls, got {call_count}"
        
        print(f"\n✅ All {num_clients} concurrent calls succeeded!")
        print(f"   Server handled {call_count} requests")
        
    finally:
        spin_active.clear()
        spin_thread.join(timeout=1.0)
        rclpy_service.destroy()
        rclpy_node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])

