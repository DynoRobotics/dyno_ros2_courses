#!/usr/bin/env python3
"""
Timing and latency tests for ros2_zenoh_python

Tests timing characteristics including:
- Message latency
- Throughput
- Multi-node communication timing
"""

import asyncio
import pytest
import sys
import time
from pathlib import Path
from typing import List, Tuple

# Add unified_output to path for ros2_interfaces_py
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "tools" / "unified_output" / "python"))

# Import from parent package (ros2_zenoh_python)
from .. import Node
from ros2_interfaces_py.geometry_msgs.msg.twist import Twist
from ros2_interfaces_py.geometry_msgs.msg.vector3 import Vector3


async def wait_for_condition(condition_fn, timeout=2.0, check_interval=0.01):
    """Wait for a condition to be true, with timeout."""
    elapsed = 0.0
    while elapsed < timeout:
        if condition_fn():
            return True
        await asyncio.sleep(check_interval)
        elapsed += check_interval
    return False


class TestTiming:
    """Timing and latency tests."""
    
    @pytest.mark.asyncio
    async def test_message_latency(self):
        """Test message delivery latency."""
        latencies: List[float] = []
        
        async def callback(msg: Twist):
            # Calculate latency (timestamp stored in linear.x)
            send_time = msg.linear.x
            recv_time = time.time()
            latency = recv_time - send_time
            latencies.append(latency)
        
        async with Node('test_publisher') as pub_node:
            async with Node('test_subscriber') as sub_node:
                pub = pub_node.create_publisher(Twist, '/test_topic')
                sub = sub_node.create_subscription(Twist, '/test_topic', callback)
                
                await asyncio.sleep(0.01)
                
                # Send 10 messages with timestamps
                for _ in range(10):
                    msg = Twist(
                        linear=Vector3(x=time.time(), y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0)
                    )
                    pub.publish(msg)
                    await asyncio.sleep(0.01)
                
                # Wait for all messages
                assert await wait_for_condition(lambda: len(latencies) >= 10, timeout=2.0)
                
                # Verify latencies
                assert len(latencies) == 10
                avg_latency = sum(latencies) / len(latencies)
                max_latency = max(latencies)
                
                print(f"\nLatency stats:")
                print(f"  Average: {avg_latency*1000:.2f} ms")
                print(f"  Max: {max_latency*1000:.2f} ms")
                print(f"  Min: {min(latencies)*1000:.2f} ms")
                
                # Sanity checks (latency should be < 100ms on localhost)
                assert avg_latency < 0.1
                assert max_latency < 0.2
    
    @pytest.mark.asyncio
    async def test_throughput(self):
        """Test message throughput."""
        received_count = [0]
        start_time = [None]
        end_time = [None]
        
        async def callback(msg: Twist):
            if received_count[0] == 0:
                start_time[0] = time.time()
            received_count[0] += 1
            if received_count[0] == 100:
                end_time[0] = time.time()
        
        async with Node('test_publisher') as pub_node:
            async with Node('test_subscriber') as sub_node:
                pub = pub_node.create_publisher(Twist, '/test_topic')
                sub = sub_node.create_subscription(Twist, '/test_topic', callback)
                
                await asyncio.sleep(0.01)
                
                # Publish 100 messages as fast as possible
                msg = Twist(
                    linear=Vector3(x=1.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                for _ in range(100):
                    pub.publish(msg)
                
                # Wait for all messages
                assert await wait_for_condition(lambda: received_count[0] >= 100, timeout=2.0)
                
                # Calculate throughput
                assert received_count[0] == 100
                duration = end_time[0] - start_time[0]
                throughput = 100 / duration
                
                print(f"\nThroughput stats:")
                print(f"  Messages: {received_count[0]}")
                print(f"  Duration: {duration:.3f} s")
                print(f"  Throughput: {throughput:.1f} msg/s")
                
                # Sanity check (should handle >100 msg/s)
                assert throughput > 100
    
    @pytest.mark.asyncio
    async def test_multi_node_timing(self, zenoh_session):
        """Test timing with multiple publisher and subscriber nodes."""
        received_by_node: dict = {
            'sub1': [],
            'sub2': [],
            'sub3': []
        }
        
        async def make_callback(node_id: str):
            async def callback(msg: Twist):
                recv_time = time.time()
                send_time = msg.linear.x
                received_by_node[node_id].append((send_time, recv_time))
            return callback
        
        async with Node('pub1', zenoh_session=zenoh_session, enable_rosout=False) as pub1:
            async with Node('pub2', zenoh_session=zenoh_session, enable_rosout=False) as pub2:
                async with Node('sub1', zenoh_session=zenoh_session, enable_rosout=False) as sub1:
                    async with Node('sub2', zenoh_session=zenoh_session, enable_rosout=False) as sub2:
                        async with Node('sub3', zenoh_session=zenoh_session, enable_rosout=False) as sub3:
                            # Create publishers
                            p1 = pub1.create_publisher(Twist, '/test_topic')
                            p2 = pub2.create_publisher(Twist, '/test_topic')
                            
                            # Create subscribers
                            s1 = sub1.create_subscription(
                                Twist, '/test_topic', await make_callback('sub1')
                            )
                            s2 = sub2.create_subscription(
                                Twist, '/test_topic', await make_callback('sub2')
                            )
                            s3 = sub3.create_subscription(
                                Twist, '/test_topic', await make_callback('sub3')
                            )
                            
                            await asyncio.sleep(0.01)
                            
                            # Each publisher sends 5 messages
                            for i in range(5):
                                msg = Twist(
                                    linear=Vector3(x=time.time(), y=float(i), z=0.0),
                                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                                )
                                p1.publish(msg)
                                p2.publish(msg)
                                await asyncio.sleep(0.01)
                            
                            # Wait for all 3 subscribers to receive at least 5 messages each
                            assert await wait_for_condition(
                                lambda: (len(received_by_node.get('sub1', [])) >= 5 and 
                                         len(received_by_node.get('sub2', [])) >= 5 and 
                                         len(received_by_node.get('sub3', [])) >= 5),
                                timeout=2.0
                            )
                            
                            # Verify all subscribers got all messages from both publishers
                            for node_id, messages in received_by_node.items():
                                assert len(messages) == 10, f"{node_id} got {len(messages)} messages"
                                
                                # Calculate average latency for this subscriber
                                latencies = [recv - send for send, recv in messages]
                                avg_latency = sum(latencies) / len(latencies)
                                print(f"\n{node_id} avg latency: {avg_latency*1000:.2f} ms")


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

