#!/usr/bin/env python3
"""
Basic pub/sub tests for ros2_zenoh_python

Tests basic functionality including:
- Single publisher to single subscriber
- Message delivery
- Multiple subscribers
- Basic timing
"""

import asyncio
import pytest
import sys
from pathlib import Path

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


class TestBasicPubSub:
    """Basic publisher/subscriber tests."""
    
    @pytest.mark.asyncio
    async def test_single_pubsub(self, zenoh_session):
        """Test single publisher to single subscriber."""
        received_messages = []
        
        async def callback(msg: Twist):
            received_messages.append(msg)
        
        # Create nodes with shared session
        async with Node('test_publisher_1', zenoh_session=zenoh_session, enable_rosout=False) as pub_node:
            async with Node('test_subscriber_1', zenoh_session=zenoh_session, enable_rosout=False) as sub_node:
                # Create publisher and subscriber
                pub = pub_node.create_publisher(Twist, '/test_topic_1')
                sub = sub_node.create_subscription(Twist, '/test_topic_1', callback)
                
                # Tiny delay for Zenoh discovery (10ms is enough!)
                await asyncio.sleep(0.01)
                
                # Publish test message
                test_msg = Twist(
                    linear=Vector3(x=1.0, y=2.0, z=3.0),
                    angular=Vector3(x=0.1, y=0.2, z=0.3)
                )
                pub.publish(test_msg)
                
                # Wait for message with timeout
                assert await wait_for_condition(lambda: len(received_messages) >= 1, timeout=1.0)
                
                # Verify
                assert len(received_messages) == 1
                assert received_messages[0].linear.x == 1.0
                assert received_messages[0].linear.y == 2.0
                assert received_messages[0].angular.z == 0.3
    
    @pytest.mark.asyncio
    async def test_multiple_messages(self, zenoh_session):
        """Test multiple message delivery."""
        received_messages = []
        
        async def callback(msg: Twist):
            received_messages.append(msg)
        
        async with Node('test_publisher_2', zenoh_session=zenoh_session, enable_rosout=False) as pub_node:
            async with Node('test_subscriber_2', zenoh_session=zenoh_session, enable_rosout=False) as sub_node:
                pub = pub_node.create_publisher(Twist, '/test_topic_2')
                sub = sub_node.create_subscription(Twist, '/test_topic_2', callback)
                
                await asyncio.sleep(0.01)
                
                # Publish 5 messages
                for i in range(5):
                    msg = Twist(
                        linear=Vector3(x=float(i), y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0)
                    )
                    pub.publish(msg)
                
                # Wait for all messages
                assert await wait_for_condition(lambda: len(received_messages) >= 5, timeout=1.0)
                
                # Verify all messages received
                assert len(received_messages) == 5
                for i, msg in enumerate(received_messages):
                    assert msg.linear.x == float(i)
    
    @pytest.mark.asyncio
    async def test_multiple_subscribers(self, zenoh_session):
        """Test single publisher to multiple subscribers."""
        received_1 = []
        received_2 = []
        
        async def callback_1(msg: Twist):
            received_1.append(msg)
        
        async def callback_2(msg: Twist):
            received_2.append(msg)
        
        async with Node('test_publisher_3', zenoh_session=zenoh_session, enable_rosout=False) as pub_node:
            async with Node('test_subscriber_3a', zenoh_session=zenoh_session, enable_rosout=False) as sub_node_1:
                async with Node('test_subscriber_3b', zenoh_session=zenoh_session, enable_rosout=False) as sub_node_2:
                    pub = pub_node.create_publisher(Twist, '/test_topic_3')
                    sub1 = sub_node_1.create_subscription(Twist, '/test_topic_3', callback_1)
                    sub2 = sub_node_2.create_subscription(Twist, '/test_topic_3', callback_2)
                    
                    await asyncio.sleep(0.01)
                    
                    # Publish message
                    test_msg = Twist(
                        linear=Vector3(x=5.0, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0)
                    )
                    pub.publish(test_msg)
                    
                    # Wait for both subscribers
                    assert await wait_for_condition(lambda: len(received_1) >= 1 and len(received_2) >= 1, timeout=1.0)
                    
                    # Both subscribers should receive
                    assert len(received_1) == 1
                    assert len(received_2) == 1
                    assert received_1[0].linear.x == 5.0
                    assert received_2[0].linear.x == 5.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

