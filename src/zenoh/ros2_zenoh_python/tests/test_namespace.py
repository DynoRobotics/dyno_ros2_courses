#!/usr/bin/env python3
"""
Namespace tests for ros2_zenoh_python

Tests namespace handling and topic name resolution.
"""

import asyncio
import pytest

from ros2_zenoh_python import Node, resolve_topic_name, normalize_namespace, get_fqn
from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.twist import Twist
from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.vector3 import Vector3


async def wait_for_condition(condition_fn, timeout=2.0, check_interval=0.01):
    """Wait for a condition to be true, with timeout."""
    elapsed = 0.0
    while elapsed < timeout:
        if condition_fn():
            return True
        await asyncio.sleep(check_interval)
        elapsed += check_interval
    return False


class TestNamespaceUtils:
    """Test namespace utility functions."""
    
    def test_normalize_namespace_empty(self):
        """Test normalizing empty namespace."""
        assert normalize_namespace("") == ""
        assert normalize_namespace("  ") == ""
    
    def test_normalize_namespace_simple(self):
        """Test normalizing simple namespace."""
        assert normalize_namespace("my_robot") == "/my_robot"
        assert normalize_namespace("/my_robot") == "/my_robot"
        assert normalize_namespace("/my_robot/") == "/my_robot"
    
    def test_normalize_namespace_nested(self):
        """Test normalizing nested namespace."""
        assert normalize_namespace("/my_robot/sensors") == "/my_robot/sensors"
        assert normalize_namespace("my_robot/sensors/") == "/my_robot/sensors"
    
    def test_normalize_namespace_double_slashes(self):
        """Test removing double slashes."""
        assert normalize_namespace("//my_robot") == "/my_robot"
        assert normalize_namespace("/my_robot//sensors") == "/my_robot/sensors"
    
    def test_resolve_topic_absolute(self):
        """Test resolving absolute topics."""
        assert resolve_topic_name("/cmd_vel", "/my_robot") == "/cmd_vel"
        assert resolve_topic_name("/cmd_vel", "") == "/cmd_vel"
    
    def test_resolve_topic_relative(self):
        """Test resolving relative topics."""
        assert resolve_topic_name("cmd_vel", "/my_robot") == "/my_robot/cmd_vel"
        assert resolve_topic_name("cmd_vel", "") == "/cmd_vel"
    
    def test_resolve_topic_nested(self):
        """Test resolving nested topics."""
        assert resolve_topic_name("sensors/imu", "/my_robot") == "/my_robot/sensors/imu"
        assert resolve_topic_name("sensors/imu", "") == "/sensors/imu"
    
    def test_get_fqn(self):
        """Test fully qualified node name."""
        assert get_fqn("my_node", "/my_robot") == "/my_robot/my_node"
        assert get_fqn("my_node", "") == "/my_node"


class TestNodeNamespace:
    """Test Node class namespace handling."""
    
    @pytest.mark.asyncio
    async def test_node_namespace(self, zenoh_session):
        """Test node with namespace."""
        async with Node('test_node', zenoh_session=zenoh_session, 
                       namespace="/my_robot", enable_rosout=False) as node:
            assert node.namespace == "/my_robot"
    
    @pytest.mark.asyncio
    async def test_node_namespace_normalization(self, zenoh_session):
        """Test namespace normalization in node."""
        async with Node('test_node', zenoh_session=zenoh_session, 
                       namespace="my_robot/", enable_rosout=False) as node:
            assert node.namespace == "/my_robot"
    
    @pytest.mark.asyncio
    async def test_node_empty_namespace(self, zenoh_session):
        """Test node with empty namespace."""
        async with Node('test_node', zenoh_session=zenoh_session, 
                       namespace="", enable_rosout=False) as node:
            assert node.namespace == ""


class TestPublisherNamespace:
    """Test Publisher namespace handling."""
    
    @pytest.mark.asyncio
    async def test_publisher_absolute_topic(self, zenoh_session):
        """Test publisher with absolute topic."""
        async with Node('test_publisher', zenoh_session=zenoh_session,
                       namespace="/my_robot", enable_rosout=False) as node:
            pub = node.create_publisher(Twist, '/cmd_vel')
            assert pub.topic == '/cmd_vel'  # Absolute topic stays as-is
    
    @pytest.mark.asyncio
    async def test_publisher_relative_topic(self, zenoh_session):
        """Test publisher with relative topic."""
        async with Node('test_publisher', zenoh_session=zenoh_session,
                       namespace="/my_robot", enable_rosout=False) as node:
            pub = node.create_publisher(Twist, 'cmd_vel')
            assert pub.topic == '/my_robot/cmd_vel'  # Relative topic prefixed with namespace
    
    @pytest.mark.asyncio
    async def test_publisher_nested_topic(self, zenoh_session):
        """Test publisher with nested topic."""
        async with Node('test_publisher', zenoh_session=zenoh_session,
                       namespace="/my_robot", enable_rosout=False) as node:
            pub = node.create_publisher(Twist, 'sensors/cmd_vel')
            assert pub.topic == '/my_robot/sensors/cmd_vel'


class TestSubscriptionNamespace:
    """Test Subscription namespace handling."""
    
    @pytest.mark.asyncio
    async def test_subscription_absolute_topic(self, zenoh_session):
        """Test subscription with absolute topic."""
        async with Node('test_subscriber', zenoh_session=zenoh_session,
                       namespace="/my_robot", enable_rosout=False) as node:
            sub = node.create_subscription(Twist, '/cmd_vel', lambda msg: None)
            assert sub.topic == '/cmd_vel'  # Absolute topic stays as-is
    
    @pytest.mark.asyncio
    async def test_subscription_relative_topic(self, zenoh_session):
        """Test subscription with relative topic."""
        async with Node('test_subscriber', zenoh_session=zenoh_session,
                       namespace="/my_robot", enable_rosout=False) as node:
            sub = node.create_subscription(Twist, 'cmd_vel', lambda msg: None)
            assert sub.topic == '/my_robot/cmd_vel'  # Relative topic prefixed with namespace


class TestNamespaceCommunication:
    """Test communication with namespaces."""
    
    @pytest.mark.asyncio
    async def test_same_namespace_relative_topic(self, zenoh_session):
        """Test pub/sub in same namespace with relative topic."""
        received_messages = []
        
        async def callback(msg: Twist):
            received_messages.append(msg)
        
        async with Node('test_pub', zenoh_session=zenoh_session,
                       namespace="/robot1", enable_rosout=False) as pub_node:
            async with Node('test_sub', zenoh_session=zenoh_session,
                           namespace="/robot1", enable_rosout=False) as sub_node:
                # Both use relative topic "cmd_vel" -> resolves to "/robot1/cmd_vel"
                pub = pub_node.create_publisher(Twist, 'cmd_vel')
                sub = sub_node.create_subscription(Twist, 'cmd_vel', callback)
                
                await asyncio.sleep(0.01)
                
                test_msg = Twist(
                    linear=Vector3(x=1.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                pub.publish(test_msg)
                
                assert await wait_for_condition(lambda: len(received_messages) >= 1, timeout=1.0)
                assert received_messages[0].linear.x == 1.0
    
    @pytest.mark.asyncio
    async def test_different_namespace_no_communication(self, zenoh_session):
        """Test pub/sub in different namespaces don't communicate."""
        received_messages = []
        
        async def callback(msg: Twist):
            received_messages.append(msg)
        
        async with Node('test_pub', zenoh_session=zenoh_session,
                       namespace="/robot1", enable_rosout=False) as pub_node:
            async with Node('test_sub', zenoh_session=zenoh_session,
                           namespace="/robot2", enable_rosout=False) as sub_node:
                # Publisher: /robot1/cmd_vel, Subscriber: /robot2/cmd_vel
                pub = pub_node.create_publisher(Twist, 'cmd_vel')
                sub = sub_node.create_subscription(Twist, 'cmd_vel', callback)
                
                await asyncio.sleep(0.01)
                
                test_msg = Twist(
                    linear=Vector3(x=1.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                pub.publish(test_msg)
                
                # Should NOT receive message (different namespaces)
                await asyncio.sleep(0.2)
                assert len(received_messages) == 0
    
    @pytest.mark.asyncio
    async def test_absolute_topic_ignores_namespace(self, zenoh_session):
        """Test absolute topics ignore namespace."""
        received_messages = []
        
        async def callback(msg: Twist):
            received_messages.append(msg)
        
        async with Node('test_pub', zenoh_session=zenoh_session,
                       namespace="/robot1", enable_rosout=False) as pub_node:
            async with Node('test_sub', zenoh_session=zenoh_session,
                           namespace="/robot2", enable_rosout=False) as sub_node:
                # Both use absolute topic "/cmd_vel" -> namespace ignored
                pub = pub_node.create_publisher(Twist, '/cmd_vel')
                sub = sub_node.create_subscription(Twist, '/cmd_vel', callback)
                
                await asyncio.sleep(0.01)
                
                test_msg = Twist(
                    linear=Vector3(x=2.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                )
                pub.publish(test_msg)
                
                # Should receive message (both use absolute topic)
                assert await wait_for_condition(lambda: len(received_messages) >= 1, timeout=1.0)
                assert received_messages[0].linear.x == 2.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

