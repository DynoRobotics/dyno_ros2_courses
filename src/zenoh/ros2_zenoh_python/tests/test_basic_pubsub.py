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

# Import from package
from ros2_zenoh_python import Node
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


class TestBasicPubSub:
    """Basic publisher/subscriber tests."""
    
    @pytest.mark.asyncio
    async def test_single_pubsub(self, shared_zenoh_node):
        """Test single publisher to single subscriber."""
        received_messages = []
        
        async def callback(msg: Twist):
            received_messages.append(msg)
        
        # Create publisher and subscriber
        pub = shared_zenoh_node.create_publisher(Twist, '/test_topic_1')
        sub = shared_zenoh_node.create_subscription(Twist, '/test_topic_1', callback)
        
        try:
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
        finally:
            pub.destroy()
            sub.destroy()
    
    @pytest.mark.asyncio
    async def test_multiple_messages(self, shared_zenoh_node):
        """Test multiple message delivery."""
        received_messages = []
        
        async def callback(msg: Twist):
            received_messages.append(msg)
        
        pub = shared_zenoh_node.create_publisher(Twist, '/test_topic_2')
        sub = shared_zenoh_node.create_subscription(Twist, '/test_topic_2', callback)
        
        try:
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
        finally:
            pub.destroy()
            sub.destroy()
    
    @pytest.mark.asyncio
    async def test_multiple_subscribers(self, shared_zenoh_node):
        """Test single publisher to multiple subscribers."""
        received_1 = []
        received_2 = []
        
        async def callback_1(msg: Twist):
            received_1.append(msg)
        
        async def callback_2(msg: Twist):
            received_2.append(msg)
        
        pub = shared_zenoh_node.create_publisher(Twist, '/test_topic_3')
        sub1 = shared_zenoh_node.create_subscription(Twist, '/test_topic_3', callback_1)
        sub2 = shared_zenoh_node.create_subscription(Twist, '/test_topic_3', callback_2)
        
        try:
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
        finally:
            pub.destroy()
            sub1.destroy()
            sub2.destroy()


class TestGeneratedInterfaces:
    """Test generated ros2_interfaces_py package."""
    
    @pytest.mark.asyncio
    async def test_std_msgs_string(self, shared_zenoh_node):
        """Test std_msgs/String from generated package."""
        try:
            from ros2_zenoh_python._bundled_msgs.std_msgs.msg.string import String
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        received_messages = []
        
        async def callback(msg: String):
            received_messages.append(msg)
        
        pub = shared_zenoh_node.create_publisher(String, '/test_string_topic')
        sub = shared_zenoh_node.create_subscription(String, '/test_string_topic', callback)
        
        try:
            await asyncio.sleep(0.01)
            
            # Publish test messages
            test_msg = String(data="Hello, ROS2!")
            pub.publish(test_msg)
            
            assert await wait_for_condition(lambda: len(received_messages) >= 1, timeout=1.0)
            assert received_messages[0].data == "Hello, ROS2!"
        finally:
            pub.destroy()
            sub.destroy()
    
    @pytest.mark.asyncio
    async def test_std_msgs_int32(self, shared_zenoh_node):
        """Test std_msgs/Int32 from generated package."""
        try:
            from ros2_zenoh_python._bundled_msgs.std_msgs.msg.int32 import Int32
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        received_messages = []
        
        async def callback(msg: Int32):
            received_messages.append(msg)
        
        pub = shared_zenoh_node.create_publisher(Int32, '/test_int_topic')
        sub = shared_zenoh_node.create_subscription(Int32, '/test_int_topic', callback)
        
        try:
            await asyncio.sleep(0.01)
            
            # Test various integer values
            for val in [0, 42, -100, 2147483647]:
                pub.publish(Int32(data=val))
            
            assert await wait_for_condition(lambda: len(received_messages) >= 4, timeout=1.0)
            assert [m.data for m in received_messages] == [0, 42, -100, 2147483647]
        finally:
            pub.destroy()
            sub.destroy()
    
    @pytest.mark.asyncio
    async def test_geometry_msgs_pose(self, shared_zenoh_node):
        """Test geometry_msgs/Pose with nested messages."""
        try:
            from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.pose import Pose
            from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.point import Point
            from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.quaternion import Quaternion
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        received_messages = []
        
        async def callback(msg: Pose):
            received_messages.append(msg)
        
        pub = shared_zenoh_node.create_publisher(Pose, '/test_pose_topic')
        sub = shared_zenoh_node.create_subscription(Pose, '/test_pose_topic', callback)
        
        try:
            await asyncio.sleep(0.01)
            
            # Test nested message structure
            test_msg = Pose(
                position=Point(x=1.0, y=2.0, z=3.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
            pub.publish(test_msg)
            
            assert await wait_for_condition(lambda: len(received_messages) >= 1, timeout=1.0)
            msg = received_messages[0]
            assert msg.position.x == 1.0
            assert msg.position.y == 2.0
            assert msg.position.z == 3.0
            assert msg.orientation.w == 1.0
        finally:
            pub.destroy()
            sub.destroy()
    
    @pytest.mark.asyncio
    async def test_generated_twist_matches_bundled(self, shared_zenoh_node):
        """Test that generated Twist has same hash as bundled Twist."""
        try:
            from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.twist import Twist as GeneratedTwist
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        # Both should have same hash
        bundled_hash = Twist.TYPE_HASH
        generated_hash = GeneratedTwist.TYPE_HASH
        
        assert bundled_hash == generated_hash, \
            f"Hash mismatch: bundled={bundled_hash}, generated={generated_hash}"
        
        # Should be the correct ROS2 hash
        expected_hash = "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a"
        assert generated_hash == expected_hash


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

