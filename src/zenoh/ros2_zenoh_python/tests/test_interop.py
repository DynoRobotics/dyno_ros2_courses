#!/usr/bin/env python3
"""
Interoperability tests for ros2_zenoh_python

Optional tests that check:
- rclpy interop (if available)
- Standard ROS2 message compatibility
- Cross-implementation communication
"""

import asyncio
import pytest
import zenoh

# Import from package
from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.twist import Twist
from ros2_zenoh_python._bundled_msgs.geometry_msgs.msg.vector3 import Vector3

# Try to import rclpy
try:
    import rclpy
    from geometry_msgs.msg import Twist as RclpyTwist
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    pytestmark = pytest.mark.skip("rclpy not available")


class TestInterop:
    """Interoperability tests with rclpy."""
    
    @pytest.mark.interop
    @pytest.mark.skipif(not RCLPY_AVAILABLE, reason="rclpy not available")
    @pytest.mark.asyncio
    async def test_zenoh_to_rclpy(self, zenoh_session_client):
        """Test ros2_zenoh_python publisher to rclpy subscriber."""
        if not RCLPY_AVAILABLE:
            pytest.skip("rclpy not available")
        
        received_messages = []
        
        def rclpy_callback(msg):
            received_messages.append(msg)
        
        # Initialize rclpy
        rclpy.init()
        
        try:
            # Create rclpy node and subscriber
            rclpy_node = rclpy.create_node('rclpy_subscriber')
            rclpy_sub = rclpy_node.create_subscription(
                RclpyTwist, 
                '/test_interop', 
                rclpy_callback, 
                10
            )
            
            # Create zenoh publisher with client session
            async with Node('zenoh_publisher', zenoh_session=zenoh_session_client) as zenoh_node:
                zenoh_pub = zenoh_node.create_publisher(Twist, '/test_interop')
                
                # Wait for discovery
                await asyncio.sleep(1.0)
                
                # Publish from zenoh
                test_msg = Twist(
                    linear=Vector3(x=1.0, y=2.0, z=3.0),
                    angular=Vector3(x=0.1, y=0.2, z=0.3)
                )
                zenoh_pub.publish(test_msg)
                
                # Spin rclpy to receive
                for _ in range(10):
                    rclpy.spin_once(rclpy_node, timeout_sec=0.1)
                    if received_messages:
                        break
                
                # Verify
                assert len(received_messages) > 0, "rclpy did not receive message from zenoh"
                msg = received_messages[0]
                assert abs(msg.linear.x - 1.0) < 0.001
                assert abs(msg.linear.y - 2.0) < 0.001
                assert abs(msg.angular.z - 0.3) < 0.001
                
                print("\n✓ Zenoh → rclpy interop works!")
        
        finally:
            rclpy_node.destroy_node()
            rclpy.shutdown()
    
    @pytest.mark.interop
    @pytest.mark.skipif(not RCLPY_AVAILABLE, reason="rclpy not available")
    @pytest.mark.asyncio
    async def test_rclpy_to_zenoh(self, zenoh_session_client):
        """Test rclpy publisher to ros2_zenoh_python subscriber."""
        if not RCLPY_AVAILABLE:
            pytest.skip("rclpy not available")
        
        received_messages = []
        
        async def zenoh_callback(msg: Twist):
            received_messages.append(msg)
        
        # Initialize rclpy
        rclpy.init()
        
        try:
            # Create rclpy node and publisher
            rclpy_node = rclpy.create_node('rclpy_publisher')
            rclpy_pub = rclpy_node.create_publisher(RclpyTwist, '/test_interop', 10)
            
            # Create zenoh subscriber with client session
            async with Node('zenoh_subscriber', zenoh_session=zenoh_session_client) as zenoh_node:
                zenoh_sub = zenoh_node.create_subscription(
                    Twist, 
                    '/test_interop', 
                    zenoh_callback
                )
                
                # Wait for discovery
                await asyncio.sleep(1.0)
                
                # Publish from rclpy
                test_msg = RclpyTwist()
                test_msg.linear.x = 5.0
                test_msg.linear.y = 6.0
                test_msg.linear.z = 7.0
                test_msg.angular.x = 0.5
                test_msg.angular.y = 0.6
                test_msg.angular.z = 0.7
                
                for _ in range(5):
                    rclpy_pub.publish(test_msg)
                    rclpy.spin_once(rclpy_node, timeout_sec=0.01)
                    await asyncio.sleep(0.1)
                
                await asyncio.sleep(0.5)
                
                # Verify
                assert len(received_messages) > 0, "zenoh did not receive message from rclpy"
                msg = received_messages[0]
                assert abs(msg.linear.x - 5.0) < 0.001
                assert abs(msg.linear.y - 6.0) < 0.001
                assert abs(msg.angular.z - 0.7) < 0.001
                
                print("\n✓ rclpy → Zenoh interop works!")
        
        finally:
            rclpy_node.destroy_node()
            rclpy.shutdown()
    
    @pytest.mark.asyncio
    async def test_message_hash_compatibility(self):
        """Test that bundled message type hashes match ROS2."""
        # Test bundled Twist message (already imported at top)
        
        # Check that TYPE_HASH is set and matches expected format
        assert hasattr(Twist, 'TYPE_HASH')
        assert Twist.TYPE_HASH.startswith('RIHS01_')
        assert len(Twist.TYPE_HASH) == 71  # RIHS01_ + 64 hex chars
        
        # Check it matches the known ROS2 hash for Twist
        expected_hash = 'RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a'
        assert Twist.TYPE_HASH == expected_hash
        
        print(f"\n✓ Bundled Twist TYPE_HASH matches ROS2: {Twist.TYPE_HASH}")


class TestGeneratedInterfaceHashes:
    """Test that generated ros2_interfaces_py has correct RIHS01 hashes."""
    
    @pytest.mark.asyncio
    async def test_std_msgs_string_hash(self):
        """Test std_msgs/String has correct hash."""
        try:
            from ros2_interfaces_py.std_msgs.msg.string import String
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        # Verify hash format
        assert hasattr(String, 'TYPE_HASH')
        assert String.TYPE_HASH.startswith('RIHS01_')
        assert len(String.TYPE_HASH) == 71
        
        print(f"\n✓ String TYPE_HASH: {String.TYPE_HASH}")
    
    @pytest.mark.asyncio
    async def test_geometry_msgs_twist_hash(self):
        """Test generated Twist has correct hash."""
        try:
            from ros2_interfaces_py.geometry_msgs.msg.twist import Twist as GeneratedTwist
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        # Should match bundled version
        assert GeneratedTwist.TYPE_HASH == Twist.TYPE_HASH
        
        # Should match known ROS2 hash
        expected = 'RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a'
        assert GeneratedTwist.TYPE_HASH == expected
        
        print(f"\n✓ Generated Twist matches ROS2 hash: {GeneratedTwist.TYPE_HASH}")
    
    @pytest.mark.asyncio
    async def test_builtin_interfaces_time_hash(self):
        """Test builtin_interfaces/Time has correct hash."""
        try:
            from ros2_interfaces_py.builtin_interfaces.msg.time import Time
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        assert hasattr(Time, 'TYPE_HASH')
        assert Time.TYPE_HASH.startswith('RIHS01_')
        
        print(f"\n✓ Time TYPE_HASH: {Time.TYPE_HASH}")
    
    @pytest.mark.asyncio
    async def test_sensor_msgs_image_hash(self):
        """Test sensor_msgs/Image has correct hash (complex nested message)."""
        try:
            from ros2_interfaces_py.sensor_msgs.msg.image import Image
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed or essential preset doesn't include sensor_msgs")
        
        assert hasattr(Image, 'TYPE_HASH')
        assert Image.TYPE_HASH.startswith('RIHS01_')
        
        # Image is complex with nested Header and arrays
        print(f"\n✓ Image TYPE_HASH: {Image.TYPE_HASH}")
    
    @pytest.mark.asyncio
    async def test_all_messages_have_dds_type_name(self):
        """Test that all generated messages have DDS_TYPE_NAME."""
        try:
            from ros2_interfaces_py.std_msgs.msg.string import String
            from ros2_interfaces_py.geometry_msgs.msg.twist import Twist as GeneratedTwist
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        # All messages should have DDS_TYPE_NAME
        assert hasattr(String, 'DDS_TYPE_NAME')
        assert String.DDS_TYPE_NAME == 'std_msgs::msg::dds_::String_'
        
        assert hasattr(GeneratedTwist, 'DDS_TYPE_NAME')
        assert GeneratedTwist.DDS_TYPE_NAME == 'geometry_msgs::msg::dds_::Twist_'
        
        print("\n✓ All messages have correct DDS_TYPE_NAME")
    
    @pytest.mark.asyncio
    async def test_serialization_roundtrip(self):
        """Test serialize/deserialize roundtrip for generated messages."""
        try:
            from ros2_interfaces_py.std_msgs.msg.string import String
            from ros2_interfaces_py.std_msgs.msg.int32 import Int32
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        # Test String
        original_str = String(data="Test message 🚀")
        serialized = original_str.serialize()
        deserialized = String.deserialize(serialized)
        assert deserialized.data == original_str.data
        
        # Test Int32
        original_int = Int32(data=42)
        serialized = original_int.serialize()
        deserialized = Int32.deserialize(serialized)
        assert deserialized.data == original_int.data
        
        print("\n✓ Serialization roundtrip works for generated messages")
    
    @pytest.mark.interop
    @pytest.mark.skipif(not RCLPY_AVAILABLE, reason="rclpy not available")
    @pytest.mark.asyncio
    async def test_generated_message_cross_compat(self, zenoh_session_client):
        """Test generated message can communicate with rclpy."""
        if not RCLPY_AVAILABLE:
            pytest.skip("rclpy not available")
        
        try:
            from ros2_interfaces_py.std_msgs.msg.string import String
        except ImportError:
            pytest.skip("ros2_interfaces_py not installed")
        
        from std_msgs.msg import String as RclpyString
        
        received_messages = []
        
        def rclpy_callback(msg):
            received_messages.append(msg)
        
        rclpy.init()
        
        try:
            rclpy_node = rclpy.create_node('test_rclpy_string')
            rclpy_sub = rclpy_node.create_subscription(
                RclpyString, '/test_cross_compat_string', rclpy_callback, 10
            )
            
            async with Node('test_zenoh_string', zenoh_session=zenoh_session_client, enable_rosout=False) as node:
                pub = node.create_publisher(String, '/test_cross_compat_string')
                
                await asyncio.sleep(0.1)
                
                # Publish from zenoh using generated message
                for i in range(3):
                    msg = String(data=f"Message {i}")
                    pub.publish(msg)
                    rclpy.spin_once(rclpy_node, timeout_sec=0.01)
                    await asyncio.sleep(0.1)
                
                await asyncio.sleep(0.5)
                
                # Verify rclpy received
                assert len(received_messages) > 0, "rclpy did not receive messages from zenoh"
                assert received_messages[0].data.startswith("Message")
                
                print(f"\n✓ Generated message cross-compatibility works! Received: {received_messages[0].data}")
        
        finally:
            rclpy_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

