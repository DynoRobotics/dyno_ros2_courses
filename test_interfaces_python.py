#!/usr/bin/env python3
"""
Test the new interfaces_python structure
"""

def test_interfaces_python():
    print("Testing new interfaces_python structure...")
    
    # Test importing from interfaces_python.msg
    from ros2_zenoh_python.interfaces_python.msg import Vector3, Twist, Header, String
    from ros2_zenoh_python.interfaces_python import Vector3 as Vector3FromInterfaces, Twist as TwistFromInterfaces
    
    # Test creating messages
    v3 = Vector3(x=1.0, y=2.0, z=3.0)
    print(f"Vector3: {v3}")
    
    twist = Twist(linear=Vector3(x=1.0), angular=Vector3(z=0.5))
    print(f"Twist: linear={twist.linear}, angular={twist.angular}")
    
    header = Header()
    print(f"Header: {header}")
    
    string_msg = String(data="Hello from interfaces_python!")
    print(f"String: {string_msg}")
    
    # Test that both import paths work
    assert Vector3 == Vector3FromInterfaces
    assert Twist == TwistFromInterfaces
    
    print("✅ All interfaces_python imports work correctly!")
    print("✅ Structure: interfaces_python/msg/{package}.py")
    print("✅ Future ready for: interfaces_python/srv/ and interfaces_python/action/")

if __name__ == '__main__':
    test_interfaces_python()
