#!/usr/bin/env python3
"""
Test: rclpy publisher → ros2_zenoh_python subscriber (Arrays message interop)
"""
import asyncio
import sys
from pathlib import Path

repo_root = Path('src/zenoh/ros2_zenoh_python')
sys.path.insert(0, str(repo_root))

from ros2_zenoh_python import Node
from ros2_zenoh_python._bundled_msgs import test_msgs

Arrays = test_msgs.msg.Arrays

received = []

def callback(msg):
    print(f"📥 Received: int32_values={msg.int32_values}, alignment_check={msg.alignment_check}")
    received.append(msg)

async def main():
    # Create subscriber first
    async with Node('arrays_subscriber') as node:
        sub = node.create_subscription(Arrays, '/test_arrays', callback)
        print("✅ Subscribed to /test_arrays")
        print("⏳ Waiting for messages from rclpy publisher...")
        print("   (Run: ros2 topic pub /test_arrays test_msgs/msg/Arrays...")
        print("    Or run rclpy publisher in another terminal)")
        
        # Wait for messages
        for i in range(10):
            await asyncio.sleep(1)
            if received:
                print(f"\n✅ Got {len(received)} message(s)!")
                msg = received[0]
                if msg.alignment_check == 42:
                    print("✅ ALIGNMENT CHECK PASSED!")
                    return True
                break
        
        if not received:
            print("\n⚠️  No messages received")
        return False

if __name__ == "__main__":
    result = asyncio.run(main())
    sys.exit(0 if result else 1)


