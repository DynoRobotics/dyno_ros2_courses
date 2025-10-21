#!/usr/bin/env python3
"""
Subscribe to all Zenoh keys to see what's being published.
"""

import zenoh
import time


def main():
    config = zenoh.Config()
    config.insert_json5('mode', '"client"')
    config.insert_json5('connect/endpoints', '["tcp/localhost:7447"]')
    
    print("🔗 Connecting to Zenoh...")
    session = zenoh.open(config)
    
    print("📡 Subscribing to all keys (**)...")
    print("   Waiting for data...\n")
    
    seen_keys = set()
    
    def callback(sample):
        key = str(sample.key_expr)
        if key not in seen_keys:
            seen_keys.add(key)
            print(f"📨 New key: {key}")
            print(f"   Payload size: {len(bytes(sample.payload))} bytes")
            print()
    
    # Subscribe to everything
    sub = session.declare_subscriber("**", callback)
    
    try:
        # Keep running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n✅ Shutting down...")
    finally:
        sub.undeclare()
        session.close()


if __name__ == '__main__':
    main()


