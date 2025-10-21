#!/usr/bin/env python3
"""
Test if we can intercept queries/queryables in Zenoh.
This will help us understand if services use queries.
"""

import zenoh
import time


def main():
    config = zenoh.Config()
    config.insert_json5('mode', '"client"')
    config.insert_json5('connect/endpoints', '["tcp/localhost:7447"]')
    
    print("🔗 Connecting to Zenoh...")
    session = zenoh.open(config)
    
    print("📡 Declaring queryable on wildcard (**)...")
    print("   This will intercept ALL queries in the Zenoh network")
    print("   Waiting for queries...\n")
    
    def query_handler(query):
        print(f"❓ Query received!")
        print(f"   Selector: {query.selector()}")
        print(f"   Key: {query.key_expr()}")
        if query.payload():
            print(f"   Payload size: {len(bytes(query.payload()))} bytes")
        print()
        # Don't reply - we're just observing
    
    # Declare queryable for everything
    queryable = session.declare_queryable("**", query_handler)
    
    print("✅ Queryable declared. Now run service client in another terminal.")
    print("   Press Ctrl+C to exit\n")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n✅ Shutting down...")
    finally:
        queryable.undeclare()
        session.close()


if __name__ == '__main__':
    main()


