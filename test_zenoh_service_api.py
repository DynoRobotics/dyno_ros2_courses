#!/usr/bin/env python3
"""Test Zenoh service/query API to understand the correct usage."""

import zenoh
import time

# Create Zenoh session
config = zenoh.Config()
session = zenoh.open(config)

print("Testing Zenoh query/queryable API...")

# Create queryable (server)
def handle_query(query):
    print(f"[QUERYABLE] Received query:")
    print(f"  Key: {query.key_expr}")
    print(f"  Payload: {query.payload}")
    print(f"  Attachment: {query.attachment}")
    
    # Try to send reply
    response_payload = b"Hello from server"
    print(f"[QUERYABLE] Sending reply...")
    try:
        # Method 1: Just key and payload
        query.reply(query.key_expr, response_payload)
        print(f"[QUERYABLE] Reply sent successfully (method 1)")
    except Exception as e:
        print(f"[QUERYABLE] Method 1 error: {e}")
        try:
            # Method 2: Use session.put
            query.reply_ok(response_payload)
            print(f"[QUERYABLE] Reply sent successfully (method 2)")
        except Exception as e2:
            print(f"[QUERYABLE] Method 2 error: {e2}")

queryable = session.declare_queryable("test/service", handle_query)
print(f"Queryable declared on: test/service")

# Give time for queryable to be ready
time.sleep(0.5)

# Send query (client)
print("\n[CLIENT] Sending query...")
replies = session.get("test/service", payload=b"Hello from client", timeout=2.0)

print(f"[CLIENT] Waiting for replies...")
count = 0
for reply in replies:
    count += 1
    print(f"[CLIENT] Got reply #{count}: {reply}")
    if hasattr(reply, 'ok'):
        print(f"[CLIENT] Reply OK: {reply.ok}")
        if reply.ok:
            print(f"[CLIENT] Payload: {bytes(reply.ok.payload)}")

print(f"[CLIENT] Total replies: {count}")

# Cleanup
queryable.undeclare()
session.close()
print("\n✅ Test complete")

