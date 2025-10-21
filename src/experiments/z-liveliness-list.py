#!/usr/bin/env python3
"""
Introspect Zenoh liveliness tokens to see ROS2 services, publishers, subscribers, etc.
"""

import zenoh
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description='List Zenoh liveliness tokens (ROS2 metadata)')
    parser.add_argument(
        '--mode',
        type=str,
        default='client',
        choices=['peer', 'client'],
        help='Zenoh session mode (default: client)'
    )
    parser.add_argument(
        '--connect',
        type=str,
        nargs='+',
        default=['tcp/localhost:7447'],
        help='Zenoh endpoints to connect to (default: tcp/localhost:7447)'
    )
    args = parser.parse_args()

    # Configure Zenoh session
    config = zenoh.Config()
    config.insert_json5('mode', f'"{args.mode}"')
    if args.mode == 'client':
        endpoints_json = '[' + ', '.join([f'"{ep}"' for ep in args.connect]) + ']'
        config.insert_json5('connect/endpoints', endpoints_json)
    
    print(f"🔗 Connecting to Zenoh in {args.mode} mode...")
    if args.mode == 'client':
        print(f"   Endpoints: {args.connect}")
    
    session = zenoh.open(config)
    
    print("\n🔍 Querying Zenoh liveliness tokens...")
    print("=" * 80)
    
    # Query liveliness with a wildcard to get all tokens
    liveliness_tokens = []
    
    # Use liveliness.get() to query liveliness tokens
    replies = session.liveliness().get("**")
    
    for reply in replies:
        # Extract the key expression from the reply
        if hasattr(reply, 'result') and hasattr(reply.result, 'key_expr'):
            key = str(reply.result.key_expr)
        else:
            # Try to parse from string representation
            reply_str = str(reply)
            if 'key_expr:' in reply_str:
                # Extract key_expr value
                import re
                match = re.search(r"key_expr:\s*ke`([^`]+)`", reply_str)
                if match:
                    key = match.group(1)
                else:
                    key = reply_str
            else:
                key = reply_str
        liveliness_tokens.append(key)
    
    if not liveliness_tokens:
        print("⚠️  No liveliness tokens found")
        print("\nMake sure:")
        print("  1. ROS2 nodes are running with rmw_zenoh")
        print("  2. Zenoh router is running (if in client mode)")
        print("  3. Endpoints are correct")
    else:
        print(f"✅ Found {len(liveliness_tokens)} liveliness tokens:\n")
        
        # Categorize tokens
        publishers = []
        subscribers = []
        services = []
        clients = []
        other = []
        
        for token in sorted(liveliness_tokens):
            # Parse the token to categorize
            # Format patterns from rmw_zenoh documentation:
            # - MP: Message Publisher (topics)
            # - MS: Message Subscriber (topics)
            # - SS: Service Server (services)
            # - SC: Service Client (services)
            if '/pub/' in token or 'MP' in token:  # Message Publisher
                publishers.append(token)
            elif '/sub/' in token or 'MS' in token:  # Message Subscriber
                subscribers.append(token)
            elif '/SS/' in token or 'Service Server' in token:  # Service Server
                services.append(token)
            elif '/SC/' in token or 'Service Client' in token:  # Service Client
                clients.append(token)
            else:
                other.append(token)
        
        if publishers:
            print(f"📤 Publishers ({len(publishers)}):")
            for token in publishers:
                print(f"   {token}")
            print()
        
        if subscribers:
            print(f"📥 Subscribers ({len(subscribers)}):")
            for token in subscribers:
                print(f"   {token}")
            print()
        
        if services:
            print(f"🔧 Service Servers ({len(services)}):")
            for token in services:
                print(f"   {token}")
            print()
        
        if clients:
            print(f"🔌 Service Clients ({len(clients)}):")
            for token in clients:
                print(f"   {token}")
            print()
        
        if other:
            print(f"❓ Other ({len(other)}):")
            for token in other:
                print(f"   {token}")
            print()
    
    session.close()
    print("\n✅ Done")


if __name__ == '__main__':
    main()

