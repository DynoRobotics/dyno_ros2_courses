#!/usr/bin/env python3
"""
List Zenoh queryables (which services might use).
"""

import zenoh
import argparse


def main():
    parser = argparse.ArgumentParser(description='List Zenoh queryables')
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
    
    print("\n🔍 Querying for queryables (sending query to **)...")
    print("=" * 80)
    
    # Send a query to ** and see what responds
    replies = session.get("**", timeout=1.0)
    
    queryable_keys = set()
    for reply in replies:
        try:
            # Extract key from reply
            if hasattr(reply, 'ok') and reply.ok:
                key = str(reply.ok.key_expr)
                queryable_keys.add(key)
        except Exception as e:
            print(f"   Error processing reply: {e}")
    
    if not queryable_keys:
        print("⚠️  No queryables found responding to wildcard query")
    else:
        print(f"✅ Found {len(queryable_keys)} queryables:\n")
        for key in sorted(queryable_keys):
            print(f"   {key}")
    
    session.close()
    print("\n✅ Done")


if __name__ == '__main__':
    main()


