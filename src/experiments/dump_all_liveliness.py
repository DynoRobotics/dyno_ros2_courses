#!/usr/bin/env python3
"""
Dump ALL liveliness tokens with full details.
"""

import zenoh


def main():
    config = zenoh.Config()
    config.insert_json5('mode', '"client"')
    config.insert_json5('connect/endpoints', '["tcp/localhost:7447"]')
    
    print("🔗 Connecting...")
    session = zenoh.open(config)
    
    print("\n📋 ALL Liveliness Tokens (raw output):\n")
    print("=" * 80)
    
    replies = session.liveliness().get("**")
    
    count = 0
    for reply in replies:
        count += 1
        print(f"\n[{count}] Reply object:")
        print(f"    {reply}")
        print()
        
        # Try to extract key in different ways
        reply_str = str(reply)
        if 'key_expr:' in reply_str:
            import re
            match = re.search(r"key_expr:\s*ke`([^`]+)`", reply_str)
            if match:
                key = match.group(1)
                print(f"    Extracted key: {key}")
                
                # Analyze key structure
                if '@adv/' in key:
                    print(f"    → Contains '@adv/' (advertisement)")
                if '/SS/' in key:
                    print(f"    → Contains '/SS/' (SERVICE SERVER!)")
                if '/SC/' in key:
                    print(f"    → Contains '/SC/' (SERVICE CLIENT!)")
                if '/pub/' in key:
                    print(f"    → Contains '/pub/' (publisher)")
                if '/sub/' in key:
                    print(f"    → Contains '/sub/' (subscriber)")
    
    print(f"\n✅ Total tokens: {count}")
    session.close()


if __name__ == '__main__':
    main()


