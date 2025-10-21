#!/usr/bin/env python3
"""
Explore different liveliness keyexpr patterns.
"""

import zenoh


def main():
    config = zenoh.Config()
    config.insert_json5('mode', '"client"')
    config.insert_json5('connect/endpoints', '["tcp/localhost:7447"]')
    
    session = zenoh.open(config)
    
    patterns = [
        "**",
        "**/SS/**",
        "**/SC/**",
        "**/@adv/**",
        "**/service/**",
        "**/srv/**",
        "**add_two_ints**",
    ]
    
    for pattern in patterns:
        print(f"\n🔍 Trying pattern: '{pattern}'")
        print("=" * 60)
        try:
            replies = session.liveliness().get(pattern)
            count = 0
            for reply in replies:
                count += 1
                reply_str = str(reply)
                if 'key_expr:' in reply_str:
                    import re
                    match = re.search(r"key_expr:\s*ke`([^`]+)`", reply_str)
                    if match:
                        key = match.group(1)
                        if 'rosout' not in key:  # Skip rosout
                            print(f"   {key}")
            if count == 0:
                print("   (no matches)")
            elif all('rosout' in str(r) for r in replies):
                print(f"   (only rosout, {count} tokens)")
        except Exception as e:
            print(f"   Error: {e}")
    
    session.close()


if __name__ == '__main__':
    main()


